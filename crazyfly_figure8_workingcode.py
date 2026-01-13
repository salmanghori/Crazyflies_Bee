# -*- coding: utf-8 -*-
"""
Crazyflie Drone Controller with QTM Motion Capture.

This script connects to a Crazyflie drone and a Qualisys Track Manager (QTM)
motion capture system. It uses the real-time 6-DoF (Degrees of Freedom) data
from QTM as an external position and orientation source for the Crazyflie's
Kalman estimator.

The script performs the following sequence:
1.  Connects to the QTM server and starts streaming data for a specific rigid body.
2.  Connects to the Crazyflie drone.
3.  Configures the Crazyflie to use the external pose data from QTM.
4.  Uploads a pre-defined polynomial trajectory (a figure-eight pattern).
5.  Commands the drone to take off, execute the trajectory, and then land.
6.  Ensures clean disconnection from both systems upon completion or error.

Dependencies:
- qtm_rt
- cflib
- scipy
"""

import asyncio
import math
import time
import xml.etree.cElementTree as ET
from threading import Thread

import cflib.crtp
import qtm_rt
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.mem import MemoryElement, Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator
from qtm_rt import QRTCommandException
from scipy.spatial.transform import Rotation


# ------------- CONFIGURATION -------------
class Config:
    """Static class to hold all configuration parameters."""
    # URI for the Crazyflie drone
    CRAZYFLIE_URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E2')

    # IP address of the QTM server
    QTM_IP = "192.168.0.105"

    # The name of the rigid body in QTM that represents the Crazyflie
    RIGID_BODY_NAME = r'cf8'

    # True: send both position and orientation to the drone's estimator.
    # False: send position only.
    SEND_FULL_POSE = False

    # Standard deviation for the orientation data. A lower value means the
    # estimator trusts the external orientation more. Fine-tune if needed.
    ORIENTATION_STD_DEV = 8.0e-3

    # Euler angle settings for converting QTM's '6deuler' data.
    # QTM provides Roll, Pitch, Yaw. 'ZYX' is a common order for aerospace.
    EULER_ORDER = "ZYX"
    EULER_DEGREES = True  # QTM Euler angles are typically in degrees.


# ------------- TRAJECTORY DEFINITION -------------
# A figure-eight trajectory defined as a list of polynomial segments.
# Each row: [duration, x_coeffs(8), y_coeffs(8), z_coeffs(8), yaw_coeffs(8)]
FIGURE_EIGHT_TRAJ = [
    [1.05, 0, -0, 0, -0, 0.830443, -0.27614, -0.384219, 0.180493, -0, 0, -0, 0, -1.356107, 0.68843, 0.587426, -0.329106, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0.71, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.03431, -0.026417, -0.030049, -0.445604, -0.684403, 0.888433, 1.49363, -1.361618, -0.139316, 0.158875, 0.095799, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0.62, 0.922409, 0.405715, -0.582968, -0.092188, -0.11467, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514, 0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0.7, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722, -0.512011, -0.209623, -0.21871, 0.108797, 0.128756, -0.055461, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0.56, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.03609, 0.078627, 0.450742, -0.385534, -0.954089, 0.128288, 0.44262, 0.05563, -0.060142, -0.076163, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0.56, 0.001062, -0.64627, -0.01256, -0.324065, 0.125327, 0.119738, 0.034567, -0.06313, 0.001593, -1.031457, 0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0.7, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354, -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0.62, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682, 0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0.71, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.00142, 0.005294, 0.28857, 0.87335, -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [1.053185, -0.398611, 0.85051, -0.144007, -0.485368, -0.079781, 0.17633, 0.234482, -0.153567, 0.447039, -0.532729, -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
]


class QtmWrapper(Thread):
    """
    Manages the connection to QTM and streams 6-DoF data in a separate thread.

    This class connects to a QTM server, identifies a specified rigid body,
    and streams its position and orientation. It prefers the '6deuler' component
    for its robustness but can fall back to '6d' if necessary. The received
    pose is passed to a user-defined callback function.
    """

    def __init__(self, body_name: str, qtm_ip: str):
        """
        Initializes the QTM wrapper.

        Args:
            body_name: The name of the rigid body to track in QTM.
            qtm_ip: The IP address of the QTM server.
        """
        super().__init__()
        self.daemon = True
        self.body_name = body_name
        self.qtm_ip = qtm_ip
        self.on_pose_callback = None
        self._connection = None
        self._qtm_6dof_labels = []
        self._stay_open = True
        self.start()

    def close(self):
        """Stops the thread and disconnects from QTM."""
        self._stay_open = False
        self.join()

    def run(self):
        """The main loop of the thread, running an asyncio event loop."""
        asyncio.run(self._life_cycle())

    async def _life_cycle(self):
        """Manages the connection and streaming lifecycle."""
        if await self._connect():
            try:
                while self._stay_open:
                    await asyncio.sleep(1)
            finally:
                await self._close()

    async def _connect(self) -> bool:
        """Establishes a connection to the QTM server and starts streaming."""
        print(f'[QTM] Connecting to {self.qtm_ip}...')
        self._connection = await qtm_rt.connect(self.qtm_ip)
        if self._connection is None:
            print('[QTM] ERROR: Failed to connect to QTM.')
            return False

        await self._discover_body_labels()

        # Prefer '6deuler' for robustness, but fall back to '6d'.
        try:
            await self._connection.stream_frames(
                frames='allframes',
                components=['6deuler'],
                on_packet=self._on_packet_6deuler
            )
            print("[QTM] Streaming '6deuler' data.")
        except QRTCommandException:
            print("[QTM] '6deuler' not supported. Falling back to '6d'.")
            await self._connection.stream_frames(
                frames='allframes',
                components=['6d'],
                on_packet=self._on_packet_6d
            )
            print("[QTM] Streaming '6d' data.")
        return True

    async def _discover_body_labels(self):
        """Fetches the names of all 6-DoF rigid bodies from QTM."""
        try:
            params = await self._connection.get_parameters(parameters=['6d'])
            xml = ET.fromstring(params)
            self._qtm_6dof_labels = [label.text.strip() for label in xml.findall('*/Body/Name')]
            if self.body_name not in self._qtm_6dof_labels:
                print(f"[QTM] WARNING: Body '{self.body_name}' not found. "
                      f"Available bodies: {self._qtm_6dof_labels}")
        except Exception as e:
            print(f"[QTM] WARNING: Could not parse 6d labels ({e}). Will default to the first body.")
            self._qtm_6dof_labels = []

    def _body_index(self) -> int:
        """Returns the index of the desired rigid body."""
        if self._qtm_6dof_labels and self.body_name in self._qtm_6dof_labels:
            return self._qtm_6dof_labels.index(self.body_name)
        return 0  # Default to the first body if not found or labels are missing

    def _on_packet_6deuler(self, packet):
        """Callback for processing '6deuler' data packets."""
        _, bodies = packet.get_6d_euler()
        if not bodies:
            return

        idx = self._body_index()
        if idx >= len(bodies):
            return

        pos_mm, euler_deg = bodies[idx]
        if pos_mm is None or any(math.isnan(v) for v in pos_mm):
            return

        # Convert position from mm to meters
        x, y, z = [p / 1000.0 for p in pos_mm]

        # Convert Euler angles to a 3x3 rotation matrix
        rot_matrix = self._euler_to_rotation_matrix(euler_deg)

        if self.on_pose_callback:
            self.on_pose_callback([x, y, z, rot_matrix])

    @staticmethod
    def _euler_to_rotation_matrix(euler_angles):
        """Converts Euler angles to a rotation matrix using scipy."""
        if euler_angles and not any(math.isnan(v) for v in euler_angles):
            try:
                # QTM order is Roll, Pitch, Yaw. Map to scipy's expected order.
                roll, pitch, yaw = euler_angles
                if Config.EULER_ORDER.upper() == 'ZYX':
                    return Rotation.from_euler('ZYX', [yaw, pitch, roll], degrees=Config.EULER_DEGREES).as_matrix()
                else:
                    return Rotation.from_euler(Config.EULER_ORDER, euler_angles, degrees=Config.EULER_DEGREES).as_matrix()
            except Exception as e:
                print(f"[QTM] Error converting Euler angles: {e}")

        # Return identity matrix as a fallback
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

    def _on_packet_6d(self, packet):
        """Callback for processing '6d' (rotation matrix) data packets."""
        _, bodies = packet.get_6d()
        if not bodies:
            return

        idx = self._body_index()
        if idx >= len(bodies):
            return

        pos_mm, rot = bodies[idx]
        if pos_mm is None or any(math.isnan(v) for v in pos_mm):
            return

        x, y, z = [p / 1000.0 for p in pos_mm]

        # Reshape the flat 9-element rotation matrix from QTM
        r = rot.matrix
        rot_matrix = [[r[0], r[3], r[6]],
                      [r[1], r[4], r[7]],
                      [r[2], r[5], r[8]]]

        if self.on_pose_callback:
            self.on_pose_callback([x, y, z, rot_matrix])

    async def _close(self):
        """Stops the data stream and disconnects from the QTM server."""
        try:
            await self._connection.stream_frames_stop()
        except Exception:
            pass
        if self._connection:
            self._connection.disconnect()
        print("[QTM] Disconnected.")


class CrazyflieController:
    """Handles all interactions with the Crazyflie drone."""

    def __init__(self, cf):
        """
        Initializes the controller with a Crazyflie instance.

        Args:
            cf: An initialized Crazyflie object from cflib.
        """
        self.cf = cf
        self.commander = cf.high_level_commander

    def setup(self):
        """Configures the drone's estimator and controller for external pose input."""
        print("[CF] Setting up estimator and controller...")
        # Set estimator to Kalman filter (2) which can accept external pose
        self.cf.param.set_value('stabilizer.estimator', '2')
        # Set controller to Mellinger (2) for trajectory tracking
        self.cf.param.set_value('stabilizer.controller', '2')
        # Adjust sensitivity to external orientation data
        self.cf.param.set_value('locSrv.extQuatStdDev', Config.ORIENTATION_STD_DEV)

    def upload_trajectory(self, trajectory_id: int, trajectory_data: list) -> float:
        """
        Uploads a polynomial trajectory to the Crazyflie's memory.

        Args:
            trajectory_id: An integer ID to assign to this trajectory.
            trajectory_data: The list of polynomial segments.

        Returns:
            The total duration of the trajectory in seconds.
        """
        print(f"[CF] Uploading trajectory ID {trajectory_id}...")
        mem = self.cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
        if not mem:
            raise RuntimeError("Crazyflie does not support trajectories.")

        mem.trajectory = []
        total_duration = 0
        for row in trajectory_data:
            duration = row[0]
            x = Poly4D.Poly(row[1:9])
            y = Poly4D.Poly(row[9:17])
            z = Poly4D.Poly(row[17:25])
            yaw = Poly4D.Poly(row[25:33])
            mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
            total_duration += duration

        mem.write_data_sync()
        self.commander.define_trajectory(trajectory_id, 0, len(mem.trajectory))
        print(f'[CF] Trajectory uploaded. Total duration: {total_duration:.1f}s')
        return total_duration

    def run_flight_sequence(self, trajectory_id: int, duration: float):
        """
        Executes a pre-defined flight sequence: takeoff, fly trajectory, land.

        Args:
            trajectory_id: The ID of the trajectory to fly.
            duration: The total duration of the trajectory.
        """
        print("[CF] Starting flight sequence...")
        try:
            # Arm the motors
            print("[CF] Arming motors...")
            self.cf.platform.send_arming_request(True)
            time.sleep(1.0)

            self.commander.takeoff(1.0, 2.0)
            time.sleep(3.0)

            print(f"[CF] Starting trajectory {trajectory_id}...")
            relative = True  # Trajectory is relative to the takeoff position
            self.commander.start_trajectory(trajectory_id, 1.0, relative)
            time.sleep(duration + 1.0) # Add a buffer

            print("[CF] Landing...")
            self.commander.land(0.0, 2.0)
            time.sleep(2.0)
        finally:
            print("[CF] Sequence finished. Stopping.")
            self.commander.stop()

    @staticmethod
    def send_external_pose(cf, pose_data: list):
        """
        Sends external pose data (position and orientation) to the Crazyflie.

        This function is designed to be used as a callback.

        Args:
            cf: The Crazyflie instance.
            pose_data: A list containing [x, y, z, rot_matrix].
        """
        x, y, z, rot_matrix = pose_data
        try:
            # Convert rotation matrix to quaternion for the drone
            quat = Rotation.from_matrix(rot_matrix).as_quat()
        except Exception:
            # Fallback to a neutral quaternion if conversion fails
            quat = [0.0, 0.0, 0.0, 1.0]

        if Config.SEND_FULL_POSE:
            cf.extpos.send_extpose(x, y, z, quat[0], quat[1], quat[2], quat[3])
        else:
            cf.extpos.send_extpos(x, y, z)


def main():
    """Main execution function."""
    cflib.crtp.init_drivers()
    qtm_wrapper = None

    try:
        # Start QTM connection in the background
        qtm_wrapper = QtmWrapper(Config.RIGID_BODY_NAME, qtm_ip=Config.QTM_IP)

        # Connect to the Crazyflie
        with SyncCrazyflie(Config.CRAZYFLIE_URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            cf = scf.cf
            controller = CrazyflieController(cf)

            # Set the QTM callback to send pose data to the Crazyflie
            qtm_wrapper.on_pose_callback = lambda pose: controller.send_external_pose(cf, pose)

            # Prepare the drone for flight
            controller.setup()
            trajectory_id = 1
            duration = controller.upload_trajectory(trajectory_id, FIGURE_EIGHT_TRAJ)

            # Reset the estimator to use the new external position data
            print("[CF] Resetting estimator...")
            reset_estimator(cf)
            time.sleep(0.5)

            # Run the flight
            controller.run_flight_sequence(trajectory_id, duration)

    except KeyboardInterrupt:
        print("\nCaught KeyboardInterrupt. Landing and shutting down.")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        if qtm_wrapper:
            qtm_wrapper.close()
        print("Shutdown complete.")


if __name__ == '__main__':
    main()
