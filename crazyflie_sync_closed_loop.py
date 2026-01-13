"""
Swarm Control with QTM for Synchronized Flight (with Closed-Loop Trajectory Tracking).

This script controls a swarm of Crazyflie drones using a Qualisys Track Manager
(QTM) motion capture system for real-time position feedback.

Features:
- Connects to multiple Crazyflies and maps each to a specific QTM rigid body.
- Streams 6-DoF data from QTM to provide external position for each drone.
- Configures each drone's estimator to use the external pose data.
- Includes open-loop sequences (`hover`, `square`) and a new closed-loop sequence.

The `closed_loop_figure8_sequence` implements an outer-loop controller that runs
at ~50Hz, constantly correcting the drone's path to precisely follow a
mathematically defined figure-8 trajectory.
"""

import asyncio
import math
import time
import xml.etree.cElementTree as ET
from collections import defaultdict
from threading import Thread

import cflib.crtp
import qtm_rt
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.swarm import Swarm, CachedCfFactory
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.reset_estimator import reset_estimator
from qtm_rt import QRTCommandException
from scipy.spatial.transform import Rotation

# ====================== USER CONFIG ======================
QTM_IP = "192.168.0.105"

URIS = [
    'radio://0/80/2M/E7E7E7E7E1',
    'radio://0/80/2M/E7E7E7E7E2',
]

RIGID_BY_URI = {
    'radio://0/80/2M/E7E7E7E7E1': 'cf7',
    'radio://0/80/2M/E7E7E7E7E2': 'cf8',
}

# --- Flight & Pose Settings ---
SEND_FULL_POSE = False
ORIENT_STD_DEV = 8.0e-3
EULER_ORDER = "ZYX"
EULER_DEGREES = True

# --- General Sequence Timings ---
TAKEOFF_Z = 0.5
TAKEOFF_TIME = 3.0
LAND_TIME = 3.0

# --- Closed-Loop Figure-8 Trajectory Settings ---
FIGURE8_DURATION = 20.0  # Total time to run the trajectory (seconds)
FIGURE8_X_AMP = 0.5  # Amplitude (size) of the figure-8 on the X-axis (meters)
FIGURE8_Y_AMP = 0.35  # Amplitude (size) of the figure-8 on the Y-axis (meters)
FIGURE8_PERIOD = 8.0  # Time to complete one full loop of the figure-8 (seconds)
CONTROL_LOOP_RATE = 50.0  # Frequency of the outer-loop controller (Hz)
# PD Gains for the outer-loop controller
KP_POS = 0.6  # Proportional gain on position error
KD_POS = 0.15  # Derivative gain (applied as feed-forward on velocity)
# ========================================================


# Global dictionary to store the latest pose from QTM for each drone.
# This is populated by the QTM callback and read by the control loop.
latest_pose = defaultdict(lambda: {"t": 0.0, "x": None, "y": None, "z": None, "yaw": 0.0})


class QtmWrapper(Thread):
    """
    Manages the connection to QTM and streams 6-DoF data.
    (This class is functionally unchanged from your working version).
    """

    def __init__(self, body_name, qtm_ip: str = QTM_IP):
        super().__init__()
        self.daemon = True
        self.body_name = body_name
        self.qtm_ip = qtm_ip
        self.on_pose_callback = None
        self.connection = None
        self.qtm_6DoF_labels = []
        self._stay_open = True
        self.start()

    def close(self):
        self._stay_open = False
        self.join()

    def run(self):
        asyncio.run(self._life_cycle())

    async def _life_cycle(self):
        if await self._connect():
            try:
                while self._stay_open:
                    await asyncio.sleep(1)
            finally:
                await self._close()

    async def _connect(self) -> bool:
        print(f'[QTM] [{self.body_name}] Connecting to {self.qtm_ip}...')
        self.connection = await qtm_rt.connect(self.qtm_ip)
        if self.connection is None:
            print(f'[QTM] [{self.body_name}] ERROR: Failed to connect.')
            return False
        await self._discover_body_labels()
        try:
            await self.connection.stream_frames(
                frames='allframes', components=['6deuler'], on_packet=self._on_packet_6deuler)
            print(f"[QTM] [{self.body_name}] Streaming '6deuler' data.")
        except QRTCommandException:
            print(f"[QTM] [{self.body_name}] '6deuler' not supported. Falling back to '6d'.")
            await self.connection.stream_frames(
                frames='allframes', components=['6d'], on_packet=self._on_packet_6d)
            print(f"[QTM] [{self.body_name}] Streaming '6d' data.")
        return True

    async def _discover_body_labels(self):
        try:
            params = await self.connection.get_parameters(parameters=['6d'])
            xml = ET.fromstring(params)
            self.qtm_6DoF_labels = [label.text.strip() for label in xml.findall('*/Body/Name')]
            if self.body_name not in self.qtm_6DoF_labels:
                print(f"[QTM] WARNING: Body '{self.body_name}' not in QTM. Available: {self.qtm_6DoF_labels}")
        except Exception as e:
            print(f"[QTM] WARNING: Could not parse 6d labels ({e}).")

    def _body_index(self) -> int:
        if self.qtm_6DoF_labels and self.body_name in self.qtm_6DoF_labels:
            return self.qtm_6DoF_labels.index(self.body_name)
        return 0

    def _on_packet_6deuler(self, packet):
        _, bodies = packet.get_6d_euler()
        if not bodies: return
        idx = self._body_index()
        if idx >= len(bodies): return
        pos_mm, euler = bodies[idx]
        if pos_mm is None or any(math.isnan(v) for v in pos_mm): return
        x, y, z = [p / 1000.0 for p in pos_mm]
        rot3 = self._euler_to_rotation_matrix(euler)
        if self.on_pose_callback:
            self.on_pose_callback([x, y, z, rot3])

    def _on_packet_6d(self, packet):
        _, bodies = packet.get_6d()
        if not bodies: return
        idx = self._body_index()
        if idx >= len(bodies): return
        pos_mm, rot = bodies[idx]
        if pos_mm is None or any(math.isnan(v) for v in pos_mm): return
        x, y, z = [p / 1000.0 for p in pos_mm]
        r = rot.matrix
        rot3 = [[r[0], r[3], r[6]], [r[1], r[4], r[7]], [r[2], r[5], r[8]]]
        if self.on_pose_callback:
            self.on_pose_callback([x, y, z, rot3])

    @staticmethod
    def _euler_to_rotation_matrix(euler_angles):
        if euler_angles and not any(math.isnan(v) for v in euler_angles):
            try:
                roll, pitch, yaw = euler_angles
                if EULER_ORDER.upper() == 'ZYX':
                    return Rotation.from_euler('ZYX', [yaw, pitch, roll], degrees=EULER_DEGREES).as_matrix()
                return Rotation.from_euler(EULER_ORDER, euler_angles, degrees=EULER_DEGREES).as_matrix()
            except Exception:
                pass
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

    async def _close(self):
        try:
            await self.connection.stream_frames_stop()
        except Exception:
            pass
        if self.connection: self.connection.disconnect()
        print(f"[QTM] [{self.body_name}] Disconnected.")


# -------------------- Drone & Swarm Functions ---------------------------
qtm_by_uri = {}


def send_extpose(cf, x, y, z, rot):
    """Sends external pose data to the Crazyflie's internal estimator."""
    try:
        quat = Rotation.from_matrix(rot).as_quat()
    except Exception:
        quat = [0.0, 0.0, 0.0, 1.0]
    if SEND_FULL_POSE:
        cf.extpos.send_extpose(x, y, z, quat[0], quat[1], quat[2], quat[3])
    else:
        cf.extpos.send_extpos(x, y, z)


def make_pose_callback(uri, cf):
    """
    Creates a callback function that both updates the global `latest_pose`
    and sends the pose to the drone's internal estimator.
    """

    def on_pose_received(pose_data):
        x, y, z, rot_matrix = pose_data

        # 1. Send pose to the drone for its internal Kalman filter
        send_extpose(cf, x, y, z, rot_matrix)

        # 2. Update the global state for the outer-loop controller
        try:
            yaw = math.atan2(rot_matrix[1][0], rot_matrix[0][0])
        except Exception:
            yaw = 0.0

        latest_pose[uri].update({
            "t": time.time(),
            "x": x, "y": y, "z": z, "yaw": yaw
        })

    return on_pose_received


def init_cf(scf: SyncCrazyflie):
    """Initializes a single Crazyflie for swarm flight with QTM."""
    cf = scf.cf
    uri = cf.link_uri
    rigid_body_name = RIGID_BY_URI[uri]
    print(f'[{uri}] Initializing...')

    wrapper = QtmWrapper(rigid_body_name, qtm_ip=QTM_IP)
    qtm_by_uri[uri] = wrapper
    wrapper.on_pose_callback = make_pose_callback(uri, cf)

    cf.param.set_value('stabilizer.estimator', '2')
    cf.param.set_value('locSrv.extQuatStdDev', ORIENT_STD_DEV)
    time.sleep(1)
    reset_estimator(cf)
    print(f'[{uri}] Initialization complete.')


def closed_loop_figure8_sequence(scf: SyncCrazyflie):
    """
    Tracks a figure-8 trajectory using a closed-loop outer-loop controller.
    - Takes off to a specified altitude.
    - Establishes a local origin.
    - Runs a 50Hz loop that calculates tracking error and sends corrected
      position setpoints to the drone.
    - Lands safely upon completion or if mocap data is lost.
    """
    cf = scf.cf
    uri = cf.link_uri
    # Use low-level commander for streaming setpoints
    cmd = cf.commander

    try:
        # --- 1. Takeoff and establish local origin ---
        print(f'[{uri}] Taking off...')
        cf.high_level_commander.takeoff(TAKEOFF_Z, TAKEOFF_TIME)
        time.sleep(TAKEOFF_TIME + 1.0)

        # Wait for a valid pose to define the center of the trajectory
        for _ in range(100):
            if latest_pose[uri]["x"] is not None: break
            time.sleep(0.02)

        if latest_pose[uri]["x"] is None:
            raise RuntimeError(f"[{uri}] No motion capture data received. Aborting.")

        x0, y0 = latest_pose[uri]["x"], latest_pose[uri]["y"]
        print(f"[{uri}] Origin set to ({x0:.2f}, {y0:.2f})")

        # --- 2. Define the trajectory and controller ---
        omega = 2.0 * math.pi / FIGURE8_PERIOD

        def get_reference(t):
            # Lissajous curve for a smooth figure-8
            xr = x0 + FIGURE8_X_AMP * math.sin(omega * t)
            yr = y0 + FIGURE8_Y_AMP * math.sin(2 * omega * t) / 2
            zr = TAKEOFF_Z
            yawr_deg = 0.0
            # Velocity feed-forward (derivative of the position)
            vxf = FIGURE8_X_AMP * omega * math.cos(omega * t)
            vyf = FIGURE8_Y_AMP * omega * math.cos(2 * omega * t)
            return xr, yr, zr, yawr_deg, vxf, vyf

        # --- 3. Run the control loop ---
        print(f"[{uri}] Starting closed-loop tracking...")
        t_start = time.time()
        dt = 1.0 / CONTROL_LOOP_RATE

        while True:
            now = time.time()
            t = now - t_start
            if t > FIGURE8_DURATION:
                break

            pose = latest_pose[uri]
            if pose["x"] is None or (now - pose["t"] > 0.2):
                print(f"[{uri}] Mocap data is stale! Breaking loop.")
                break

            # Get desired state from trajectory
            xr, yr, zr, yawr_deg, vxf, vyf = get_reference(t)

            # Calculate position error
            ex = xr - pose["x"]
            ey = yr - pose["y"]
            ez = zr - pose["z"]

            # PD Controller -> Corrected Position Setpoint
            # P term corrects position error.
            # D term adds velocity feed-forward to reduce lag.
            x_cmd = xr + KP_POS * ex + KD_POS * vxf
            y_cmd = yr + KP_POS * ey + KD_POS * vyf
            z_cmd = zr + KP_POS * ez

            cmd.send_position_setpoint(x_cmd, y_cmd, z_cmd, yawr_deg)

            # Sleep to maintain the loop rate
            sleep_time = dt - (time.time() - now)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except Exception as e:
        print(f"[{uri}] Error in sequence: {e}")
    finally:
        # --- 4. Land safely ---
        print(f"[{uri}] Sequence finished. Landing...")
        # Hold position for a moment before landing
        pose = latest_pose[uri]
        if pose['x'] is not None:
            for _ in range(20):
                cmd.send_position_setpoint(pose['x'], pose['y'], TAKEOFF_Z, 0)
                time.sleep(0.05)

        cf.high_level_commander.land(0.0, LAND_TIME)
        time.sleep(LAND_TIME)
        cf.high_level_commander.stop()


# ------------------------------ MAIN ------------------------------------
if __name__ == '__main__':
    for uri in URIS:
        if uri not in RIGID_BY_URI:
            raise ValueError(f"Config error: Missing rigid-body mapping for {uri}")

    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    try:
        with Swarm(URIS, factory=factory) as swarm:
            print('[Swarm] All Crazyflies connected.')

            print('[Swarm] Initializing all drones in parallel...')
            swarm.parallel_safe(init_cf)
            print('[Swarm] All drones initialized.')

            print('[Swarm] Running closed-loop figure-8 flight sequence...')
            swarm.parallel_safe(closed_loop_figure8_sequence)

    except Exception as e:
        print(f"\n[Swarm] An unhandled error occurred: {e}")
    finally:
        print('[Swarm] Shutting down all QTM connections...')
        for uri, wrapper in qtm_by_uri.items():
            if wrapper:
                wrapper.close()
        print('[Swarm] Script finished.')