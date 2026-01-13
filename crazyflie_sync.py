#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# """
# Swarm hover with QTM:
# - One QTM stream per rigid body -> feeds extpos to the matching Crazyflie
# - Estimator init + (optional) controller switch
# - Synchronized takeoff -> hover -> land using swarm.parallel_safe()
#
# NOTE: All figure-8 trajectory code is commented out, per request.
# """
#
# import asyncio
# import math
# import time
# import xml.etree.cElementTree as ET
# from threading import Thread
#
# import qtm_rt
# from qtm_rt import QRTCommandException
# from scipy.spatial.transform import Rotation
#
# import cflib.crtp
# from cflib.crazyflie import Crazyflie
# from cflib.crazyflie.swarm import Swarm, CachedCfFactory
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.utils.reset_estimator import reset_estimator
#
# # ====================== USER CONFIG ======================
# QTM_IP = "192.168.0.105"
#
# URIS = [
#     'radio://0/80/2M/E7E7E7E7E8',
#     'radio://0/80/2M/E7E7E7E7E7',
# ]
#
# RIGID_BY_URI = {
#     'radio://0/80/2M/E7E7E7E7E8': 'cf8',
#     'radio://0/80/2M/E7E7E7E7E7': 'cf7',
# }
#
# SEND_FULL_POSE = False          # Position-only is usually more robust
# ORIENT_STD_DEV = 8.0e-3         # If SEND_FULL_POSE=True, how much to trust external orientation
#
# EULER_ORDER = "ZYX"
# EULER_DEGREES = False           # Set True if your QTM '6deuler' is in degrees
#
# TAKEOFF_Z = 0.8                 # meters
# TAKEOFF_TIME = 2.0              # seconds
# HOVER_TIME = 5.0                # seconds at altitude
# LAND_TIME = 2.0                 # seconds
# # ========================================================
#
#
# # ---------------- QTM WRAPPER (same as your working code) ----------------
# class QtmWrapper(Thread):
#     """
#     Connects to QTM at QTM_IP, streams 6DoF data, and calls self.on_pose([x,y,z,3x3rot]).
#     Prefers '6deuler'; falls back to '6d'.
#     """
#     def __init__(self, body_name, qtm_ip: str = QTM_IP):
#         super().__init__()
#         self.daemon = True
#         self.body_name = body_name
#         self.qtm_ip = qtm_ip
#
#         self.on_pose = None
#         self.connection = None
#         self.qtm_6DoF_labels = []
#         self._stay_open = True
#
#         self.start()
#
#     def close(self):
#         self._stay_open = False
#         self.join()
#
#     def run(self):
#         asyncio.run(self._life_cycle())
#
#     async def _life_cycle(self):
#         ok = await self._connect()
#         if not ok:
#             return
#         try:
#             while self._stay_open:
#                 await asyncio.sleep(1)
#         finally:
#             await self._close()
#
#     async def _connect(self) -> bool:
#         print(f'[QtmWrapper] Connecting to QTM on {self.qtm_ip}')
#         self.connection = await qtm_rt.connect(self.qtm_ip)
#         if self.connection is None:
#             print('[QtmWrapper] ERROR: Failed to connect to QTM.')
#             return False
#
#         try:
#             params = await self.connection.get_parameters(parameters=['6d'])
#             xml = ET.fromstring(params)
#             self.qtm_6DoF_labels = [label.text.strip() for label in xml.findall('*/Body/Name')]
#             if self.body_name not in self.qtm_6DoF_labels:
#                 print(f"[QtmWrapper] WARNING: Body '{self.body_name}' not found. Available: {self.qtm_6DoF_labels}")
#         except Exception as e:
#             print(f"[QtmWrapper] WARNING: Could not parse 6d labels ({e}); defaulting to index 0.")
#             self.qtm_6DoF_labels = []
#
#         try:
#             await self.connection.stream_frames(
#                 frames='allframes',
#                 components=['6deuler'],
#                 on_packet=self._on_packet_6deuler
#             )
#             print("[QtmWrapper] Streaming '6deuler'.")
#         except QRTCommandException:
#             print("[QtmWrapper] '6deuler' not supported. Falling back to '6d'.")
#             await self.connection.stream_frames(
#                 frames='allframes',
#                 components=['6d'],
#                 on_packet=self._on_packet_6d
#             )
#             print("[QtmWrapper] Streaming '6d'.")
#         return True
#
#     def _body_index(self) -> int:
#         if self.qtm_6DoF_labels and self.body_name in self.qtm_6DoF_labels:
#             return self.qtm_6DoF_labels.index(self.body_name)
#         return 0
#
#     def _on_packet_6deuler(self, packet):
#         if not hasattr(packet, 'get_6d_euler'):
#             return
#         header, bodies = packet.get_6d_euler()
#         if not bodies:
#             return
#
#         idx = self._body_index()
#         if idx >= len(bodies):
#             return
#
#         pos_mm, euler = bodies[idx]  # [roll, pitch, yaw]
#         if pos_mm is None or any(math.isnan(v) for v in pos_mm):
#             return
#
#         x = pos_mm[0] / 1000.0
#         y = pos_mm[1] / 1000.0
#         z = pos_mm[2] / 1000.0
#
#         rot3 = None
#         if euler is not None and not any(math.isnan(v) for v in euler):
#             roll, pitch, yaw = euler
#             try:
#                 if EULER_ORDER.upper() == 'ZYX':
#                     rot3 = Rotation.from_euler('ZYX', [yaw, pitch, roll], degrees=EULER_DEGREES).as_matrix()
#                 else:
#                     rot3 = Rotation.from_euler(EULER_ORDER, [roll, pitch, yaw], degrees=EULER_DEGREES).as_matrix()
#             except Exception:
#                 rot3 = None
#
#         if rot3 is None:
#             rot3 = [[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]]
#
#         if self.on_pose:
#             self.on_pose([x, y, z, rot3])
#
#     def _on_packet_6d(self, packet):
#         header, bodies = packet.get_6d()
#         if not bodies:
#             return
#
#         idx = self._body_index()
#         if idx >= len(bodies):
#             return
#
#         pos_mm, rot = bodies[idx]
#         if pos_mm is None or any(math.isnan(v) for v in pos_mm):
#             return
#
#         x = pos_mm[0] / 1000.0
#         y = pos_mm[1] / 1000.0
#         z = pos_mm[2] / 1000.0
#
#         r = rot.matrix  # flat list length 9
#         rot3 = [[r[0], r[3], r[6]], [r[1], r[4], r[7]], [r[2], r[5], r[8]]]
#
#         if self.on_pose:
#             self.on_pose([x, y, z, rot3])
#
#     async def _close(self):
#         try:
#             await self.connection.stream_frames_stop()
#         except Exception:
#             pass
#         if self.connection:
#             self.connection.disconnect()
#         print("[QtmWrapper] Disconnected from QTM.")
# # -------------------------------------------------------------------------
#
#
# # -------------------- Pose streaming + helpers ---------------------------
# def send_extpose_rot_matrix(cf, x, y, z, rot):
#     try:
#         quat = Rotation.from_matrix(rot).as_quat()
#     except Exception:
#         quat = [0.0, 0.0, 0.0, 1.0]
#
#     if SEND_FULL_POSE:
#         cf.extpos.send_extpose(x, y, z, quat[0], quat[1], quat[2], quat[3])
#     else:
#         cf.extpos.send_extpos(x, y, z)
#
# def adjust_orientation_sensitivity(cf):
#     cf.param.set_value('locSrv.extQuatStdDev', ORIENT_STD_DEV)
#
# def activate_kalman_estimator(cf):
#     cf.param.set_value('stabilizer.estimator', '2')
#     cf.param.set_value('locSrv.extQuatStdDev', 0.06)
#
# def activate_mellinger_controller(cf):
#     cf.param.set_value('stabilizer.controller', '2')   # optional
# # -------------------------------------------------------------------------
#
#
# # -------------------- Swarm tasks (NO figure-8) --------------------------
# qtm_by_uri = {}
#
# def init_cf(scf: SyncCrazyflie):
#     """Attach QTM stream -> CF, set estimator, (optional) controller, reset estimator."""
#     cf = scf.cf
#     uri = cf.link_uri
#     rigid = RIGID_BY_URI[uri]
#
#     # QTM stream for this CF
#     wrapper = QtmWrapper(rigid, qtm_ip=QTM_IP)
#     qtm_by_uri[uri] = wrapper
#
#     def on_pose(pose, _cf=cf):
#         send_extpose_rot_matrix(_cf, pose[0], pose[1], pose[2], pose[3])
#     wrapper.on_pose = on_pose
#
#     adjust_orientation_sensitivity(cf)
#     activate_kalman_estimator(cf)
#     # activate_mellinger_controller(cf)  # uncomment if you want Mellinger
#     #
#     # Ensure estimator is consistent after pose starts flowing
#     reset_estimator(cf)
#
# def hover_sequence(scf: SyncCrazyflie):
#     """Step 4: Taking off and landing in sync."""
#     cf = scf.cf
#     cmd = cf.high_level_commander
#
#     # Takeoff
#     cmd.takeoff(TAKEOFF_Z, TAKEOFF_TIME)
#     time.sleep(TAKEOFF_TIME + 0.5)
#
#     # Hover at altitude
#     time.sleep(HOVER_TIME)
#
#     # Land
#     cmd.land(0.0, LAND_TIME)
#     time.sleep(LAND_TIME + 0.5)
#
#     cmd.stop()
# # -------------------------------------------------------------------------
#
#
# # ------------------------------ MAIN ------------------------------------
# if __name__ == '__main__':
#     # sanity
#     for uri in URIS:
#         assert uri in RIGID_BY_URI, f"Missing rigid-body mapping for {uri}"
#
#     cflib.crtp.init_drivers()
#     factory = CachedCfFactory(rw_cache='./cache')
#
#     try:
#         with Swarm(URIS, factory=factory) as swarm:
#             print('[Swarm] Connected to Crazyflies')
#
#             # Init each CF (QTM stream + estimator)
#             swarm.parallel_safe(init_cf)
#
#             # >>> FIGURE-8 CODE REMOVED/COMMENTED <<<:
#             # # We would upload Poly4D trajectory and start it here.
#             # # duration = upload_trajectory(...)
#             # # swarm.parallel_safe(start_trajectory)
#             # -------------------------------------------------------
#
#             # Step 4: Taking off and Landing in Sync
#             swarm.parallel_safe(hover_sequence)
#
#     finally:
#         # Close all QTM wrappers
#         for uri, wrapper in qtm_by_uri.items():
#             try:
#                 wrapper.close()
#             except Exception:
#                 pass
#         print('[Swarm] Done.')


"""
Swarm Control with QTM for Synchronized Flight.

This script controls a swarm of Crazyflie drones using a Qualisys Track Manager
(QTM) motion capture system for real-time position feedback.

Features:
- Connects to multiple Crazyflies and maps each to a specific QTM rigid body.
- Streams 6-DoF data from QTM to provide external position for each drone.
- Configures each drone's estimator to use the external pose data.
- Provides two synchronized flight sequences using `swarm.parallel_safe()`:
    1. `hover_sequence`: A simple takeoff, hover, and land.
    2. `square_sequence`: A takeoff, flight in a square pattern, and land.

To switch between sequences, modify the call in the `if __name__ == '__main__':` block.
"""

import asyncio
import math
import time
import xml.etree.cElementTree as ET
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

# List of URIs for the Crazyflies in the swarm.
URIS = [
    'radio://0/80/2M/E7E7E7E7E1',
    'radio://0/80/2M/E7E7E7E7E2',
]

# Mapping of each Crazyflie URI to its corresponding rigid body1me in QTM.
RIGID_BY_URI = {
    'radio://0/80/2M/E7E7E7E7E1': 'cf7',
    'radio://0/80/2M/E7E7E7E7E2': 'cf8',
}

# --- Flight & Pose Settings ---
SEND_FULL_POSE = False          # Position-only is often more robust.
ORIENT_STD_DEV = 8.0e-3         # If SEND_FULL_POSE=True, how much to trust QTM orientation.

EULER_ORDER = "ZYX"             # Euler order from QTM (Roll, Pitch, Yaw). 'ZYX' is common.
EULER_DEGREES = True            # Set True if your QTM '6deuler' component outputs degrees.

# --- Sequence Timings & Dimensions ---
TAKEOFF_Z = 0.5                 # Takeoff altitude in meters.
TAKEOFF_TIME = 3.0              # Duration of takeoff maneuver in seconds.
HOVER_TIME = 5.0                # Duration to hover at altitude in seconds.
LAND_TIME = 3.0                 # Duration of landing maneuver in seconds.

SQUARE_SIDE_LENGTH = 0.5        # Side length of the square in meters.
SQUARE_MOVE_TIME = 2.5          # Time to fly one side of the square in seconds.
# ========================================================


class QtmWrapper(Thread):
    """
    Manages the connection to QTM and streams 6-DoF data in a separate thread.
    This class connects to a QTM server, identifies a specified rigid body,
    and streams its position and orientation. It prefers the '6deuler' component
    but can fall back to '6d'. The received pose is passed to a callback.
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
        """Fetches the names of all 6-DoF rigid bodies from QTM."""
        try:
            params = await self.connection.get_parameters(parameters=['6d'])
            xml = ET.fromstring(params)
            self.qtm_6DoF_labels = [label.text.strip() for label in xml.findall('*/Body/Name')]
            if self.body_name not in self.qtm_6DoF_labels:
                print(f"[QTM] WARNING: Body '{self.body_name}' not in QTM. Available: {self.qtm_6DoF_labels}")
        except Exception as e:
            print(f"[QTM] WARNING: Could not parse 6d labels ({e}). Will default to the first body.")

    def _body_index(self) -> int:
        """Returns the index of the desired rigid body."""
        if self.qtm_6DoF_labels and self.body_name in self.qtm_6DoF_labels:
            return self.qtm_6DoF_labels.index(self.body_name)
        return 0

    def _on_packet_6deuler(self, packet):
        """Callback for processing '6deuler' data packets."""
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
        """Callback for processing '6d' (rotation matrix) data packets."""
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
        """Converts Euler angles to a 3x3 rotation matrix using scipy."""
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
        """Stops the data stream and disconnects from the QTM server."""
        try:
            await self.connection.stream_frames_stop()
        except Exception: pass
        if self.connection:
            self.connection.disconnect()
        print(f"[QTM] [{self.body_name}] Disconnected.")


# -------------------- Drone & Swarm Functions ---------------------------
qtm_by_uri = {}  # Global dict to hold QTM wrappers for cleanup

def send_extpose(cf, x, y, z, rot):
    """Sends external pose data to the Crazyflie's estimator."""
    try:
        quat = Rotation.from_matrix(rot).as_quat()
    except Exception:
        quat = [0.0, 0.0, 0.0, 1.0]  # Fallback quaternion

    if SEND_FULL_POSE:
        cf.extpos.send_extpose(x, y, z, quat[0], quat[1], quat[2], quat[3])
    else:
        cf.extpos.send_extpos(x, y, z)

def init_cf(scf: SyncCrazyflie):
    """
    Initializes a single Crazyflie.
    - Creates and attaches a QTM stream for external position.
    - Sets the estimator to Kalman (to accept extpos).
    - Resets the estimator to apply the new settings.
    """
    cf = scf.cf
    uri = cf.link_uri
    rigid_body_name = RIGID_BY_URI[uri]
    print(f'[{uri}] Initializing...')

    # Start a QTM stream for this specific Crazyflie
    wrapper = QtmWrapper(rigid_body_name, qtm_ip=QTM_IP)
    qtm_by_uri[uri] = wrapper

    # The callback that pipes QTM pose data to the Crazyflie
    def on_pose_received(pose_data):
        send_extpose(cf, pose_data[0], pose_data[1], pose_data[2], pose_data[3])
    wrapper.on_pose_callback = on_pose_received

    # Configure the drone's estimator
    cf.param.set_value('stabilizer.estimator', '2')  # 2 = Kalman estimator
    cf.param.set_value('locSrv.extQuatStdDev', ORIENT_STD_DEV)

    # Reset the estimator to start using the external pose data
    time.sleep(1) # Allow some packets to arrive before resetting
    reset_estimator(cf)
    print(f'[{uri}] Initialization complete.')

def hover_sequence(scf: SyncCrazyflie):
    """A simple synchronized sequence: takeoff, hover, land."""
    cf = scf.cf
    cmd = cf.high_level_commander
    uri = cf.link_uri

    print(f'[{uri}] Starting hover sequence...')
    cmd.takeoff(TAKEOFF_Z, TAKEOFF_TIME)
    time.sleep(TAKEOFF_TIME + HOVER_TIME)
    cmd.land(0.0, LAND_TIME)
    time.sleep(LAND_TIME + 0.5)
    cmd.stop()
    print(f'[{uri}] Hover sequence finished.')

def square_sequence(scf: SyncCrazyflie):
    """A synchronized sequence to fly in a square pattern."""
    cf = scf.cf
    cmd = cf.high_level_commander
    uri = cf.link_uri

    print(f'[{uri}] Starting square sequence...')

    # 1. Takeoff
    cmd.takeoff(TAKEOFF_Z, TAKEOFF_TIME)
    time.sleep(TAKEOFF_TIME + 1.0) # Wait for takeoff to complete + buffer

    # 2. Fly the square using relative go_to commands
    print(f"[{uri}] Side 1: Forward (+X)")
    cmd.go_to(SQUARE_SIDE_LENGTH, 0, 0, 0, SQUARE_MOVE_TIME, relative=True)
    time.sleep(SQUARE_MOVE_TIME)

    print(f"[{uri}] Side 2: Left (+Y)")
    cmd.go_to(0, SQUARE_SIDE_LENGTH, 0, 0, SQUARE_MOVE_TIME, relative=True)
    time.sleep(SQUARE_MOVE_TIME)

    print(f"[{uri}] Side 3: Backward (-X)")
    cmd.go_to(-SQUARE_SIDE_LENGTH, 0, 0, 0, SQUARE_MOVE_TIME, relative=True)
    time.sleep(SQUARE_MOVE_TIME)

    print(f"[{uri}] Side 4: Right (-Y)")
    cmd.go_to(0, -SQUARE_SIDE_LENGTH, 0, 0, SQUARE_MOVE_TIME, relative=True)
    time.sleep(SQUARE_MOVE_TIME)

    # 3. Land
    print(f"[{uri}] Landing...")
    cmd.land(0.0, LAND_TIME)
    time.sleep(LAND_TIME + 0.5)

    cmd.stop()
    print(f'[{uri}] Square sequence finished.')

# ------------------------------ MAIN ------------------------------------
if __name__ == '__main__':
    # Sanity check the configuration
    for uri in URIS:
        if uri not in RIGID_BY_URI:
            raise ValueError(f"Configuration error: Missing rigid-body mapping for {uri}")

    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    try:
        with Swarm(URIS, factory=factory) as swarm:
            print('[Swarm] All Crazyflies connected.')

            print('[Swarm] Step 1: Initializing all drones in parallel...')
            swarm.parallel_safe(init_cf)
            print('[Swarm] All drones initialized.')

            print('[Swarm] Step 2: Running flight sequence...')
            # --- CHOOSE YOUR SEQUENCE ---
            # Uncomment the sequence you want to run.
            # swarm.parallel_safe(hover_sequence)
            swarm.parallel_safe(square_sequence)
            # --------------------------

    except Exception as e:
        print(f"\n[Swarm] An error occurred: {e}")
    finally:
        # Ensure all QTM connections are closed cleanly.
        print('[Swarm] Shutting down all QTM connections...')
        for uri, wrapper in qtm_by_uri.items():
            if wrapper:
                wrapper.close()
        print('[Swarm] Script finished.')

