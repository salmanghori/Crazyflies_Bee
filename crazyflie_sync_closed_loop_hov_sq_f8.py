"""
Swarm Control with QTM for Synchronized Flight (Closed-Loop + Open-Loop Sequences)

This script controls a swarm of Crazyflie drones using a Qualisys Track Manager (QTM)
motion capture system for real-time position feedback.

Features:
- Connects to multiple Crazyflies and maps each to a specific QTM rigid body.
- Streams 6-DoF data from QTM to provide external position for each drone.
- Configures each drone's estimator to use the external pose data.
- Provides:
    1. hover_sequence()         – simple takeoff, hover, and land (open-loop)
    2. square_sequence()        – fly a square pattern (open-loop)
    3. closed_loop_figure8_sequence() – continuous 50 Hz closed-loop figure-8 tracking
"""

import asyncio
import math
import time
import xml.etree.cElementTree as ET
from collections import defaultdict
from threading import Thread
from typing import Dict, List, Any

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

RIGID_BY_URI: Dict[str, str] = {
    'radio://0/80/2M/E7E7E7E7E1': 'cf7',
    'radio://0/80/2M/E7E7E7E7E2': 'cf8',
}

# --- Flight & Pose Settings ---
SEND_FULL_POSE = False
ORIENT_STD_DEV = 8.0e-3
EULER_ORDER = "ZYX"
EULER_DEGREES = True

# --- General Timings ---
TAKEOFF_Z = 0.5
TAKEOFF_TIME = 3.0
HOVER_TIME = 5.0
LAND_TIME = 3.0

# --- Square Flight Settings ---
SQUARE_SIDE = 0.5
SQUARE_MOVE_TIME = 2.0

# --- Closed-Loop Figure-8 Settings ---
FIGURE8_DURATION = 20.0
FIGURE8_X_AMP = 0.5
FIGURE8_Y_AMP = 0.35
FIGURE8_PERIOD = 8.0
CONTROL_LOOP_RATE = 50.0
KP_POS = 0.6  # Proportional gain on position error
KD_POS = 0.15  # Derivative gain (used for velocity feed-forward)
# =========================================================

# Global mocap state per drone, updated by QTM callbacks
latest_pose: Dict[str, Dict[str, Any]] = defaultdict(
    lambda: {"t": 0.0, "x": None, "y": None, "z": None, "yaw": 0.0}
)


# =========================================================
# QTM Wrapper (Unchanged)
# =========================================================
class QtmWrapper(Thread):
    def __init__(self, body_name: str, qtm_ip: str = QTM_IP):
        super().__init__()
        self.daemon = True
        self.body_name = body_name
        self.qtm_ip = qtm_ip
        self.on_pose_callback = None
        self.connection = None
        self.qtm_6DoF_labels: List[str] = []
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


# =========================================================
# Drone & Swarm Helpers
# =========================================================
qtm_by_uri: Dict[str, QtmWrapper] = {}


def send_extpose(cf, x, y, z, rot):
    try:
        quat = Rotation.from_matrix(rot).as_quat()
    except Exception:
        quat = [0.0, 0.0, 0.0, 1.0]
    if SEND_FULL_POSE:
        cf.extpos.send_extpose(x, y, z, *quat)
    else:
        cf.extpos.send_extpos(x, y, z)


def make_pose_callback(uri, cf):
    def on_pose_received(pose_data):
        x, y, z, rot_matrix = pose_data
        send_extpose(cf, x, y, z, rot_matrix)
        try:
            yaw = math.atan2(rot_matrix[1][0], rot_matrix[0][0])
        except Exception:
            yaw = 0.0
        latest_pose[uri].update({"t": time.time(), "x": x, "y": y, "z": z, "yaw": yaw})

    return on_pose_received


def init_cf(scf: SyncCrazyflie):
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


# =========================================================
# SEQUENCES
# =========================================================

# [REFACTOR] Centralized, safe landing function to ensure consistent behavior.
def safe_land(scf: SyncCrazyflie):
    """Holds current position for a moment, then lands and stops the motors."""
    cf = scf.cf
    uri = cf.link_uri
    print(f"[{uri}] Landing sequence initiated...")

    # Hold position smoothly before landing
    pose = latest_pose[uri]
    if pose["x"] is not None:
        # Hold last known position for 1 second
        for _ in range(20):
            cf.commander.send_position_setpoint(pose["x"], pose["y"], TAKEOFF_Z, 0)
            time.sleep(0.05)

    cf.high_level_commander.land(0.0, LAND_TIME)
    time.sleep(LAND_TIME)
    cf.high_level_commander.stop()
    print(f"[{uri}] Sequence finished.")


def hover_sequence(scf: SyncCrazyflie):
    """Simple open-loop hover."""
    cf = scf.cf
    cmd = cf.high_level_commander
    uri = cf.link_uri
    print(f"[{uri}] Hover sequence start...")
    try:
        cmd.takeoff(TAKEOFF_Z, TAKEOFF_TIME)
        time.sleep(TAKEOFF_TIME + HOVER_TIME)
    finally:
        safe_land(scf)


def square_sequence(scf: SyncCrazyflie):
    """Open-loop square path."""
    cf = scf.cf
    cmd = cf.high_level_commander
    uri = cf.link_uri
    print(f"[{uri}] Square sequence start...")
    try:
        cmd.takeoff(TAKEOFF_Z, TAKEOFF_TIME)
        time.sleep(TAKEOFF_TIME + 0.8)

        moves = [
            (SQUARE_SIDE, 0, 0, 0), (0, SQUARE_SIDE, 0, 0),
            (-SQUARE_SIDE, 0, 0, 0), (0, -SQUARE_SIDE, 0, 0)
        ]
        for dx, dy, dz, yaw in moves:
            cmd.go_to(dx, dy, dz, yaw, SQUARE_MOVE_TIME, relative=True)
            time.sleep(SQUARE_MOVE_TIME)
    finally:
        safe_land(scf)


def closed_loop_figure8_sequence(scf: SyncCrazyflie):
    """Closed-loop 50Hz trajectory tracking (figure-8)."""
    cf = scf.cf
    uri = cf.link_uri
    cmd = cf.commander

    print(f'[{uri}] Takeoff for closed-loop sequence...')
    cf.high_level_commander.takeoff(TAKEOFF_Z, TAKEOFF_TIME)
    time.sleep(TAKEOFF_TIME + 1.0)

    try:
        # Wait for a valid pose to define the trajectory origin
        for _ in range(100):
            if latest_pose[uri]["x"] is not None: break
            time.sleep(0.02)
        if latest_pose[uri]["x"] is None:
            raise RuntimeError(f"[{uri}] No mocap data received. Aborting.")

        x0, y0 = latest_pose[uri]["x"], latest_pose[uri]["y"]
        print(f"[{uri}] Origin set to ({x0:.2f},{y0:.2f})")

        omega = 2.0 * math.pi / FIGURE8_PERIOD

        def get_reference(t: float):
            xr = x0 + FIGURE8_X_AMP * math.sin(omega * t)
            yr = y0 + FIGURE8_Y_AMP * math.sin(2 * omega * t) / 2
            zr = TAKEOFF_Z
            vxf = FIGURE8_X_AMP * omega * math.cos(omega * t)
            vyf = FIGURE8_Y_AMP * omega * math.cos(2 * omega * t)
            return xr, yr, zr, vxf, vyf

        print(f"[{uri}] Closed-loop tracking start.")
        t0 = time.time()
        dt = 1.0 / CONTROL_LOOP_RATE
        while True:
            now = time.time()
            t = now - t0
            if t > FIGURE8_DURATION: break

            pose = latest_pose[uri]
            # CRITICAL SAFETY CHECK: Abort if mocap data is stale
            if pose["x"] is None or (now - pose["t"] > 0.2):
                print(f"[{uri}] Mocap stale! Aborting trajectory.")
                break

            xr, yr, zr, vxf, vyf = get_reference(t)
            ex, ey, ez = xr - pose["x"], yr - pose["y"], zr - pose["z"]

            # PD controller with velocity feed-forward
            x_cmd = xr + KP_POS * ex + KD_POS * vxf
            y_cmd = yr + KP_POS * ey + KD_POS * vyf
            z_cmd = zr + KP_POS * ez

            cmd.send_position_setpoint(x_cmd, y_cmd, z_cmd, 0.0)

            sleep_t = dt - (time.time() - now)
            if sleep_t > 0: time.sleep(sleep_t)

    except Exception as e:
        print(f"[{uri}] An error occurred in the sequence: {e}")
    finally:
        # [REFACTOR] All exit paths now lead to the same safe landing function.
        safe_land(scf)


# =========================================================
# MAIN
# =========================================================
if __name__ == '__main__':
    for uri in URIS:
        if uri not in RIGID_BY_URI:
            raise ValueError(f"Missing rigid-body mapping for {uri}")

    cflib.crtp.init_drivers()
    factory = CachedCfFactory(rw_cache='./cache')

    try:
        with Swarm(URIS, factory=factory) as swarm:
            print('[Swarm] Connected. Initializing...')
            swarm.parallel_safe(init_cf)
            print('[Swarm] All drones initialized.')

            print('[Swarm] Running chosen sequence...')
            # --- Choose one sequence to run ---
            swarm.parallel_safe(hover_sequence)
            # swarm.parallel_safe(square_sequence)
            # swarm.parallel_safe(closed_loop_figure8_sequence)

    except Exception as e:
        print(f"\n[Swarm] A critical error occurred in the main block: {e}")
    finally:
        print('[Swarm] Shutting down QTM connections...')
        for uri, wrapper in qtm_by_uri.items():
            if wrapper: wrapper.close()
        print('[Swarm] Script finished.')