import asyncio
import math
import time
import xml.etree.cElementTree as ET
from threading import Thread
from typing import Dict, List, Callable, Optional

# Crazyflie / QTM
import cflib.crtp
import qtm_rt
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.reset_estimator import reset_estimator
from qtm_rt import QRTCommandException
from scipy.spatial.transform import Rotation

import numpy as np

"""
Single Crazyflie Showcase with QTM Position Feedback (Hover / Square / Figure‑8)

This script drives **one Crazyflie** using Qualisys Track Manager (QTM) pose
streaming into the Crazyflie EKF. It provides three working routines, plus a
stub for Poly4D (on‑board trajectory memory) that currently falls back to the
smooth segmented figure‑8. If you want a true Poly4D uploader, say the word and
I'll drop in the polynomial–coefficient generator next.

Routines (set MODE below):
  - 'hover' : takeoff → hover → land
  - 'square': 4 legs at constant z using HL go_to
  - 'fig8'  : polynomial‑eased lemniscate sampled into many tiny HL moves
  - 'poly4d': (stub) currently redirects to 'fig8' and prints a note

Requirements:
  pip install cflib qtm-rt numpy scipy
"""

# ====================== USER CONFIG — EDIT THESE ======================
QTM_IP: str = "192.168.0.105"
URI: str = 'radio://0/80/2M/E7E7E7E7E1'
RIGID_BODY: str = 'cf9'  # must match QTM rigid body name

# --- External pose streaming to the EKF ---
SEND_FULL_POSE: bool = False   # True: position+quaternion, False: position only
ORIENT_STD_DEV: float = 8.0e-3 # locSrv.extQuatStdDev
EULER_ORDER: str = "ZYX"
EULER_DEGREES: bool = True

# --- Flight Timings/Heights ---
TAKEOFF_Z: float = 0.5
TAKEOFF_TIME: float = 3.0
HOVER_TIME: float = 5.0
LAND_TIME: float = 3.0

# --- Square Flight Settings ---
SQUARE_SIDE_LENGTH: float = 0.6
SQUARE_MOVE_TIME: float = 2.2  # per leg

# --- Figure‑8 (polynomial‑eased path sampled to HL segments) ---
FIG8_A: float = 0.40     # x amplitude (m)
FIG8_B: float = 0.30     # y amplitude (m)
FIG8_PERIOD: float = 12.0  # seconds per full figure‑8
FIG8_SEGMENTS: int = 48    # number of HL segments (>= 30 recommended)

# Choose what to run: 'hover', 'square', 'fig8', 'poly4d' (stub → fig8)
MODE: str = 'hover'

# =====================================================================
# QTM Wrapper (single body)
# =====================================================================
class QtmWrapper(Thread):
    def __init__(self, body_name: str, qtm_ip: str = QTM_IP):
        super().__init__()
        self.daemon = True
        self.body_name = body_name
        self.qtm_ip = qtm_ip
        self.on_pose_callback: Optional[Callable[[List[float]], None]] = None
        self.connection = None
        self.qtm_6DoF_labels: List[str] = []
        self._stay_open = True
        self.start()

    def close(self):
        self._stay_open = False
        try:
            self.join(timeout=2.0)
        except Exception:
            pass

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
        print(f'[QTM] Connecting to {self.qtm_ip} …')
        self.connection = await qtm_rt.connect(self.qtm_ip)
        if self.connection is None:
            print('[QTM] ERROR: Failed to connect.')
            return False
        await self._discover_body_labels()
        try:
            await self.connection.stream_frames(
                frames='allframes', components=['6deuler'], on_packet=self._on_packet_6deuler)
            print("[QTM] Streaming '6deuler' data.")
        except QRTCommandException:
            print("[QTM] '6deuler' not supported. Falling back to '6d'.")
            await self.connection.stream_frames(
                frames='allframes', components=['6d'], on_packet=self._on_packet_6d)
            print("[QTM] Streaming '6d' data.")
        return True

    async def _discover_body_labels(self):
        try:
            params = await self.connection.get_parameters(parameters=['6d'])
            xml = ET.fromstring(params)
            self.qtm_6DoF_labels = [label.text.strip() for label in xml.findall('*/Body/Name')]
            if self.body_name not in self.qtm_6DoF_labels:
                print(f"[QTM] WARNING: Body '{self.body_name}' not found. Available: {self.qtm_6DoF_labels}")
        except Exception as e:
            print(f"[QTM] WARNING: Could not parse 6d labels ({e}).")

    def _body_index(self) -> int:
        if self.qtm_6DoF_labels and self.body_name in self.qtm_6DoF_labels:
            return self.qtm_6DoF_labels.index(self.body_name)
        return 0

    def _on_packet_6deuler(self, packet):
        _, bodies = packet.get_6d_euler()
        if not bodies:
            return
        idx = self._body_index()
        if idx >= len(bodies):
            return
        pos_mm, euler = bodies[idx]
        if pos_mm is None or any(math.isnan(v) for v in pos_mm):
            return
        x, y, z = [p / 1000.0 for p in pos_mm]
        rot3 = self._euler_to_rotation_matrix(euler)
        if self.on_pose_callback:
            self.on_pose_callback([x, y, z, rot3])

    def _on_packet_6d(self, packet):
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
                return Rotation.from_euler(EULER_ORDER, [roll, pitch, yaw], degrees=EULER_DEGREES).as_matrix()
            except Exception:
                pass
        return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

    async def _close(self):
        try:
            await self.connection.stream_frames_stop()
        except Exception:
            pass
        if self.connection:
            self.connection.disconnect()
        print('[QTM] Disconnected.')

# =====================================================================
# Helpers for Crazyflie HL Commander
# =====================================================================

def _send_extpose(cf, x, y, z, rot):
    # Send full pose if desired, else just position (3‑arg call)
    if SEND_FULL_POSE:
        try:
            quat = Rotation.from_matrix(rot).as_quat()
        except Exception:
            quat = [0.0, 0.0, 0.0, 1.0]
        cf.extpos.send_extpose(x, y, z, *quat)
    else:
        cf.extpos.send_extpos(x, y, z)


def _make_pose_callback(cf):
    def on_pose(pose):
        x, y, z, rot = pose
        _send_extpose(cf, x, y, z, rot)
    return on_pose


def init_cf_and_qtm(scf: SyncCrazyflie):
    cf = scf.cf
    print('[CF] Setting estimator to EKF (2) and configuring ext pose…')
    cf.param.set_value('stabilizer.estimator', '2')
    cf.param.set_value('locSrv.extQuatStdDev', ORIENT_STD_DEV)
    time.sleep(0.5)

    print('[CF] Resetting estimator…')
    reset_estimator(cf)

    print('[QTM] Spawning QTM streaming thread…')
    qw = QtmWrapper(RIGID_BODY, qtm_ip=QTM_IP)
    qw.on_pose_callback = _make_pose_callback(cf)
    return qw  # caller should keep and later close()


# =====================================================================
# SHOWCASES (single CF)
# =====================================================================

def hover_showcase(scf: SyncCrazyflie):
    hl = scf.cf.high_level_commander
    print('[SHOW] Hover start…')
    hl.takeoff(TAKEOFF_Z, TAKEOFF_TIME)
    time.sleep(TAKEOFF_TIME + 0.6)
    time.sleep(HOVER_TIME)
    hl.land(0.0, LAND_TIME)
    time.sleep(LAND_TIME + 0.6)
    hl.stop()
    print('[SHOW] Hover done.')


def square_showcase(scf: SyncCrazyflie):
    hl = scf.cf.high_level_commander
    L = SQUARE_SIDE_LENGTH
    T = SQUARE_MOVE_TIME
    print('[SHOW] Square start…')
    hl.takeoff(TAKEOFF_Z, TAKEOFF_TIME); time.sleep(TAKEOFF_TIME + 0.6)

    # four smooth relative legs
    for dx, dy in [( L, 0.0), (0.0,  L), (-L, 0.0), (0.0, -L)]:
        hl.go_to(dx, dy, 0.0, 0.0, T, relative=True)
        time.sleep(T + 0.08)

    hl.land(0.0, LAND_TIME); time.sleep(LAND_TIME + 0.6); hl.stop()
    print('[SHOW] Square done.')


def _lemniscate_samples(a: float, b: float, period: float, segments: int):
    """Return arrays (dx[i], dy[i], dt[i]) forming a smooth figure‑8 centered at (0,0)."""
    t = np.linspace(0.0, 2.0 * np.pi, segments + 1)
    x = a * np.sin(t)
    y = b * np.sin(t) * np.cos(t)

    # Quintic time‑scaling for minimum jerk along the param (0→1)
    s = t / t[-1]
    s5 = 6*s**5 - 15*s**4 + 10*s**3  # smooth start/stop
    # Re‑sample x,y on s5 timeline to get eased velocity
    x = np.interp(s5, s, x)
    y = np.interp(s5, s, y)

    dt = np.full(segments, period / segments)
    dx = np.diff(x)
    dy = np.diff(y)
    return dx.astype(float), dy.astype(float), dt.astype(float)


def figure8_showcase_polynomial(scf: SyncCrazyflie):
    hl = scf.cf.high_level_commander
    print('[SHOW] Figure‑8 (polynomial‑eased) start…')
    hl.takeoff(TAKEOFF_Z, TAKEOFF_TIME); time.sleep(TAKEOFF_TIME + 0.6)

    dx, dy, dts = _lemniscate_samples(FIG8_A, FIG8_B, FIG8_PERIOD, FIG8_SEGMENTS)

    for i in range(len(dts)):
        hl.go_to(float(dx[i]), float(dy[i]), 0.0, 0.0, float(dts[i]), relative=True)
        time.sleep(float(dts[i]))

    hl.land(0.0, LAND_TIME); time.sleep(LAND_TIME + 0.6); hl.stop()
    print('[SHOW] Figure‑8 done.')


def figure8_onboard_poly4d_stub(scf: SyncCrazyflie):
    """
    Placeholder for a true Poly4D uploader. For now, it redirects to the
    smooth segmented figure‑8 and prints a clear note. The proper Poly4D
    implementation requires generating polynomial coefficients per axis and
    loading them into trajectory memory via cf.mem (Poly4D). Happy to add that
    next.
    """
    print('[SHOW] Poly4D stub: running smooth segmented figure‑8 instead…')
    figure8_showcase_polynomial(scf)


# =====================================================================
# MAIN
# =====================================================================
if __name__ == '__main__':
    print('--- Single‑CF Showcase (Hover / Square / Figure‑8) ---')
    print(f'URI={URI} | QTM={QTM_IP} | RigidBody={RIGID_BODY}')

    # Init drivers
    cflib.crtp.init_drivers()

    # Connect to one Crazyflie
    qtm_thread = None
    try:
        with SyncCrazyflie(URI) as scf:
            # QTM + EKF setup
            qtm_thread = init_cf_and_qtm(scf)
            time.sleep(1.0)

            # Pick the routine
            mode = MODE.lower().strip()
            if mode == 'hover':
                hover_showcase(scf)
            elif mode == 'square':
                square_showcase(scf)
            elif mode == 'fig8' or mode == 'figure8':
                figure8_showcase_polynomial(scf)
            elif mode == 'poly4d':
                figure8_onboard_poly4d_stub(scf)
            else:
                print(f"[WARN] Unknown MODE '{MODE}'. Falling back to hover…")
                hover_showcase(scf)

    except KeyboardInterrupt:
        print('\n[MAIN] Interrupted by user.')
    except Exception as e:
        print(f'[MAIN] ERROR: {e}')
    finally:
        # tidy up QTM thread
        if qtm_thread is not None:
            qtm_thread.close()
        print('--- Finished ---')
