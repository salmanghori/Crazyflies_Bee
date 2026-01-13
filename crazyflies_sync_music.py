"""
Swarm Control + QTM + HL-only demos + Beat-synced music show (open loop)

- Hover / Square / Figure-8 shows: High-Level commander only.
- Music show: reads beat timestamps from CSV, plays WAV/MP3 with pygame,
  and schedules HL go_to moves to start exactly on real beats.

Install (once):
  pip install pygame pandas cflib qtm-rt scipy numpy

If you generate beats from MP3:
  pip install librosa soundfile
"""

import asyncio
import math
import time
import xml.etree.cElementTree as ET
from pathlib import Path
from threading import Thread
from typing import Dict, List, Any
import traceback

# Crazyflie / QTM
import cflib.crtp
import qtm_rt
from cflib.crazyflie.swarm import Swarm, CachedCfFactory
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.reset_estimator import reset_estimator
from qtm_rt import QRTCommandException
from scipy.spatial.transform import Rotation

# Music show deps
import threading
import pandas as pd
import pygame  # audio playback (WAV/MP3)

# ====================== USER CONFIG — EDIT THESE ======================
QTM_IP = "192.168.0.105"

URIS = [
    'radio://0/80/2M/E7E7E7E7E1',
    'radio://0/80/2M/E7E7E7E7E2',
]

RIGID_BY_URI: Dict[str, str] = {
    'radio://0/80/2M/E7E7E7E7E1': 'cf_new_1',
    'radio://0/80/2M/E7E7E7E7E2': 'cf_new_2',
}

# Music show paths (set to real files on your machine)
WAV_FILE  = r"C:\Users\ghoriss\Downloads\Vivaldi_60s.wav"        # WAV or MP3
BEATS_CSV = r"C:\Users\ghoriss\Downloads\Vivaldi_beats_60s.csv"  # CSV with 'time_sec' column

# BEATS_CSV     = r"C:\Users\ghoriss\Downloads\Rimsky_1m_slice_beats.csv"
# WAV_FILE     = r"C:\Users\ghoriss\Downloads\Rimsky_1m_slice.wav"
# =====================================================================

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
SQUARE_SIDE_LENGTH = 0.5
SQUARE_MOVE_TIME = 2.5

# --- Figure-8 segmented (HL-only) ---
FIG8_A = 0.35   # X amplitude
FIG8_B = 0.25   # Y amplitude
FIG8_T = 1.2    # sec per segment
FIG8_BUF = 0.08

# --- Music show options ---
PHASE_OFFSETS = {  # per-drone phase offsets in beats (e.g., 0.5 = half-beat)
    'radio://0/80/2M/E7E7E7E7E1': 0.0,
    'radio://0/80/2M/E7E7E7E7E2': 0.5,
}
MUSIC_PREROLL_S = 2.0
AUDIO_LATENCY_COMP_S = -0.01  # optional small constant (e.g., 0.02–0.04) after testing

# =========================================================
# QTM Wrapper
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
        # noinspection PyUnresolvedReferences
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
# HL-only utility helpers
# =========================================================
def _hl_takeoff_wait(cmd, z, t_up, extra=0.7):
    cmd.takeoff(z, t_up)
    time.sleep(t_up + extra)

def _hl_go_rel(cmd, dx, dy, dz, yaw_deg, move_t, buffer=0.10):
    cmd.go_to(dx, dy, dz, yaw_deg, move_t, relative=True)
    time.sleep(move_t + buffer)

def _hl_land_stop(cmd, t_down, extra=0.7):
    cmd.land(0.0, t_down)
    time.sleep(t_down + extra)
    cmd.stop()

# =========================================================
# SHOWCASE SEQUENCES (HL-only, open loop)
# =========================================================
def hover_showcase(scf: SyncCrazyflie):
    cf  = scf.cf
    cmd = cf.high_level_commander
    uri = cf.link_uri
    print(f"[{uri}] Hover showcase start...")
    try:
        _hl_takeoff_wait(cmd, TAKEOFF_Z, TAKEOFF_TIME)
        time.sleep(HOVER_TIME)
        _hl_land_stop(cmd, LAND_TIME)
        print(f"[{uri}] Hover showcase done.")
    except Exception as e:
        print(f"[{uri}] Hover showcase error: {e}")
        try: _hl_land_stop(cmd, LAND_TIME)
        except: pass

def square_showcase(scf: SyncCrazyflie):
    cf  = scf.cf
    cmd = cf.high_level_commander
    uri = cf.link_uri
    print(f"[{uri}] Square showcase start...")
    try:
        _hl_takeoff_wait(cmd, TAKEOFF_Z, TAKEOFF_TIME)
        _hl_go_rel(cmd,  SQUARE_SIDE_LENGTH, 0.0, 0.0, 0.0, SQUARE_MOVE_TIME)
        _hl_go_rel(cmd,  0.0, SQUARE_SIDE_LENGTH, 0.0, 0.0, SQUARE_MOVE_TIME)
        _hl_go_rel(cmd, -SQUARE_SIDE_LENGTH, 0.0, 0.0, 0.0, SQUARE_MOVE_TIME)
        _hl_go_rel(cmd,  0.0,-SQUARE_SIDE_LENGTH, 0.0, 0.0, SQUARE_MOVE_TIME)
        _hl_land_stop(cmd, LAND_TIME)
        print(f"[{uri}] Square showcase done.")
    except Exception as e:
        print(f"[{uri}] Square showcase error: {e}")
        try: _hl_land_stop(cmd, LAND_TIME)
        except: pass

def figure8_showcase(scf: SyncCrazyflie):
    cf  = scf.cf
    cmd = cf.high_level_commander
    uri = cf.link_uri
    print(f"[{uri}] Figure-8 showcase start...")
    try:
        _hl_takeoff_wait(cmd, TAKEOFF_Z, TAKEOFF_TIME)
        legs = [
            (+FIG8_A,  0.00, 0.0), (+0.00, +FIG8_B, 0.0),
            (-FIG8_A, +FIG8_B, 0.0), (-FIG8_A,  0.00, 0.0),
            (-FIG8_A, -FIG8_B, 0.0), (+0.00, -FIG8_B, 0.0),
            (+FIG8_A,  0.00, 0.0),
            (+FIG8_A,  0.00, 0.0), (+0.00, +FIG8_B, 0.0),
            (-FIG8_A, +FIG8_B, 0.0), (-FIG8_A,  0.00, 0.0),
            (-FIG8_A, -FIG8_B, 0.0), (+0.00, -FIG8_B, 0.0),
            (+FIG8_A,  0.00, 0.0),
        ]
        for dx, dy, dz in legs:
            cmd.go_to(dx, dy, dz, 0.0, FIG8_T, relative=True)
            time.sleep(FIG8_T + FIG8_BUF)
        _hl_land_stop(cmd, LAND_TIME)
        print(f"[{uri}] Figure-8 showcase done.")
    except Exception as e:
        print(f"[{uri}] Figure-8 showcase error: {e}")
        try: _hl_land_stop(cmd, LAND_TIME)
        except: pass


def figure8_showcase_smoother(scf: SyncCrazyflie):
    """HL-only segmented figure-8 with many tiny legs and no inter-leg buffer."""
    cf  = scf.cf
    hl  = cf.high_level_commander
    uri = cf.link_uri
    print(f"[{uri}] Figure-8 smoother (HL segments) start...")

    try:
        hl.takeoff(TAKEOFF_Z, TAKEOFF_TIME); time.sleep(TAKEOFF_TIME + 0.7)

        # Parametric lemniscate of Gerono: x=A*sin(t), y=B*sin(t)*cos(t)
        A, B = 0.35, 0.25
        T_total = 12.0       # seconds for a full 8 (t from 0..2π)
        N = 48               # segments (more segments = smoother)
        seg_T = T_total / N  # duration per segment

        # Build small relative steps between consecutive parametric points
        import numpy as np
        t = np.linspace(0, 2*np.pi, N+1)
        x = A*np.sin(t)
        y = B*np.sin(t)*np.cos(t)
        z = np.zeros_like(x)

        for i in range(N):
            dx = float(x[i+1] - x[i])
            dy = float(y[i+1] - y[i])
            dz = float(z[i+1] - z[i])
            hl.go_to(dx, dy, dz, 0.0, seg_T, relative=True)
            time.sleep(seg_T)  # no extra buffer

        hl.land(0.0, LAND_TIME); time.sleep(LAND_TIME + 0.7)
        hl.stop()
        print(f"[{uri}] Figure-8 smoother done.")
    except Exception as e:
        print(f"[{uri}] Figure-8 smoother error: {e}")
        try: hl.land(0.0, LAND_TIME); time.sleep(LAND_TIME + 0.7); hl.stop()
        except: pass

# =========================================================
# MUSIC SHOW (HL-only, beat-synced via pygame)
# =========================================================
MUSIC_START_EVENT = threading.Event()
MUSIC_T0 = {"t0": None}  # monotonic start time shared across threads

def load_beats_csv(csv_path: str):
    df = pd.read_csv(csv_path)
    if "time_sec" not in df.columns:
        raise RuntimeError(f"CSV {csv_path} must have a 'time_sec' column.")
    beats = df["time_sec"].to_numpy()
    if beats.size < 2:
        raise RuntimeError("Not enough beats found in CSV.")
    return beats

def start_music_once_and_set_t0(wav_path: str, pre_roll_s: float = 2.0):
    """
    Starts audio (WAV or MP3) with pygame and publishes a common start time t0.
    Safe to call multiple times; only the first call actually starts playback.
    """
    if MUSIC_T0["t0"] is not None:
        MUSIC_START_EVENT.set()
        return

    print(f"[Music] Starting in {pre_roll_s:.1f}s...")
    time.sleep(pre_roll_s)

    if not pygame.mixer.get_init():
        pygame.mixer.init(buffer=256)  # default device & sample rate

    try:
        pygame.mixer.music.load(wav_path)  # supports WAV/MP3
    except Exception as e:
        raise RuntimeError(f"Failed to load audio file: {wav_path} ({e})")

    pygame.mixer.music.play()  # non-blocking
    t0 = time.perf_counter()   # high-res monotonic
    MUSIC_T0["t0"] = t0
    MUSIC_START_EVENT.set()
    print("[Music] PLAY! t0 published.")

def music_showcase_from_beats_factory(
    wav_path: str,
    beats_csv_path: str,
    score_by_uri: dict,
    phase_offset_beats: dict = None,
    pre_roll_s: float = 2.0,
):
    """
    score_by_uri: { uri: [ (b_start, b_end, dx, dy, dz, yaw_deg), ... ] }
      Durations are computed from absolute beat times: beat_times[b_end] - beat_times[b_start]
    """
    phase_offset_beats = phase_offset_beats or {}
    leader_uri = URIS[0]

    def _runner(scf: SyncCrazyflie):
        uri = scf.cf.link_uri
        hl  = scf.cf.high_level_commander
        try:
            beat_times = load_beats_csv(beats_csv_path)

            # Takeoff before music
            hl.takeoff(TAKEOFF_Z, TAKEOFF_TIME)
            time.sleep(TAKEOFF_TIME + 0.8)

            # Music: leader plays, others wait
            if uri == leader_uri:
                start_music_once_and_set_t0(wav_path, pre_roll_s=pre_roll_s)
            else:
                MUSIC_START_EVENT.wait()
            t0 = MUSIC_T0["t0"]
            if t0 is None:
                raise RuntimeError("Music t0 not set.")

            # Approx avg beat period for phase delay
            avg_period = (beat_times[-1] - beat_times[0]) / (len(beat_times) - 1)
            phase_beats = phase_offset_beats.get(uri, 0.0)
            phase_delay = phase_beats * avg_period - AUDIO_LATENCY_COMP_S

            # Align to the first beat + phase
            first_when = t0 + beat_times[0] + phase_delay
            now = time.perf_counter()
            if now < first_when:
                time.sleep(first_when - now)

            # Execute the score
            for (b_start, b_end, dx, dy, dz, yaw_deg) in score_by_uri.get(uri, []):
                if b_start < 0 or b_end <= b_start or b_end >= len(beat_times):
                    print(f"[{uri}] Skipping invalid beat span ({b_start}->{b_end})")
                    continue

                move_t0 = t0 + beat_times[b_start] + phase_delay
                move_t1 = t0 + beat_times[b_end]   + phase_delay
                duration = max(0.25, move_t1 - move_t0)

                # Wait for exact start
                now = time.perf_counter()
                if now < move_t0:
                    time.sleep(move_t0 - now)

                # Fire HL move (straight segment, relative)
                hl.go_to(dx, dy, dz, yaw_deg, duration, relative=True)

                # Sleep until end (with tiny buffer)
                left = move_t1 - time.perf_counter()
                if left > 0:
                    time.sleep(left + 0.05)

            # Land
            hl.land(0.0, LAND_TIME)
            time.sleep(LAND_TIME + 0.7)
            hl.stop()
            print(f"[{uri}] Music-beat showcase complete.")

        except Exception as e:
            print(f"[{uri}] Music-beat showcase error: {e}")
            try:
                hl.land(0.0, LAND_TIME); time.sleep(LAND_TIME + 0.7); hl.stop()
            except:
                pass

    return _runner

# ===== Example music score (uses beat indices) =====
S = SQUARE_SIDE_LENGTH
Z = 0.0

def square_span(start, leg_beats=4):
    return [
        (start+0, start+leg_beats*1, +S, 0.0, Z, 0),
        (start+leg_beats*1, start+leg_beats*2, 0.0, +S, Z, 0),
        (start+leg_beats*2, start+leg_beats*3, -S, 0.0, Z, 0),
        (start+leg_beats*3, start+leg_beats*4, 0.0, -S, Z, 0),
    ]

# Drone 1 pattern
score_d1 = []
score_d1 += square_span(0, 4)  # beats 0..16
score_d1 += [(16, 20, +S, +S, Z, 0), (20, 24, -S, -S, Z, 0)]  # diagonals
fig8_legs = [(+S,0),(0,+S),(-S,0),(0,-S),(-S,0),(0,+S),(+S,0),(0,-S)]
b = 24
for (dx, dy) in fig8_legs:
    score_d1.append((b, b+2, dx*0.7, dy*0.5, Z, 0)); b += 2
score_d1.append((b, min(b+8, 150), 0.0, 0.0, Z, 0))  # final hold (up to ~beat 150)

# Drone 2 pattern (mirrored; PHASE_OFFSETS adds half-beat delay)
score_d2 = []
score_d2 += [(0,4,-S,0.0,Z,0),(4,8,0.0,+S,Z,0),(8,12,+S,0.0,Z,0),(12,16,0.0,-S,Z,0)]
score_d2 += [(16,20,-S,+S,Z,0),(20,24,+S,-S,Z,0)]
b = 24
for (dx, dy) in fig8_legs:
    score_d2.append((b, b+2, -dx*0.7, dy*0.5, Z, 0)); b += 2
score_d2.append((b, min(b+8, 150), 0.0, 0.0, Z, 0))

SCORE_BY_URI = {
    'radio://0/80/2M/E7E7E7E7E1': score_d1,
    'radio://0/80/2M/E7E7E7E7E2': score_d2,
}

MUSIC_SHOW = music_showcase_from_beats_factory(
    wav_path=WAV_FILE,
    beats_csv_path=BEATS_CSV,
    score_by_uri=SCORE_BY_URI,
    phase_offset_beats=PHASE_OFFSETS,
    pre_roll_s=MUSIC_PREROLL_S,
)

# =========================================================
# PREFLIGHT CHECKS
# =========================================================
def preflight_checks():
    print("--- Preflight checks ---")
    # 1) URIS ↔ rigid mapping
    missing = [u for u in URIS if u not in RIGID_BY_URI]
    if missing:
        raise ValueError(f"Missing rigid-body mapping for: {missing}")
    print("✓ URIS ↔ rigid-body mapping OK")

    # 2) Music files present (if you run MUSIC_SHOW)
    wav_p = Path(WAV_FILE)
    csv_p = Path(BEATS_CSV)
    if not wav_p.exists():
        raise FileNotFoundError(f"WAV/MP3 not found: {wav_p}")
    if not csv_p.exists():
        raise FileNotFoundError(f"Beats CSV not found: {csv_p}")
    print(f"✓ Music files found\n   Audio: {wav_p}\n   CSV:   {csv_p}")

    # 3) CSV sanity
    df = pd.read_csv(csv_p)
    if "time_sec" not in df.columns or len(df["time_sec"]) < 4:
        raise ValueError("Beats CSV must contain a 'time_sec' column with multiple rows.")
    print(f"✓ Beats loaded: {len(df)} entries "
          f"(first={df['time_sec'].iloc[0]:.3f}s, last={df['time_sec'].iloc[-1]:.3f}s)")

    # 4) Announce leader & phases
    leader = URIS[0]
    print(f"✓ Music leader URI: {leader}")
    print(f"✓ Phase offsets (beats): {PHASE_OFFSETS}")
    print("--- Preflight checks passed ---")

# =========================================================
# MAIN (with preflight + clearer logs)
# =========================================================
if __name__ == '__main__':
    print("--- Drone Music Showcase Initializing ---")
    try:
        preflight_checks()  # fast-fail before touching radios/QTM

        print("Initializing Crazyflie drivers...")
        cflib.crtp.init_drivers()
        factory = CachedCfFactory(rw_cache='./cache')
        print("✓ Crazyflie drivers initialized.")

        with Swarm(URIS, factory=factory) as swarm:
            print("\n[Swarm] Connected. Initializing QTM + estimators...")
            swarm.parallel_safe(init_cf)
            print("✓ All drones initialized and receiving QTM extpos.")

            print("\n[Swarm] Starting the show sequence...")
            # Pick ONE:
            # swarm.parallel_safe(hover_showcase)
            # swarm.parallel_safe(square_showcase)
            # swarm.parallel_safe(figure8_showcase)
            # swarm.parallel_safe(figure8_showcase_smoother)
            swarm.parallel_safe(MUSIC_SHOW)
            print("\n[Swarm] Show sequence finished.")

    except Exception as e:
        print(f"\n[Swarm] A critical error occurred: {e}")
        traceback.print_exc()
    finally:
        print("[Swarm] Shutting down QTM connections...")
        for uri, wrapper in qtm_by_uri.items():
            if wrapper: wrapper.close()
        print("--- Script Finished ---")
