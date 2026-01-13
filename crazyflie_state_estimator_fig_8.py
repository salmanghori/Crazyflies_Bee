# import asyncio
# import math
# import time
# import statistics
# import xml.etree.cElementTree as ET
# from collections import deque
# from threading import Thread, Event
#
# import qtm_rt
# from qtm_rt import QRTCommandException
# from scipy.spatial.transform import Rotation
#
# import cflib.crtp
# from cflib.crazyflie import Crazyflie
# from cflib.crazyflie.mem import MemoryElement, Poly4D
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.utils import uri_helper
# from cflib.utils.reset_estimator import reset_estimator
#
# # ============================================
# # --- 1. CONFIGURATION ---
# # ============================================
#
# # --- Crazyflie Setup ---
# URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
#
# # --- QTM Setup ---
# QTM_IP = "192.168.0.105"
# RIGID_BODY_NAME = r'cf8'
#
# # --- Flight Control ---
# SEND_RATE_HZ = 60
# SEND_FULL_POSE = False  # start position-only; flip to True after you’ve verified axes
#
# # --- Failsafe for Unstable Tracking ---
# QTM_TIMEOUT_S = 0.7       # stale if older than this
# QTM_REACQUIRE_S = 1.0     # must be stable this long before takeoff
# PRETAKEOFF_FRESH_STREAK = 20  # consecutive fresh packets required before takeoff
#
# # --- Onboard Estimator Tuning ---
# EXTPOS_STD_DEV = 0.01          # meters (std dev). Larger => less trust in QTM.
# EXTATT_STD_DEV_RAD = 0.1       # radians, if using full pose
#
# # --- Trajectory Parameters ---
# TAKEOFF_HEIGHT = 0.7  # meters
# TAKEOFF_DURATION = 3.0  # seconds
# LAND_DURATION = 4.0  # seconds
#
# # Figure-8 trajectory data: [duration, x(8), y(8), yaw(8), z(8)]
# FIGURE8_TRAJECTORY = [
#     [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493, -0.000000, 0.000000,
#      -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
#      0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
#     [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049, -0.445604, -0.684403,
#      0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
#      0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
#     [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514,
#      0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
#      0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
#     [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722,
#      -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
#      0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
#     [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627, 0.450742, -0.385534,
#      -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
#      0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
#     [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130, 0.001593, -1.031457,
#      0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
#      0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
#     [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354,
#      -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0.000000, 0.000000, 0.000000, 0.000000,
#      0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
#      0.000000],
#     [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682,
#      0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
#      0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
#     [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294, 0.288570, 0.873350,
#      -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
#      0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
#     [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567, 0.447039, -0.532729,
#      -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
#      0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
# ]
#
# # ============================================
# # Helpers
# # ============================================
#
# class RateMeter:
#     """Simple wall-clock Hz estimator."""
#     def __init__(self, window_s=1.0):
#         self.window_s = window_s
#         self.last_t = time.time()
#         self.count = 0
#
#     def tick(self, label=None):
#         self.count += 1
#         now = time.time()
#         dt = now - self.last_t
#         if dt >= self.window_s:
#             hz = self.count / dt if dt > 0 else 0.0
#             if label:
#                 print(f"[RATE] {label}: {hz:.1f} Hz")
#             self.last_t = now
#             self.count = 0
#             return hz
#         return None
#
# def _set_param_or_warn(cf, key, value) -> bool:
#     try:
#         cf.param.set_value(key, value)
#         print(f"[CFG] {key} = {value}")
#         return True
#     except Exception:
#         print(f"[CFG] WARN: param '{key}' not found")
#         return False
#
# def soft_reboot_cf(uri: str, settle_s: float = 2.0):
#     """Connect -> stop HL -> reset EKF -> close -> wait."""
#     print(f"[PRE] Connecting to {uri} ...")
#     cflib.crtp.init_drivers()
#     try:
#         with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
#             cf = scf.cf
#             try: cf.param.set_value('commander.enHighLevel', '0')
#             except Exception: pass
#             try: cf.high_level_commander.stop()
#             except Exception: pass
#             print("[PRE] Resetting onboard estimator (EKF) ...")
#             reset_estimator(cf)
#             try: cf.param.set_value('commander.enHighLevel', '1')
#             except Exception: pass
#             print("[PRE] Soft reset done. Closing link...")
#     except Exception as e:
#         print(f"[PRE] Soft reboot failed: {e}")
#     print(f"[PRE] Waiting {settle_s:.1f}s for tasks to settle...")
#     time.sleep(settle_s)
#     print("[PRE] Ready for main script.")
#
# # ============================================
# # QTM Wrapper (with live rate estimation)
# # ============================================
#
# class QtmWrapper:
#     """Connects to QTM, streams 6DoF data, provides latest pose and rate estimates."""
#
#     def __init__(self, body_name, qtm_ip):
#         self._body_name = body_name
#         self._qtm_ip = qtm_ip
#         self._stay_open = True
#         self.latest_pose = None
#         self.last_packet_timestamp = 0.0
#         self._body_indices = {}
#         self._pkt_count = 0
#
#         # Rolling timing to estimate QTM fps
#         self._ts_window = deque(maxlen=200)  # ~ couple seconds at 100 Hz
#
#         self._thread = Thread(target=self._run, daemon=True)
#         self._thread.start()
#
#     def _on_packet(self, packet):
#         header, bodies = packet.get_6d()
#         if bodies is None: return
#         try:
#             body_index = self._body_indices[self._body_name]
#             pos, rot_matrix = bodies[body_index]
#         except (KeyError, IndexError):
#             return
#         if pos is None or any(math.isnan(v) for v in pos):
#             return
#
#         # position mm->m
#         x, y, z = pos[0] / 1000.0, pos[1] / 1000.0, pos[2] / 1000.0
#         # col-major -> row-major
#         r = rot_matrix.matrix
#         rot = [[r[0], r[3], r[6]], [r[1], r[4], r[7]], [r[2], r[5], r[8]]]
#
#         now = time.time()
#         self.latest_pose = (x, y, z, rot)
#         self.last_packet_timestamp = now
#         self._pkt_count += 1
#         self._ts_window.append(now)
#
#     def get_qtm_rate_hz(self):
#         """Estimated incoming QTM frame rate based on rolling window."""
#         ts = list(self._ts_window)
#         if len(ts) < 2:
#             return 0.0, 0.0, 0.0
#         # Inter-arrival times
#         dts = [t2 - t1 for t1, t2 in zip(ts[:-1], ts[1:]) if t2 > t1]
#         if not dts:
#             return 0.0, 0.0, 0.0
#         avg_dt = statistics.mean(dts)
#         min_dt = min(dts)
#         max_dt = max(dts)
#         avg_hz = 1.0 / avg_dt if avg_dt > 0 else 0.0
#         return avg_hz, min_dt, max_dt
#
#     async def _main_loop(self):
#         connection = None
#         while self._stay_open:
#             try:
#                 print(f"[QTM] Connecting to {self._qtm_ip} ...")
#                 connection = await qtm_rt.connect(self._qtm_ip)
#                 if connection is None:
#                     raise ConnectionError("Failed to connect")
#                 print("[QTM] Connected.")
#
#                 params = await connection.get_parameters(parameters=['6d'])
#                 xml = ET.fromstring(params)
#                 labels = [label.text.strip() for label in xml.findall('*/Body/Name')]
#                 self._body_indices = {name: i for i, name in enumerate(labels)}
#                 print(f"[QTM] Bodies: {labels}")
#
#                 if self._body_name not in self._body_indices:
#                     print(f"[QTM] ERROR: Body '{self._body_name}' not found!")
#                     break
#
#                 await connection.stream_frames(components=['6d'], on_packet=self._on_packet)
#                 print(f"[QTM] Streaming frames for '{self._body_name}' ...")
#                 while self._stay_open:
#                     # Periodic debug
#                     avg_hz, min_dt, max_dt = self.get_qtm_rate_hz()
#                     if avg_hz > 0:
#                         print(f"[QTM] est rate ~ {avg_hz:5.1f} Hz (min dt {min_dt*1000:5.1f} ms, max dt {max_dt*1000:5.1f} ms)")
#                     await asyncio.sleep(1.0)
#
#             except (ConnectionError, asyncio.TimeoutError, QRTCommandException) as e:
#                 print(f"[QTM] Connection lost/failed: {e}")
#                 if connection:
#                     try: await connection.stream_frames_stop()
#                     except Exception: pass
#                     try: connection.disconnect()
#                     except Exception: pass
#                 self.latest_pose = None
#                 self._pkt_count = 0
#                 self._ts_window.clear()
#                 await asyncio.sleep(2)
#
#         # Cleanup
#         if connection:
#             try: await connection.stream_frames_stop()
#             except Exception: pass
#             try: connection.disconnect()
#             except Exception: pass
#         print("[QTM] Wrapper shut down.")
#
#     def _run(self):
#         asyncio.run(self._main_loop())
#
#     def close(self):
#         self._stay_open = False
#         time.sleep(0.2)
#         self._thread.join(timeout=1.0)
#
# # ============================================
# # Flight Controller
# # ============================================
#
# class FlightController:
#     """Manages Crazyflie connection, configuration, and flight sequences."""
#
#     def __init__(self, uri, qtm_wrapper):
#         self._uri = uri
#         self._qtm = qtm_wrapper
#         self._scf = SyncCrazyflie(self._uri, cf=Crazyflie(rw_cache='./cache'))
#         self._cf = None
#         self._commander = None
#         self._stop_event = Event()
#         self._flight_thread = None
#         self.hover_position = [0, 0, 0]
#
#     def __enter__(self):
#         self._scf.open_link()
#         self._cf = self._scf.cf
#         self._commander = self._cf.high_level_commander
#         return self
#
#     def __exit__(self, exc_type, exc_val, exc_tb):
#         print("Shutdown requested. Landing and disconnecting...")
#         self._stop_event.set()
#         if self._flight_thread and self._flight_thread.is_alive():
#             self._flight_thread.join(timeout=2.0)
#
#         if self._commander:
#             try:
#                 self._commander.land(0.0, LAND_DURATION)
#                 time.sleep(LAND_DURATION)
#                 self._commander.stop()
#             except Exception as e:
#                 print(f"Error during final landing sequence: {e}")
#
#         self._scf.close_link()
#         print("Disconnected from Crazyflie.")
#
#     def _setup_estimator_and_controller(self):
#         print("Configuring onboard estimator and controller...")
#         self._cf.param.set_value('commander.enHighLevel', '1')
#         self._cf.param.set_value('stabilizer.estimator', '2')
#
#         # Prefer modern params; fall back to legacy names.
#         if not _set_param_or_warn(self._cf, 'locSrv.extPosStdDev', str(EXTPOS_STD_DEV)):
#             _set_param_or_warn(self._cf, 'extpos.stdDev', str(EXTPOS_STD_DEV))
#
#         if SEND_FULL_POSE:
#             _set_param_or_warn(self._cf, 'locSrv.extQuatStdDev', str(EXTATT_STD_DEV_RAD))
#
#         self._cf.param.set_value('stabilizer.controller', '2')
#
#         print("Resetting estimator...")
#         reset_estimator(self._cf)
#         print("Configuration complete.")
#
#     def _upload_trajectory(self, trajectory_id, trajectory_data):
#         try:
#             traj_mems = self._cf.mem.get_mems(MemoryElement.TYPE_TRAJ)
#         except (KeyError, AttributeError):
#             traj_mems = []
#
#         if not traj_mems:
#             print("[FATAL] Trajectory memory not found in firmware!")
#             print("Please build firmware with TRAJECTORY_MEMORY=1.")
#             return None
#
#         traj_mem = traj_mems[0]
#         print(f"Uploading trajectory {trajectory_id}...")
#         total_duration = 0.0
#         pieces = []
#         for row in trajectory_data:
#             duration = row[0]
#             x = Poly4D.Poly(row[1:9])
#             y = Poly4D.Poly(row[9:17])
#             yaw = Poly4D.Poly(row[17:25])
#             z = Poly4D.Poly(row[25:33])
#             pieces.append(Poly4D(duration, x, y, z, yaw))
#             total_duration += duration
#
#         traj_mem.upload_polynomials(pieces, 0)
#         self._commander.define_trajectory(trajectory_id, 0, len(pieces))
#         print(f"Upload complete. Trajectory duration: {total_duration:.2f}s")
#         return total_duration
#
#     def _send_external_pose(self, x, y, z, rot_matrix):
#         if SEND_FULL_POSE:
#             try:
#                 if (isinstance(rot_matrix, (list, tuple)) and len(rot_matrix) == 3
#                         and all(isinstance(r, (list, tuple)) and len(r) == 3 for r in rot_matrix)):
#                     quat = Rotation.from_matrix(rot_matrix).as_quat()
#                     self._cf.extpos.send_extpose(x, y, z, *quat)
#                 else:
#                     self._cf.extpos.send_extpos(x, y, z)
#             except ValueError:
#                 self._cf.extpos.send_extpos(x, y, z)
#         else:
#             self._cf.extpos.send_extpos(x, y, z)
#
#     def _main_flight_loop(self, trajectory_id, duration):
#         """Send QTM -> CF at SEND_RATE_HZ and show the *actual* radio send rate."""
#         radio_meter = RateMeter(window_s=1.0)
#
#         is_tracking = False
#         in_failsafe = False
#         fresh_count = 0
#
#         print("\n--- Waiting for stable QTM signal before takeoff ---")
#         while not self._stop_event.is_set():
#             last_seen = self._qtm.last_packet_timestamp
#             pose = self._qtm.latest_pose
#             now = time.time()
#             is_fresh = bool(pose) and (now - last_seen) < QTM_TIMEOUT_S
#
#             if is_fresh:
#                 if not is_tracking:
#                     print("[QTM] Tracking signal acquired.")
#                 is_tracking = True
#                 fresh_count += 1
#                 self._send_external_pose(*pose)
#                 radio_meter.tick(label="radio send")  # show the effective radio rate
#             else:
#                 if is_tracking:
#                     print("[QTM] WARNING: Tracking signal lost!")
#                 is_tracking = False
#                 fresh_count = 0
#
#             if fresh_count >= PRETAKEOFF_FRESH_STREAK:
#                 break
#
#             time.sleep(max(0.0, 1.0 / SEND_RATE_HZ))
#
#         if self._stop_event.is_set():
#             return
#
#         print(f"--- Takeoff to {TAKEOFF_HEIGHT} m ---")
#         self.hover_position = [self._qtm.latest_pose[0], self._qtm.latest_pose[1], TAKEOFF_HEIGHT]
#         self._commander.takeoff(TAKEOFF_HEIGHT, TAKEOFF_DURATION)
#         time.sleep(TAKEOFF_DURATION + 1.0)
#
#         print(f"--- Starting Trajectory {trajectory_id} ---")
#         self._commander.start_trajectory(trajectory_id, 1.0, relative=False)
#
#         start_time = time.time()
#         while time.time() - start_time < duration and not self._stop_event.is_set():
#             last_seen = self._qtm.last_packet_timestamp
#             pose = self._qtm.latest_pose
#             now = time.time()
#
#             if pose and (now - last_seen) < QTM_TIMEOUT_S:
#                 self._send_external_pose(*pose)
#                 radio_meter.tick(label="radio send")
#                 if in_failsafe:
#                     print("[QTM] Tracking reacquired. Resuming trajectory.")
#                     self._commander.start_trajectory(trajectory_id, 1.0, relative=False)
#                     in_failsafe = False
#             else:
#                 if not in_failsafe:
#                     print("[FAILSAFE] Tracking lost! Pausing trajectory and hovering.")
#                     self._commander.stop()
#                     in_failsafe = True
#
#             time.sleep(max(0.0, 1.0 / SEND_RATE_HZ))
#
#         if in_failsafe:
#             print("\n--- Trajectory ended while in failsafe ---")
#         else:
#             print("\n--- Trajectory complete ---")
#
#     def run_sequence(self):
#         """Execute full flight plan."""
#         self._setup_estimator_and_controller()
#         trajectory_id = 1
#         duration = self._upload_trajectory(trajectory_id, FIGURE8_TRAJECTORY)
#         if duration is None:
#             return
#         self._flight_thread = Thread(target=self._main_flight_loop, args=(trajectory_id, duration))
#         self._flight_thread.start()
#         self._flight_thread.join()
#
# # ============================================
# # QTM pre-benchmark (before any radio activity)
# # ============================================
#
# def qtm_pre_benchmark(qtm_wrapper: QtmWrapper, sample_s: float = 3.0):
#     """Wait for first pose, then measure incoming QTM rate for sample_s seconds."""
#     print("Initializing QTM benchmark... (waiting for first pose)")
#     deadline = time.time() + 8.0
#     while qtm_wrapper.latest_pose is None and time.time() < deadline:
#         time.sleep(0.02)
#
#     if qtm_wrapper.latest_pose is None:
#         print("\n[FATAL] No pose from QTM within timeout.")
#         return False
#
#     print("[QTM] First pose received. Measuring rate...")
#     start = time.time()
#     start_count = qtm_wrapper._pkt_count
#     # Also collect an explicit dt sample set
#     sample_ts = []
#     last_seen = qtm_wrapper.last_packet_timestamp
#     while time.time() - start < sample_s:
#         if qtm_wrapper.last_packet_timestamp != last_seen:
#             last_seen = qtm_wrapper.last_packet_timestamp
#             sample_ts.append(last_seen)
#         time.sleep(0.001)
#
#     got = qtm_wrapper._pkt_count - start_count
#     elapsed = time.time() - start
#     est_hz = got / elapsed if elapsed > 0 else 0.0
#
#     # Compute dt stats from sample_ts
#     dts = [t2 - t1 for t1, t2 in zip(sample_ts[:-1], sample_ts[1:]) if t2 > t1]
#     if dts:
#         avg_dt = statistics.mean(dts)
#         min_dt = min(dts)
#         max_dt = max(dts)
#         print(f"[QTM] Bench: ~{est_hz:.1f} Hz over {elapsed:.2f}s "
#               f"(avg dt {avg_dt*1000:.1f} ms, min {min_dt*1000:.1f} ms, max {max_dt*1000:.1f} ms)")
#     else:
#         print(f"[QTM] Bench: ~{est_hz:.1f} Hz over {elapsed:.2f}s (insufficient dt samples)")
#
#     return True
#
# # ============================================
# # Main
# # ============================================
#
# def main():
#     # 1) Start QTM first and benchmark its incoming speed
#     qtm = QtmWrapper(RIGID_BODY_NAME, QTM_IP)
#     try:
#         ok = qtm_pre_benchmark(qtm, sample_s=3.0)
#         if not ok:
#             print("Aborting due to missing QTM pose.")
#             return
#
#         # 2) Only now touch the radio: soft reboot to clean the EKF state
#         cflib.crtp.init_drivers()
#         soft_reboot_cf(URI, settle_s=2.0)
#
#         # 3) Fly and show radio send frequency live
#         with FlightController(URI, qtm) as controller:
#             controller.run_sequence()
#
#     except KeyboardInterrupt:
#         print("\nKeyboard interrupt detected. Shutting down.")
#     except Exception as e:
#         print(f"\nAn unhandled error occurred: {e}")
#     finally:
#         qtm.close()
#         print("Script finished.")
#
# if __name__ == '__main__':
#     main()


import time
import math
import threading
from collections import deque

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.mem import MemoryElement
from cflib.utils import uri_helper

import qtm_rt
import asyncio
import xml.etree.ElementTree as ET

# ============================================
# CONFIGURATION
# ============================================

# Crazyflie configuration
URI = 'radio://0/80/2M/E7E7E7E7E7'  # Change to your Crazyflie address

# QTM configuration
QTM_IP = "192.168.0.105"  # Change to your QTM computer IP
RIGID_BODY = "cf8"  # Change to your rigid body name in QTM

# Flight parameters
TAKEOFF_HEIGHT = 0.7  # meters
TAKEOFF_TIME = 3.0  # seconds
LAND_TIME = 3.0  # seconds

# Figure-8 trajectory (simplified - duration, x, y, z, yaw)
FIGURE_8_WAYPOINTS = [
    [2.0, 0.5, 0.0, TAKEOFF_HEIGHT, 0],
    [3.0, 0.5, 0.5, TAKEOFF_HEIGHT, 0],
    [3.0, 0.0, 0.5, TAKEOFF_HEIGHT, 0],
    [3.0, -0.5, 0.5, TAKEOFF_HEIGHT, 0],
    [3.0, -0.5, 0.0, TAKEOFF_HEIGHT, 0],
    [3.0, -0.5, -0.5, TAKEOFF_HEIGHT, 0],
    [3.0, 0.0, -0.5, TAKEOFF_HEIGHT, 0],
    [3.0, 0.5, -0.5, TAKEOFF_HEIGHT, 0],
    [2.0, 0.5, 0.0, TAKEOFF_HEIGHT, 0],
]


# ============================================
# QTM QUALISYS INTERFACE
# ============================================

class QTMInterface:
    def __init__(self, ip, body_name):
        self.ip = ip
        self.body_name = body_name
        self.connection = None
        self.running = True
        self.latest_pose = None  # (x, y, z, rotation_matrix)
        self.body_index = None

    async def connect(self):
        """Connect to QTM and start streaming"""
        print(f"Connecting to QTM at {self.ip}...")
        self.connection = await qtm_rt.connect(self.ip)

        # Get parameters to find body index
        params = await self.connection.get_parameters(parameters=["6d"])
        xml_root = ET.fromstring(params)

        bodies = []
        for index, body in enumerate(xml_root.findall(".//Body/Name")):
            name = body.text.strip()
            bodies.append(name)
            if name == self.body_name:
                self.body_index = index

        print(f"Found bodies: {bodies}")

        if self.body_index is None:
            raise Exception(f"Body '{self.body_name}' not found in QTM")

        # Start streaming
        await self.connection.stream_frames(components=["6d"], on_packet=self._on_packet)
        print("QTM streaming started")

    def _on_packet(self, packet):
        """Process incoming QTM data packet"""
        header, bodies = packet.get_6d()

        if bodies and self.body_index < len(bodies):
            position, rotation = bodies[self.body_index]

            if position is not None:
                # Convert mm to meters
                x, y, z = position[0] / 1000, position[1] / 1000, position[2] / 1000

                # Convert rotation matrix to proper format
                rot_matrix = [
                    [rotation.matrix[0], rotation.matrix[3], rotation.matrix[6]],
                    [rotation.matrix[1], rotation.matrix[4], rotation.matrix[7]],
                    [rotation.matrix[2], rotation.matrix[5], rotation.matrix[8]]
                ]

                self.latest_pose = (x, y, z, rot_matrix)

    async def run(self):
        """Main QTM run loop"""
        await self.connect()
        while self.running:
            await asyncio.sleep(0.1)

    def stop(self):
        """Stop QTM connection"""
        self.running = False
        if self.connection:
            asyncio.run_coroutine_threadsafe(
                self.connection.stream_frames_stop(),
                asyncio.get_event_loop()
            )


# ============================================
# CRAZYFLIE CONTROL
# ============================================

class CrazyflieController:
    def __init__(self, uri, qtm_interface):
        self.uri = uri
        self.qtm = qtm_interface
        self.cf = None
        self.commander = None
        self.running = True

    def connect(self):
        """Connect to Crazyflie and setup parameters"""
        print(f"Connecting to Crazyflie at {self.uri}...")

        cflib.crtp.init_drivers()
        self.cf = Crazyflie(rw_cache='./cache')

        with SyncCrazyflie(self.uri, cf=self.cf) as scf:
            self.cf = scf.cf
            self.commander = self.cf.high_level_commander

            # Configure estimator to use external positioning
            print("Configuring estimator...")
            self.cf.param.set_value('stabilizer.estimator', '2')  # Use EKF
            self.cf.param.set_value('commander.enHighLevel', '1')
            self.cf.param.set_value('locSrv.extPosStdDev', '0.01')

            # Reset estimator
            self._reset_estimator()

            print("Crazyflie connected and configured")

    def _reset_estimator(self):
        """Reset the Crazyflie position estimator"""
        self.cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self.cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2.0)  # Wait for estimator to stabilize

    def send_external_position(self, x, y, z):
        """Send external position estimate to Crazyflie"""
        try:
            self.cf.extpos.send_extpos(x, y, z)
        except Exception as e:
            print(f"Error sending position: {e}")

    def takeoff(self, height, duration):
        """Execute takeoff maneuver"""
        print(f"Taking off to {height}m...")
        self.commander.takeoff(height, duration)
        time.sleep(duration + 0.5)  # Wait for takeoff to complete

    def land(self, duration):
        """Execute landing maneuver"""
        print("Landing...")
        self.commander.land(0.0, duration)
        time.sleep(duration + 0.5)
        self.commander.stop()

    def goto(self, x, y, z, yaw=0, duration=2.0):
        """Go to specific position"""
        self.commander.go_to(x, y, z, yaw, duration)
        time.sleep(duration + 0.1)

    def run_figure_8(self):
        """Execute figure-8 flight pattern"""
        print("Starting figure-8 trajectory...")

        # Send QTM data at high rate during flight
        pose_thread = threading.Thread(target=self._send_pose_loop, daemon=True)
        pose_thread.start()

        try:
            # Execute waypoints
            for duration, x, y, z, yaw in FIGURE_8_WAYPOINTS:
                if not self.running:
                    break

                print(f"Going to: x={x:.1f}, y={y:.1f}, z={z:.1f}")
                self.goto(x, y, z, yaw, duration)

        except KeyboardInterrupt:
            print("Figure-8 interrupted by user")
        finally:
            self.running = False

    def _send_pose_loop(self):
        """Continuously send QTM pose data to Crazyflie"""
        while self.running:
            if self.qtm.latest_pose:
                x, y, z, rot = self.qtm.latest_pose
                self.send_external_position(x, y, z)
            time.sleep(0.02)  # ~50 Hz update rate


# ============================================
# MAIN APPLICATION
# ============================================

def run_qtm_in_thread(qtm_interface):
    """Run QTM interface in a separate thread"""

    def run():
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(qtm_interface.run())

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    return thread


def wait_for_qtm_tracking(qtm_interface, timeout=10):
    """Wait for QTM to start tracking the rigid body"""
    print("Waiting for QTM tracking...")
    start_time = time.time()

    while time.time() - start_time < timeout:
        if qtm_interface.latest_pose is not None:
            x, y, z, rot = qtm_interface.latest_pose
            print(f"QTM tracking acquired! Position: x={x:.2f}, y={y:.2f}, z={z:.2f}")
            return True
        time.sleep(0.1)

    print("Timeout waiting for QTM tracking!")
    return False


def main():
    """Main flight sequence"""
    print("=== Crazyflie Figure-8 Flight with QTM ===")

    # Initialize QTM
    qtm = QTMInterface(QTM_IP, RIGID_BODY)
    qtm_thread = run_qtm_in_thread(qtm)

    try:
        # Wait for QTM tracking
        if not wait_for_qtm_tracking(qtm):
            return

        # Initialize Crazyflie controller
        controller = CrazyflieController(URI, qtm)
        controller.connect()

        # Flight sequence
        controller.takeoff(TAKEOFF_HEIGHT, TAKEOFF_TIME)
        controller.run_figure_8()
        controller.land(LAND_TIME)

        print("Flight completed successfully!")

    except KeyboardInterrupt:
        print("\nFlight interrupted by user")
    except Exception as e:
        print(f"Flight error: {e}")
    finally:
        # Cleanup
        if 'controller' in locals():
            controller.running = False
        qtm.stop()
        time.sleep(1)


if __name__ == "__main__":
    main()