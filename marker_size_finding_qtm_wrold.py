# # import asyncio, time, threading, json, os, sys
# # from collections import deque
# # from datetime import datetime, timezone, timedelta
# # import numpy as np
# #
# # import qtm_rt
# # import cflib.crtp
# # from cflib.crazyflie import Crazyflie
# # from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# # from cflib.positioning.motion_commander import MotionCommander
# #
# # # ---------------- USER SETTINGS ----------------
# # QTM_IP          = "192.168.0.105"   # "127.0.0.1" if same PC
# # URI             = "radio://0/80/2M"
# # STREAM_HZ       = 100
# # HOVER_HEIGHT_M  = 0.5
# # DO_HOVER_DEMO   = True
# #
# # # If you already know your mapping, set it here ("A","B","C","D") or leave None to use what’s in JSON:
# # FORCE_MAPPING   = "B"   # e.g. "A"  (None = use JSON or interactive)
# # CALFILE         = "qtm_to_cf_extpos_cal.json"
# # XY_LOCK_AT_TAKEOFF = True  # makes current XY at reset → (0,0)
# # # ------------------------------------------------
# #
# # def now_iso():
# #     # user is in Asia/Riyadh (UTC+3)
# #     return datetime.now(timezone(timedelta(hours=3))).isoformat()
# #
# # # ---------- QTM thread (6d) ----------
# # class QTMThread(threading.Thread):
# #     def __init__(self, ip, out_pose):
# #         super().__init__(daemon=True)
# #         self.ip = ip
# #         self.out = out_pose  # {"pos_mm": None, "ts": 0.0}
# #         self.stop_flag = threading.Event()
# #
# #     def run(self):
# #         asyncio.run(self._amain())
# #
# #     async def _amain(self):
# #         print("[QTM] event loop…")
# #         conn = None
# #         try:
# #             conn = await qtm_rt.connect(self.ip)
# #             if not conn:
# #                 print(f"[QTM] ERROR: cannot connect {self.ip}")
# #                 return
# #             await conn.stream_frames(components=["6d"], on_packet=self._on_packet)
# #             print("[QTM] streaming '6d'")
# #             while not self.stop_flag.is_set():
# #                 await asyncio.sleep(0.01)
# #         except Exception as e:
# #             print(f"[QTM] ERROR: {e}")
# #         finally:
# #             try:
# #                 if conn:
# #                     await conn.disconnect()
# #             except Exception:
# #                 pass
# #
# #     def _on_packet(self, packet):
# #         try:
# #             info, bodies = packet.get_6d()
# #             if not bodies: return
# #             pos_mm, _rotm = bodies[0]
# #             self.out["pos_mm"] = np.array(pos_mm, float)
# #             self.out["ts"] = time.time()
# #         except Exception as e:
# #             print(f"[QTM] parse error: {e}")
# #
# #     def stop(self): self.stop_flag.set()
# #
# # # ---------- Axis mapping ----------
# # def map_pos_mm_to_cf_m(pos_mm, origin_mm, mapping, xy_bias=np.zeros(3)):
# #     """
# #     Convert QTM position (mm) to CF ENU (m) with selected axis map.
# #     A (most common):
# #         CF.x = +QTM.y, CF.y = -QTM.x, CF.z = +QTM.z
# #     B (identity):
# #         CF.x = +QTM.x, CF.y = +QTM.y, CF.z = +QTM.z
# #     C:
# #         CF.x = +QTM.x, CF.y = -QTM.y, CF.z = +QTM.z
# #     D:
# #         CF.x = +QTM.y, CF.y = +QTM.x, CF.z = +QTM.z
# #     """
# #     dx = (pos_mm - origin_mm) * 0.001  # mm → m
# #
# #     if mapping == "A":
# #         out = np.array([ dx[1], -dx[0],  dx[2] ], float)
# #     elif mapping == "B":
# #         out = np.array([ dx[0],  dx[1],  dx[2] ], float)
# #     elif mapping == "C":
# #         out = np.array([ dx[0], -dx[1],  dx[2] ], float)
# #     elif mapping == "D":
# #         out = np.array([ dx[1],  dx[0],  dx[2] ], float)
# #     else:
# #         # default to A
# #         out = np.array([ dx[1], -dx[0], dx[2] ], float)
# #
# #     return out - xy_bias  # subtract bias (xy_lock), z-bias left untouched
# #
# # # ---------- Calibration persistence ----------
# # def save_cal(calfile, qtm_ip, origin_mm, mapping, scale_m_per_mm=0.001, xy_lock=True):
# #     data = {
# #         "qtm_ip": qtm_ip,
# #         "mapping": mapping,
# #         "origin_mm": [float(origin_mm[0]), float(origin_mm[1]), float(origin_mm[2])],
# #         "scale_m_per_mm": float(scale_m_per_mm),
# #         "xy_lock_at_takeoff": bool(xy_lock),
# #         "saved_at": now_iso(),
# #     }
# #     with open(calfile, "w", encoding="utf-8") as f:
# #         json.dump(data, f, indent=2)
# #     print(f"[CAL] saved → {calfile}")
# #
# # def load_cal(calfile):
# #     if not os.path.exists(calfile):
# #         return None
# #     with open(calfile, "r", encoding="utf-8") as f:
# #         data = json.load(f)
# #     print(f"[CAL] loaded ← {calfile}")
# #     return data
# #
# # def pick_mapping_interactive(shared, origin_mm):
# #     print("\n=== Axis-map quick test (2s per option) ===")
# #     print("Move the rig ~+0.20 m along your REAL FORWARD. Pick where:")
# #     print("  CF x increases ~+0.20 and CF y ≈ 0.")
# #
# #     options = ["A","B","C","D"]
# #     scores = {}
# #     for opt in options:
# #         print(f"\nOption {opt}: move +forward now for ~2 s…")
# #         time.sleep(0.5)
# #         xs, ys = [], []
# #         t_end = time.time() + 2.0
# #         while time.time() < t_end:
# #             pos_mm = shared["pos_mm"]
# #             if pos_mm is not None:
# #                 p_cf = map_pos_mm_to_cf_m(pos_mm, origin_mm, mapping=opt)
# #                 xs.append(p_cf[0]); ys.append(p_cf[1])
# #                 print(f"  CF pos ~ x={p_cf[0]: .3f}  y={p_cf[1]: .3f}  z={p_cf[2]: .3f}", end="\r")
# #             time.sleep(0.1)
# #         if xs:
# #             dx = xs[-1] - xs[0]
# #             dy = ys[-1] - ys[0]
# #             # Prefer large +dx and small |dy|
# #             scores[opt] = (dx, -abs(dy))
# #         else:
# #             scores[opt] = (-1e9, -1e9)
# #         print()
# #
# #     best = max(scores.items(), key=lambda kv: (kv[1][0], kv[1][1]))[0]
# #     print(f"Chosen mapping: {best}")
# #     return best
# #
# # def main():
# #     # 1) Start QTM
# #     shared = {"pos_mm": None, "ts": 0.0}
# #     qtm = QTMThread(QTM_IP, shared)
# #     qtm.start()
# #
# #     # Wait for first frame
# #     t0 = time.time()
# #     while shared["pos_mm"] is None and time.time() - t0 < 5.0:
# #         time.sleep(0.01)
# #     if shared["pos_mm"] is None:
# #         print("No QTM frames. Enable Real-Time Server + 6d for your trackable.")
# #         return
# #     print("First QTM frame (mm):", shared["pos_mm"])
# #
# #     # Load prior calibration (if any)
# #     cal = load_cal(CALFILE) or {}
# #     mapping = FORCE_MAPPING or cal.get("mapping")
# #     origin_mm = np.array(cal.get("origin_mm"), float) if "origin_mm" in cal else None
# #
# #     # 2) Lock origin if missing
# #     if origin_mm is None:
# #         print("\n=== Origin lock === Hold the rig still…")
# #         buf = deque(maxlen=30)
# #         while len(buf) < buf.maxlen:
# #             if shared["pos_mm"] is not None:
# #                 buf.append(shared["pos_mm"])
# #             time.sleep(0.01)
# #         origin_mm = np.mean(np.stack(buf, axis=0), axis=0)
# #         print("Origin (QTM units):", origin_mm)
# #
# #     # 3) Pick mapping if missing
# #     if mapping is None:
# #         mapping = pick_mapping_interactive(shared, origin_mm)
# #
# #     # Save/update calibration
# #     save_cal(CALFILE, QTM_IP, origin_mm, mapping, scale_m_per_mm=0.001, xy_lock=XY_LOCK_AT_TAKEOFF)
# #
# #     # 4) Connect Crazyflie
# #     cflib.crtp.init_drivers(enable_debug_driver=False)
# #     cf = Crazyflie(rw_cache='./cache')
# #
# #     with SyncCrazyflie(URI, cf=cf) as scf:
# #         print("Connected to Crazyflie.")
# #         # Estimator = Kalman (if present)
# #         try: cf.param.set_value('stabilizer.estimator', '2')
# #         except KeyError: pass
# #
# #         # Warm-up: send a few frames before reset
# #         print("Warming up extpos…")
# #         last_p = np.zeros(3)
# #         # XY bias lock (so current place becomes (0,0))
# #         xy_bias = np.zeros(3)
# #         warm_end = time.time() + 0.3
# #         while time.time() < warm_end:
# #             pos_mm = shared["pos_mm"]
# #             if pos_mm is not None:
# #                 p_cf = map_pos_mm_to_cf_m(pos_mm, origin_mm, mapping, xy_bias=np.zeros(3))
# #                 last_p = p_cf
# #                 cf.extpos.send_extpos(p_cf[0], p_cf[1], p_cf[2])
# #             time.sleep(1.0/STREAM_HZ)
# #
# #         # Perform XY lock at takeoff (optional but recommended)
# #         if XY_LOCK_AT_TAKEOFF:
# #             # compute current mapped XY, subtract it from all future sends
# #             xy_bias = np.array([last_p[0], last_p[1], 0.0], float)
# #
# #         # Reset estimation with current pose (after applying bias)
# #         try:
# #             x0, y0, z0 = (last_p[0]-xy_bias[0], last_p[1]-xy_bias[1], last_p[2])
# #             cf.param.set_value('kalman.initialX', f'{x0:.3f}')
# #             cf.param.set_value('kalman.initialY', f'{y0:.3f}')
# #             cf.param.set_value('kalman.initialZ', f'{z0:.3f}')
# #             cf.param.set_value('kalman.resetEstimation', '1')
# #             time.sleep(0.1)
# #             cf.param.set_value('kalman.resetEstimation', '0')
# #         except KeyError:
# #             pass
# #         print("Estimator reset.")
# #
# #         # Background streaming
# #         stop_evt = threading.Event()
# #         def loop():
# #             period = 1.0/STREAM_HZ
# #             next_t = time.time()
# #             while not stop_evt.is_set():
# #                 pos_mm = shared["pos_mm"]
# #                 if pos_mm is not None:
# #                     p_cf = map_pos_mm_to_cf_m(pos_mm, origin_mm, mapping, xy_bias=xy_bias)
# #                     cf.extpos.send_extpos(p_cf[0], p_cf[1], p_cf[2])
# #                 next_t += period
# #                 dt = next_t - time.time()
# #                 if dt > 0: time.sleep(dt)
# #                 else: next_t = time.time()
# #
# #         th = threading.Thread(target=loop, daemon=True)
# #         th.start()
# #         print(f"Streaming extpos to CF (map={mapping}, xy_lock={XY_LOCK_AT_TAKEOFF})…")
# #
# #         try:
# #             if DO_HOVER_DEMO:
# #                 try: cf.param.set_value('commander.enHighLevel', '1')
# #                 except KeyError: pass
# #                 print(f"Takeoff to {HOVER_HEIGHT_M:.2f} m, hold 12 s…")
# #                 with MotionCommander(scf, default_height=HOVER_HEIGHT_M) as mc:
# #                     time.sleep(12.0)
# #                     print("Landing…")
# #                     mc.land()
# #                     time.sleep(2.0)
# #             else:
# #                 print("Press Ctrl+C to stop.")
# #                 while True:
# #                     time.sleep(0.5)
# #         except KeyboardInterrupt:
# #             print("Interrupted.")
# #         finally:
# #             stop_evt.set(); th.join(timeout=1.0)
# #             print("Stopped streaming.")
# #
# #     qtm.stop(); qtm.join(timeout=1.0)
# #     print("Exit.")
# #
# # if __name__ == "__main__":
# #     main()
#
#
#
# """
# Run Crazyflie with external position from QTM using saved calibration (JSON).
# - Position-only (extpos) for stability.
# - Uses saved origin, mapping (A/B/C/D), scale, and XY-lock flag from JSON.
# """
#
# import asyncio, time, threading, json, os
# from collections import deque
# import numpy as np
#
# import qtm_rt
# import cflib.crtp
# from cflib.crazyflie import Crazyflie
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.positioning.motion_commander import MotionCommander
#
# # ------ user toggles ------
# CALFILE         = "qtm_to_cf_extpos_cal.json"  # your saved file
# URI             = "radio://0/80/2M"            # radio uri
# STREAM_HZ       = 100
# HOVER_HEIGHT_M  = 0.50
# DO_HOVER_DEMO   = True                         # set False to just stream
# # --------------------------
#
# def load_cal(path):
#     if not os.path.exists(path):
#         raise FileNotFoundError(f"Calibration file not found: {path}")
#     with open(path, "r", encoding="utf-8") as f:
#         cal = json.load(f)
#     # basic validation
#     for k in ["qtm_ip","mapping","origin_mm","scale_m_per_mm","xy_lock_at_takeoff"]:
#         if k not in cal:
#             raise ValueError(f"Missing '{k}' in {path}")
#     return cal
#
# # QTM thread: read 6d (position, rotation matrix not used here)
# class QTMThread(threading.Thread):
#     def __init__(self, ip, out_pose):
#         super().__init__(daemon=True)
#         self.ip = ip
#         self.out = out_pose  # {"pos_mm": None, "ts": 0.0}
#         self.stop_flag = threading.Event()
#
#     def run(self):
#         asyncio.run(self._amain())
#
#     async def _amain(self):
#         print("[QTM] event loop…")
#         conn = None
#         try:
#             conn = await qtm_rt.connect(self.ip)
#             if not conn:
#                 print(f"[QTM] ERROR: cannot connect {self.ip}")
#                 return
#             await conn.stream_frames(components=["6d"], on_packet=self._on_packet)
#             print("[QTM] streaming '6d'")
#             while not self.stop_flag.is_set():
#                 await asyncio.sleep(0.01)
#         except Exception as e:
#             print(f"[QTM] ERROR: {e}")
#         finally:
#             try:
#                 if conn:
#                     await conn.disconnect()
#             except Exception:
#                 pass
#
#     def _on_packet(self, packet):
#         try:
#             info, bodies = packet.get_6d()
#             if not bodies: return
#             pos_mm, _rotm = bodies[0]
#             self.out["pos_mm"] = np.array(pos_mm, float)
#             self.out["ts"] = time.time()
#         except Exception as e:
#             print(f"[QTM] parse error: {e}")
#
#     def stop(self): self.stop_flag.set()
#
# # axis mapping helpers
# def map_pos_to_cf(pos_mm, origin_mm, mapping, scale, xy_bias=np.zeros(3)):
#     d = (pos_mm - origin_mm) * float(scale)  # -> meters
#     if mapping == "A":   out = np.array([ d[1], -d[0],  d[2] ], float)
#     elif mapping == "B": out = np.array([ d[0],  d[1],  d[2] ], float)
#     elif mapping == "C": out = np.array([ d[0], -d[1],  d[2] ], float)
#     elif mapping == "D": out = np.array([ d[1],  d[0],  d[2] ], float)
#     else:                out = np.array([ d[1], -d[0],  d[2] ], float)  # default A
#     return out - xy_bias  # keep z as-is
#
# def main():
#     # 1) Load calibration
#     cal = load_cal(CALFILE)
#     QTM_IP = cal["qtm_ip"]
#     MAPPING = cal["mapping"]
#     ORIGIN_MM = np.array(cal["origin_mm"], float)
#     SCALE = float(cal["scale_m_per_mm"])  # 0.001 for mm->m
#     XY_LOCK = bool(cal["xy_lock_at_takeoff"])
#
#     print(f"[CAL] {CALFILE} loaded")
#     print(f"      qtm_ip={QTM_IP}  mapping={MAPPING}  scale={SCALE}  xy_lock={XY_LOCK}")
#     print(f"      origin_mm={ORIGIN_MM}")
#
#     # 2) Start QTM
#     shared = {"pos_mm": None, "ts": 0.0}
#     qtm = QTMThread(QTM_IP, shared)
#     qtm.start()
#
#     t0 = time.time()
#     while shared["pos_mm"] is None and time.time() - t0 < 5.0:
#         time.sleep(0.01)
#     if shared["pos_mm"] is None:
#         print("No QTM frames. Enable Real-Time Server + 6d for your trackable.")
#         return
#     print("First QTM frame (mm):", shared["pos_mm"])
#
#     # 3) Connect CF
#     cflib.crtp.init_drivers(enable_debug_driver=False)
#     cf = Crazyflie(rw_cache='./cache')
#     with SyncCrazyflie(URI, cf=cf) as scf:
#         print("Connected to Crazyflie.")
#         # Set Kalman if param exists
#         try: cf.param.set_value('stabilizer.estimator', '2')
#         except KeyError: pass
#
#         # Warm-up: stream a few frames to seed last_p
#         print("Warming up extpos…")
#         last_p = np.zeros(3)
#         warm_end = time.time() + 0.3
#         while time.time() < warm_end:
#             pos_mm = shared["pos_mm"]
#             if pos_mm is not None:
#                 last_p = map_pos_to_cf(pos_mm, ORIGIN_MM, MAPPING, SCALE)
#                 cf.extpos.send_extpos(last_p[0], last_p[1], last_p[2])
#             time.sleep(1.0/STREAM_HZ)
#
#         # XY-lock: treat current XY as (0,0)
#         xy_bias = np.array([last_p[0], last_p[1], 0.0], float) if XY_LOCK else np.zeros(3)
#
#         # Reset Kalman near (after-bias) pose
#         try:
#             x0, y0, z0 = (last_p[0]-xy_bias[0], last_p[1]-xy_bias[1], last_p[2])
#             cf.param.set_value('kalman.initialX', f'{x0:.3f}')
#             cf.param.set_value('kalman.initialY', f'{y0:.3f}')
#             cf.param.set_value('kalman.initialZ', f'{z0:.3f}')
#             cf.param.set_value('kalman.resetEstimation', '1')
#             time.sleep(0.1)
#             cf.param.set_value('kalman.resetEstimation', '0')
#         except KeyError:
#             pass
#         print("Estimator reset.")
#
#         # Background streaming loop
#         stop_evt = threading.Event()
#         def loop():
#             period = 1.0/STREAM_HZ
#             next_t = time.time()
#             while not stop_evt.is_set():
#                 pm = shared["pos_mm"]
#                 if pm is not None:
#                     p = map_pos_to_cf(pm, ORIGIN_MM, MAPPING, SCALE, xy_bias=xy_bias)
#                     cf.extpos.send_extpos(p[0], p[1], p[2])
#                 next_t += period
#                 dt = next_t - time.time()
#                 if dt > 0: time.sleep(dt)
#                 else: next_t = time.time()
#
#         th = threading.Thread(target=loop, daemon=True)
#         th.start()
#         print(f"Streaming extpos (mapping={MAPPING}, xy_lock={XY_LOCK})…")
#
#         try:
#             if DO_HOVER_DEMO:
#                 try: cf.param.set_value('commander.enHighLevel', '1')
#                 except KeyError: pass
#                 print(f"Takeoff to {HOVER_HEIGHT_M:.2f} m, hold 12 s…")
#                 with MotionCommander(scf, default_height=HOVER_HEIGHT_M) as mc:
#                     time.sleep(12.0)
#                     print("Landing…")
#                     mc.land()
#                     time.sleep(2.0)
#             else:
#                 print("Press Ctrl+C to stop.")
#                 while True:
#                     time.sleep(0.5)
#         except KeyboardInterrupt:
#             print("Interrupted.")
#         finally:
#             stop_evt.set(); th.join(timeout=1.0)
#             print("Stopped streaming.")
#
#     qtm.stop(); qtm.join(timeout=1.0)
#     print("Exit.")
#
# if __name__ == "__main__":
#     main()

# """
# Crazyflie ← QTM external POSITION (mapping = B only, no JSON)
# - QTM '6d' streaming, mm→m
# - Fixed axis map B (identity): CF(x,y,z) = QTM(x,y,z)
# - XY-lock at takeoff: wherever the rig is when we reset = (0,0)
# - EKF = Crazyflie Kalman (stabilizer.estimator = 2)
# - Position-only (extpos) for stability
# """
#
# import asyncio
# import time
# import threading
# from collections import deque
# import numpy as np
#
# import qtm_rt
# import cflib.crtp
# from cflib.crazyflie import Crazyflie
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.positioning.motion_commander import MotionCommander
#
# # ---------- USER CONFIG ----------
# QTM_IP = "192.168.0.105"     # "127.0.0.1" if the script runs on the QTM PC
# URI    = "radio://0/80/2M"   # set to your dongle/channel
#
# STREAM_HZ       = 100        # extpos send rate
# HOVER_HEIGHT_M  = 0.50       # takeoff height
# HOVER_SECONDS   = 12.0       # hold duration
# DO_HOVER_DEMO   = True       # False = only stream extpos (no takeoff/land)
# XY_LOCK_AT_TAKEOFF = True    # True = make current XY -> (0,0) at reset
# LPF_ALPHA       = 0.2        # 0 (no smoothing) … 1 (heavy smoothing) on position
# # ---------------------------------
#
#
# # ========== QTM client thread (6d) ==========
# class QTMThread(threading.Thread):
#     def __init__(self, ip, out_pose):
#         super().__init__(daemon=True)
#         self.ip = ip
#         self.out = out_pose  # {"pos_mm": None, "ts": 0.0}
#         self.stop_flag = threading.Event()
#
#     def run(self):
#         asyncio.run(self._amain())
#
#     async def _amain(self):
#         print("[QTM] event loop…")
#         conn = None
#         try:
#             conn = await qtm_rt.connect(self.ip)
#             if not conn:
#                 print(f"[QTM] ERROR: cannot connect to {self.ip}")
#                 return
#             await conn.stream_frames(components=["6d"], on_packet=self._on_packet)
#             print("[QTM] streaming '6d'")
#             while not self.stop_flag.is_set():
#                 await asyncio.sleep(0.01)
#         except Exception as e:
#             print(f"[QTM] ERROR: {e}")
#         finally:
#             try:
#                 if conn:
#                     await conn.disconnect()
#             except Exception:
#                 pass
#
#     def _on_packet(self, packet):
#         try:
#             info, bodies = packet.get_6d()
#             if not bodies:
#                 return
#             pos_mm, _rotm = bodies[0]  # rotation not used (position-only)
#             self.out["pos_mm"] = np.array(pos_mm, dtype=float)
#             self.out["ts"] = time.time()
#         except Exception as e:
#             print(f"[QTM] parse error: {e}")
#
#     def stop(self):
#         self.stop_flag.set()
#
#
# # ========== mapping B only + simple smoothing ==========
# def map_pos_mm_to_cf_m_mappingB(pos_mm, origin_mm, scale=0.001, xy_bias=np.zeros(3)):
#     """
#     mapping B (identity): CF.x = QTM.x, CF.y = QTM.y, CF.z = QTM.z
#     origin_mm is subtracted, then converted mm→m (scale=0.001).
#     xy_bias is subtracted from (x,y,0) AFTER mapping (for XY-lock).
#     """
#     d = (pos_mm - origin_mm) * float(scale)
#     out = np.array([d[0], d[1], d[2]], dtype=float)
#     return out - xy_bias  # keep z unchanged in bias (bias.z=0)
#
#
# class LPF3:
#     """First-order low-pass filter on a 3-vector (to calm tiny jitter)."""
#     def __init__(self, alpha=0.2):
#         self.alpha = float(alpha)
#         self.y = None
#
#     def reset(self):
#         self.y = None
#
#     def __call__(self, x):
#         x = np.asarray(x, dtype=float)
#         if self.y is None:
#             self.y = x.copy()
#         else:
#             self.y = (1 - self.alpha) * self.y + self.alpha * x
#         return self.y
#
#
# # ========== helpers ==========
# def try_set_param(cf, name, value):
#     """Best-effort set param; skip if not present in TOC."""
#     try:
#         cf.param.set_value(name, value)
#         return True
#     except KeyError:
#         # print(f"[CF] Param '{name}' not found; skipping.")
#         return False
#
#
# # ========== main ==========
# def main():
#     # 1) Start QTM thread, wait for frames
#     shared = {"pos_mm": None, "ts": 0.0}
#     qtm = QTMThread(QTM_IP, shared)
#     qtm.start()
#
#     t0 = time.time()
#     while shared["pos_mm"] is None and time.time() - t0 < 5.0:
#         time.sleep(0.01)
#     if shared["pos_mm"] is None:
#         print("No QTM frames. Enable Real-Time Server + 6d for your trackable.")
#         return
#
#     print("First QTM frame (mm):", shared["pos_mm"])
#
#     # 2) Lock origin (average while rig is still)
#     print("\n=== Origin lock === Hold the rig still…")
#     buf = deque(maxlen=30)
#     while len(buf) < buf.maxlen:
#         if shared["pos_mm"] is not None:
#             buf.append(shared["pos_mm"])
#         time.sleep(0.01)
#     origin_mm = np.mean(np.stack(buf, axis=0), axis=0)
#     print("Origin (QTM units):", origin_mm)
#
#     # 3) Connect Crazyflie
#     cflib.crtp.init_drivers(enable_debug_driver=False)
#     cf = Crazyflie(rw_cache="./cache")
#     with SyncCrazyflie(URI, cf=cf) as scf:
#         print("Connected to Crazyflie.")
#
#         # Ensure EKF (Kalman) is active if param exists
#         try_set_param(cf, "stabilizer.estimator", "2")  # 2 = Kalman
#
#         # Optional: if your firmware has these Kalman params, they’ll apply; otherwise silently skip.
#         # Lower process noise a bit (nudges EKF to trust extpos more).
#         try_set_param(cf, "kalman.resetEstimation", "1"); time.sleep(0.05); try_set_param(cf, "kalman.resetEstimation", "0")
#
#         # 4) Warm-up: send a few extpos frames before reset
#         print("Warming up extpos…")
#         last_p = np.zeros(3)
#         lpf = LPF3(alpha=LPF_ALPHA)
#         warm_end = time.time() + 0.4
#         while time.time() < warm_end:
#             pm = shared["pos_mm"]
#             if pm is not None:
#                 p = map_pos_mm_to_cf_m_mappingB(pm, origin_mm, scale=0.001, xy_bias=np.zeros(3))
#                 p = lpf(p)
#                 last_p = p
#                 cf.extpos.send_extpos(p[0], p[1], p[2])
#             time.sleep(1.0 / STREAM_HZ)
#
#         # 5) XY-lock at takeoff (strict hold around current place)
#         xy_bias = np.array([last_p[0], last_p[1], 0.0], dtype=float) if XY_LOCK_AT_TAKEOFF else np.zeros(3)
#
#         # 6) Reset EKF around (after-bias) current pose (x,y)≈(0,0)
#         x0, y0, z0 = (last_p[0] - xy_bias[0], last_p[1] - xy_bias[1], last_p[2])
#         try_set_param(cf, "kalman.initialX", f"{x0:.3f}")
#         try_set_param(cf, "kalman.initialY", f"{y0:.3f}")
#         try_set_param(cf, "kalman.initialZ", f"{z0:.3f}")
#         try_set_param(cf, "kalman.resetEstimation", "1"); time.sleep(0.12); try_set_param(cf, "kalman.resetEstimation", "0")
#         print("Estimator reset.")
#
#         # 7) Background streaming loop (strict rate)
#         stop_evt = threading.Event()
#
#         def stream_loop():
#             period = 1.0 / STREAM_HZ
#             next_t = time.time()
#             while not stop_evt.is_set():
#                 pm = shared["pos_mm"]
#                 if pm is not None:
#                     p = map_pos_mm_to_cf_m_mappingB(pm, origin_mm, scale=0.001, xy_bias=xy_bias)
#                     p = lpf(p)
#                     cf.extpos.send_extpos(p[0], p[1], p[2])
#                 next_t += period
#                 dt = next_t - time.time()
#                 if dt > 0:
#                     time.sleep(dt)
#                 else:
#                     next_t = time.time()
#
#         th = threading.Thread(target=stream_loop, daemon=True)
#         th.start()
#         print(f"Streaming extpos (mapping=B, xy_lock={XY_LOCK_AT_TAKEOFF})…")
#
#         # 8) Optional hover demo (uses EKF for hold)
#         try:
#             if DO_HOVER_DEMO:
#                 try_set_param(cf, "commander.enHighLevel", "1")
#                 print(f"Takeoff to {HOVER_HEIGHT_M:.2f} m, hold {HOVER_SECONDS:.0f} s…")
#                 with MotionCommander(scf, default_height=HOVER_HEIGHT_M) as mc:
#                     time.sleep(HOVER_SECONDS)
#                     print("Landing…")
#                     mc.land()
#                     time.sleep(2.0)
#             else:
#                 print("Press Ctrl+C to stop.")
#                 while True:
#                     time.sleep(0.5)
#         except KeyboardInterrupt:
#             print("Interrupted.")
#         finally:
#             stop_evt.set()
#             th.join(timeout=1.0)
#             print("Stopped streaming.")
#
#     # 9) Cleanup QTM
#     qtm.stop()
#     qtm.join(timeout=1.0)
#     print("Exit.")
#
#
# if __name__ == "__main__":
#     main()



# """
# Crazyflie ← QTM external POSITION (fixed config: mapping=B, origin/scale baked-in)
# - QTM '6d' streaming, mm→m
# - Fixed axis map B (identity): CF(x,y,z) = QTM(x,y,z)
# - Fixed origin_mm and scale from your known calibration
# - XY-lock at takeoff so current place becomes (0,0)
# - EKF = Crazyflie Kalman (stabilizer.estimator = 2)
# - Position-only (extpos) for stability
# """
#
# import asyncio
# import time
# import threading
# import numpy as np
#
# import qtm_rt
# import cflib.crtp
# from cflib.crazyflie import Crazyflie
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.positioning.motion_commander import MotionCommander
#
# # -------- FIXED CONFIG FROM YOUR SETUP --------
# QTM_IP = "192.168.0.105"        # QTM real-time server
# URI    = "radio://0/90/2M"      # Crazyflie radio URI
#
# # Mapping "B" (identity): CF = QTM (after origin/scale)
# MAPPING_B = True                # (kept for clarity)
#
# # Origin in QTM units (mm) from your JSON
# ORIGIN_MM = np.array([
#     14131.634244791667,
#     423.228471883138,
#     -7.1656776110331215
# ], dtype=float)
#
# SCALE_M_PER_MM   = 0.001        # QTM mm -> meters
# STREAM_HZ        = 100          # extpos send rate
# HOVER_HEIGHT_M   = 0.50         # takeoff height
# HOVER_SECONDS    = 12.0         # hold duration
# DO_HOVER_DEMO    = True         # False = only stream extpos
# XY_LOCK_AT_TAKEOFF = True       # make current XY -> (0,0) at reset
# # ------------------------------------------------
#
#
# # ---------- QTM thread (6d) ----------
# class QTMThread(threading.Thread):
#     def __init__(self, ip, out_pose):
#         super().__init__(daemon=True)
#         self.ip = ip
#         self.out = out_pose  # {"pos_mm": None, "ts": 0.0}
#         self.stop_flag = threading.Event()
#
#     def run(self):
#         asyncio.run(self._amain())
#
#     async def _amain(self):
#         print("[QTM] event loop…")
#         conn = None
#         try:
#             conn = await qtm_rt.connect(self.ip)
#             if not conn:
#                 print(f"[QTM] ERROR: cannot connect {self.ip}")
#                 return
#             await conn.stream_frames(components=["6d"], on_packet=self._on_packet)
#             print("[QTM] streaming '6d'")
#             while not self.stop_flag.is_set():
#                 await asyncio.sleep(0.01)
#         except Exception as e:
#             print(f"[QTM] ERROR: {e}")
#         finally:
#             try:
#                 if conn:
#                     await conn.disconnect()
#             except Exception:
#                 pass
#
#     def _on_packet(self, packet):
#         try:
#             info, bodies = packet.get_6d()
#             if not bodies:
#                 return
#             pos_mm, _rotm = bodies[0]  # rotation not used (position-only)
#             self.out["pos_mm"] = np.array(pos_mm, dtype=float)
#             self.out["ts"] = time.time()
#         except Exception as e:
#             print(f"[QTM] parse error: {e}")
#
#     def stop(self):
#         self.stop_flag.set()
#
#
# # ---------- Mapping B (identity) + helper ----------
# def map_pos_mm_to_cf_m_mappingB(pos_mm, origin_mm, scale=SCALE_M_PER_MM, xy_bias=np.zeros(3)):
#     """
#     mapping B (identity): CF.x = QTM.x, CF.y = QTM.y, CF.z = QTM.z
#     Subtract origin, convert mm->m, then subtract XY-bias (for XY-lock).
#     """
#     d = (pos_mm - origin_mm) * float(scale)  # -> meters
#     out = np.array([d[0], d[1], d[2]], dtype=float)
#     return out - xy_bias  # xy_bias = [x_bias, y_bias, 0]
#
#
# def try_set_param(cf, name, value):
#     """Best-effort set param; skip if not present in TOC."""
#     try:
#         cf.param.set_value(name, value)
#         return True
#     except KeyError:
#         return False
#
#
# def main():
#     # 1) Start QTM thread and wait for frames
#     shared = {"pos_mm": None, "ts": 0.0}
#     qtm = QTMThread(QTM_IP, shared)
#     qtm.start()
#
#     t0 = time.time()
#     while shared["pos_mm"] is None and time.time() - t0 < 5.0:
#         time.sleep(0.01)
#     if shared["pos_mm"] is None:
#         print("No QTM frames. Enable Real-Time Server + 6d for your trackable.")
#         return
#
#     print("First QTM frame (mm):", shared["pos_mm"])
#     print("Using fixed origin_mm (mm):", ORIGIN_MM)
#     print("Mapping: B (identity), Scale:", SCALE_M_PER_MM)
#
#     # 2) Connect Crazyflie
#     cflib.crtp.init_drivers(enable_debug_driver=False)
#     cf = Crazyflie(rw_cache="./cache")
#     with SyncCrazyflie(URI, cf=cf) as scf:
#         print("Connected to Crazyflie.")
#
#         # Ensure EKF (Kalman) if available
#         try_set_param(cf, "stabilizer.estimator", "2")  # 2 = Kalman
#
#         # 3) Warm-up: send a few extpos frames before reset
#         print("Warming up extpos…")
#         last_p = np.zeros(3)
#         warm_end = time.time() + 0.4
#         while time.time() < warm_end:
#             pm = shared["pos_mm"]
#             if pm is not None:
#                 last_p = map_pos_mm_to_cf_m_mappingB(pm, ORIGIN_MM, scale=SCALE_M_PER_MM, xy_bias=np.zeros(3))
#                 cf.extpos.send_extpos(last_p[0], last_p[1], last_p[2])
#             time.sleep(1.0 / STREAM_HZ)
#
#         # 4) XY-lock at takeoff: make current XY be (0,0)
#         xy_bias = np.array([last_p[0], last_p[1], 0.0], dtype=float) if XY_LOCK_AT_TAKEOFF else np.zeros(3)
#
#         # 5) Reset EKF around (after-bias) current pose
#         x0, y0, z0 = (last_p[0] - xy_bias[0], last_p[1] - xy_bias[1], last_p[2])
#         try_set_param(cf, "kalman.initialX", f"{x0:.3f}")
#         try_set_param(cf, "kalman.initialY", f"{y0:.3f}")
#         try_set_param(cf, "kalman.initialZ", f"{z0:.3f}")
#         try_set_param(cf, "kalman.resetEstimation", "1")
#         time.sleep(0.12)
#         try_set_param(cf, "kalman.resetEstimation", "0")
#         print("Estimator reset.")
#
#         # 6) Background streaming loop at fixed rate
#         stop_evt = threading.Event()
#         def stream_loop():
#             period = 1.0 / STREAM_HZ
#             next_t = time.time()
#             dbg_t = 0.0
#             while not stop_evt.is_set():
#                 pm = shared["pos_mm"]
#                 if pm is not None:
#                     p = map_pos_mm_to_cf_m_mappingB(pm, ORIGIN_MM, scale=SCALE_M_PER_MM, xy_bias=xy_bias)
#                     cf.extpos.send_extpos(p[0], p[1], p[2])
#                     # light debug once per second
#                     if time.time() - dbg_t > 1.0:
#                         print(f"[extpos] x={p[0]:+.3f}  y={p[1]:+.3f}  z={p[2]:+.3f}")
#                         dbg_t = time.time()
#                 next_t += period
#                 dt = next_t - time.time()
#                 if dt > 0:
#                     time.sleep(dt)
#                 else:
#                     next_t = time.time()
#
#         th = threading.Thread(target=stream_loop, daemon=True)
#         th.start()
#         print(f"Streaming extpos (mapping=B, xy_lock={XY_LOCK_AT_TAKEOFF})…")
#
#         # 7) Optional hover demo
#         try:
#             if DO_HOVER_DEMO:
#                 try_set_param(cf, "commander.enHighLevel", "1")
#                 print(f"Takeoff to {HOVER_HEIGHT_M:.2f} m, hold {HOVER_SECONDS:.0f} s…")
#                 with MotionCommander(scf, default_height=HOVER_HEIGHT_M) as mc:
#                     time.sleep(HOVER_SECONDS)
#                     print("Landing…")
#                     mc.land()
#                     time.sleep(2.0)
#             else:
#                 print("Press Ctrl+C to stop.")
#                 while True:
#                     time.sleep(0.5)
#         except KeyboardInterrupt:
#             print("Interrupted.")
#         finally:
#             stop_evt.set()
#             th.join(timeout=1.0)
#             print("Stopped streaming.")
#
#     # 8) Cleanup QTM
#     qtm.stop()
#     qtm.join(timeout=1.0)
#     print("Exit.")
#
#
# if __name__ == "__main__":
#     main()


# -*- coding: utf-8 -*-
"""
Crazyflie ← QTM external POSITION (hold, move +X, return, land)

What it does:
- Connects to QTM (component: '6d'), reads position (mm), converts to meters
- Streams extpos at 100 Hz (position-only) with a fixed QTM→CF mapping (identity)
- XY-locks at takeoff (current XY becomes (0,0)) and resets EKF
- Takeoff → hold for a few seconds → move +X by distance → return → land
"""

import asyncio
import time
import threading
import numpy as np

import qtm_rt
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

# ---------------- USER CONFIG ----------------
QTM_IP = "192.168.0.105"       # QTM real-time server
URI    = "radio://0/90/2M"     # Crazyflie radio URI

# QTM → CF mapping: identity (after origin removal and mm→m)
ORIGIN_MM = np.array([14131.634244791667, 423.228471883138, -7.1656776110331215], dtype=float)
SCALE_M_PER_MM = 0.001

STREAM_HZ = 100                # extpos rate
XY_LOCK_AT_TAKEOFF = True

# Flight plan
HOVER_HEIGHT_M         = 1  # takeoff height
HOLD_BEFORE_MOVE_S     = 10.0   # hold time before moving
MOVE_DISTANCE_X_M      = 2.00  # forward distance (+X)
MOVE_VELOCITY_MPS      = 0.20  # MotionCommander velocity
HOLD_AFTER_MOVE_S      = 4.0   # brief hold at the far point
HOLD_AFTER_RETURN_S    = 4.0   # brief hold back at start

# Freshness guard for mocap (seconds)
POSE_STALE_S = 0.2
# --------------------------------------------


# ------------ QTM Thread (6d) ---------------
class QTMThread(threading.Thread):
    def __init__(self, ip, out_pose):
        super().__init__(daemon=True)
        self.ip = ip
        self.out = out_pose  # {"pos_mm": None, "ts": 0.0}
        self.stop_flag = threading.Event()

    def run(self):
        asyncio.run(self._amain())

    async def _amain(self):
        print("[QTM] connecting…")
        conn = None
        try:
            conn = await qtm_rt.connect(self.ip)
            if not conn:
                print(f"[QTM] ERROR: cannot connect to {self.ip}")
                return
            await conn.stream_frames(components=["6d"], on_packet=self._on_packet)
            print("[QTM] streaming '6d'")
            while not self.stop_flag.is_set():
                await asyncio.sleep(0.01)
        except Exception as e:
            print(f"[QTM] ERROR: {e}")
        finally:
            try:
                if conn:
                    await conn.disconnect()
            except Exception:
                pass

    def _on_packet(self, packet):
        try:
            _info, bodies = packet.get_6d()
            if not bodies:
                return
            pos_mm, _rotm_flat = bodies[0]  # rotation unused (position-only)
            self.out["pos_mm"] = np.array(pos_mm, dtype=float)
            self.out["ts"] = time.time()
        except Exception as e:
            print(f"[QTM] parse error: {e}")

    def stop(self):
        self.stop_flag.set()


# ---------- Mapping helper (identity) ----------
def map_pos_mm_to_cf_m(pos_mm, origin_mm, scale=SCALE_M_PER_MM, xy_bias=np.zeros(3)):
    """
    CF = (QTM_pos_mm - origin_mm) * 0.001  minus XY bias (for XY-lock).
    Swap/negate axes here if your lab/world frames differ from Crazyflie.
    Crazyflie convention: X forward, Y left, Z up.
    """
    d = (pos_mm - origin_mm) * float(scale)  # meters
    out = np.array([d[0], d[1], d[2]], dtype=float)
    return out - xy_bias  # xy_bias = [x_bias, y_bias, 0]


def try_set_param(cf, name, value):
    try:
        cf.param.set_value(name, value)
        return True
    except KeyError:
        return False


def main():
    # Start QTM
    shared = {"pos_mm": None, "ts": 0.0}
    qtm = QTMThread(QTM_IP, shared)
    qtm.start()

    # Wait for first frame
    t0 = time.time()
    while shared["pos_mm"] is None and time.time() - t0 < 5.0:
        time.sleep(0.01)
    if shared["pos_mm"] is None:
        print("No QTM frames. Enable Real-Time Server + 6d for your trackable.")
        return

    print("First QTM frame (mm):", shared["pos_mm"])
    print("Using origin_mm (mm):", ORIGIN_MM)
    print("Mapping: identity, scale:", SCALE_M_PER_MM)

    # Connect Crazyflie
    cflib.crtp.init_drivers(enable_debug_driver=False)
    cf = Crazyflie(rw_cache="./cache")
    with SyncCrazyflie(URI, cf=cf) as scf:
        print("Connected to Crazyflie.")

        # Use Kalman (external position supported)
        try_set_param(cf, "stabilizer.estimator", "2")
        try_set_param(cf, "commander.enHighLevel", "1")

        # Warm-up: stream some extpos before EKF reset
        print("Warming up extpos…")
        last_p = np.zeros(3)
        warm_end = time.time() + 0.5
        while time.time() < warm_end:
            pm = shared["pos_mm"]
            if pm is not None:
                last_p = map_pos_mm_to_cf_m(pm, ORIGIN_MM, scale=SCALE_M_PER_MM, xy_bias=np.zeros(3))
                cf.extpos.send_extpos(last_p[0], last_p[1], last_p[2])
            time.sleep(1.0 / STREAM_HZ)

        # XY-lock at takeoff: current XY becomes (0,0)
        xy_bias = np.array([last_p[0], last_p[1], 0.0], dtype=float) if XY_LOCK_AT_TAKEOFF else np.zeros(3)

        # EKF reset around current pose (after XY bias)
        x0, y0, z0 = (last_p[0] - xy_bias[0], last_p[1] - xy_bias[1], last_p[2])
        try_set_param(cf, "kalman.initialX", f"{x0:.3f}")
        try_set_param(cf, "kalman.initialY", f"{y0:.3f}")
        try_set_param(cf, "kalman.initialZ", f"{z0:.3f}")
        try_set_param(cf, "kalman.resetEstimation", "1")
        time.sleep(0.12)
        try_set_param(cf, "kalman.resetEstimation", "0")
        print(f"Estimator reset at ({x0:.3f},{y0:.3f},{z0:.3f}).")

        # Background extpos streamer
        stop_evt = threading.Event()

        def stream_loop():
            period = 1.0 / STREAM_HZ
            next_t = time.time()
            dbg_t = 0.0
            while not stop_evt.is_set():
                pm, ts = shared["pos_mm"], shared["ts"]
                if pm is not None:
                    if time.time() - ts <= POSE_STALE_S:
                        p = map_pos_mm_to_cf_m(pm, ORIGIN_MM, scale=SCALE_M_PER_MM, xy_bias=xy_bias)
                        cf.extpos.send_extpos(p[0], p[1], p[2])
                        if time.time() - dbg_t > 1.0:
                            print(f"[extpos] x={p[0]:+.3f}  y={p[1]:+.3f}  z={p[2]:+.3f}")
                            dbg_t = time.time()
                    else:
                        # If mocap stalls, keep CF alive but don't inject stale pose
                        pass
                next_t += period
                dt = next_t - time.time()
                if dt > 0:
                    time.sleep(dt)
                else:
                    next_t = time.time()

        th = threading.Thread(target=stream_loop, daemon=True)
        th.start()
        print(f"Streaming extpos (xy_lock={XY_LOCK_AT_TAKEOFF})…")

        # Flight plan: takeoff → hold → +X move → hold → return → hold → land
        try:
            with MotionCommander(scf, default_height=HOVER_HEIGHT_M) as mc:
                print(f"Takeoff to {HOVER_HEIGHT_M:.2f} m, holding {HOLD_BEFORE_MOVE_S:.1f} s…")
                time.sleep(HOLD_BEFORE_MOVE_S)

                print(f"Move +X by {MOVE_DISTANCE_X_M:.2f} m @ {MOVE_VELOCITY_MPS:.2f} m/s")
                mc.forward(MOVE_DISTANCE_X_M, velocity=MOVE_VELOCITY_MPS)
                time.sleep(HOLD_AFTER_MOVE_S)

                print(f"Return -X by {MOVE_DISTANCE_X_M:.2f} m @ {MOVE_VELOCITY_MPS:.2f} m/s")
                mc.back(MOVE_DISTANCE_X_M, velocity=MOVE_VELOCITY_MPS)
                time.sleep(HOLD_AFTER_RETURN_S)

                print("Landing…")
                mc.land()
                time.sleep(2.0)

        except KeyboardInterrupt:
            print("Interrupted by user, landing…")
            try:
                # Best-effort soft stop
                scf.cf.commander.send_stop_setpoint()
            except Exception:
                pass
        finally:
            stop_evt.set()
            th.join(timeout=1.0)
            print("Stopped streaming.")

    # Cleanup QTM
    qtm.stop()
    qtm.join(timeout=1.0)
    print("Exit.")


if __name__ == "__main__":
    main()
