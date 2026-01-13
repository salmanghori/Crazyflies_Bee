# # """
# # Crazyflie ← QTM external POSE with:
# # - Auto frame alignment (try common B rotations; pick best by quaternion error at rest)
# # - Proper XY-lock and EKF reset *after* choosing best_B (critical fix)
# # - Live verification (mean angle error between sent and stateEstimate quaternion)
# # - Safe flight gating; positional fallback if orientation looks bad
# #
# # Assumes:
# # - QTM streams '6d' (position mm + 3x3 rotation)
# # - ORIGIN_MM is your QTM world origin for (0,0,0)
# # """
# #
# # import asyncio, time, threading
# # import numpy as np
# #
# # import qtm_rt
# # import cflib.crtp
# # from cflib.crazyflie import Crazyflie
# # from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# # from cflib.crazyflie.log import LogConfig
# # from cflib.positioning.motion_commander import MotionCommander
# #
# # # ---------- USER / LAB CONFIG ----------
# # QTM_IP = "192.168.0.105"
# # URI    = "radio://0/90/2M"
# #
# # # From your calibration
# # ORIGIN_MM = np.array([14131.634244791667, 423.228471883138, -7.1656776110331215], float)
# # SCALE_M_PER_MM = 0.001
# #
# # # Streaming / verification
# # STREAM_HZ        = 100
# # VERIFY_HZ        = 50
# # VERIFY_WINDOW_S  = 2.0
# # ANGLE_OK_DEG     = 15.0
# # PROBE_SECONDS    = 2.5   # keep CF still during probe
# #
# # # Flight demo (OFF by default for safety)
# # DO_FLIGHT_TEST        = True
# # HOVER_HEIGHT_M        = 0.50
# # XY_LOCK_AT_TAKEOFF    = True
# #
# # # During flight, if mean angle error goes above this for a few seconds → fall back to position-only
# # ANGLE_FALLBACK_DEG    = 35.0
# # FALLBACK_HYST_S       = 1.5
# # # --------------------------------------
# #
# #
# # # ---------- math helpers ----------
# # def rotm_to_quat(R):
# #     """3x3 rotation matrix -> quaternion (x,y,z,w), right-handed."""
# #     m = np.asarray(R, float)
# #     t = np.trace(m)
# #     if t > 0:
# #         s = np.sqrt(t + 1.0) * 2.0
# #         w = 0.25 * s
# #         x = (m[2,1] - m[1,2]) / s
# #         y = (m[0,2] - m[2,0]) / s
# #         z = (m[1,0] - m[0,1]) / s
# #     else:
# #         i = int(np.argmax(np.diag(m)))
# #         if i == 0:
# #             s = np.sqrt(1.0 + m[0,0] - m[1,1] - m[2,2]) * 2.0
# #             x = 0.25 * s
# #             y = (m[0,1] + m[1,0]) / s
# #             z = (m[0,2] + m[2,0]) / s
# #             w = (m[2,1] - m[1,2]) / s
# #         elif i == 1:
# #             s = np.sqrt(1.0 + m[1,1] - m[0,0] - m[2,2]) * 2.0
# #             x = (m[0,1] + m[1,0]) / s
# #             y = 0.25 * s
# #             z = (m[1,2] + m[2,1]) / s
# #             w = (m[0,2] - m[2,0]) / s
# #         else:
# #             s = np.sqrt(1.0 + m[2,2] - m[0,0] - m[1,1]) * 2.0
# #             x = (m[0,2] + m[2,0]) / s
# #             y = (m[1,2] + m[2,1]) / s
# #             z = 0.25 * s
# #             w = (m[1,0] - m[0,1]) / s
# #     q = np.array([x, y, z, w], float)
# #     n = np.linalg.norm(q)
# #     return q / n if n > 0 else np.array([0,0,0,1], float)
# #
# # def quat_align_sign(q_ref, q):
# #     return q if np.dot(q_ref, q) >= 0 else -q
# #
# # def quat_angle_deg(q1, q2):
# #     q1 = q1 / np.linalg.norm(q1); q2 = q2 / np.linalg.norm(q2)
# #     d = abs(np.dot(q1, q2)); d = np.clip(d, -1.0, 1.0)
# #     return np.degrees(2.0 * np.arccos(d))
# #
# # def try_set_param(cf, name, value):
# #     try:
# #         cf.param.set_value(name, value); return True
# #     except KeyError:
# #         return False
# #
# #
# # # ---------- QTM thread (6d) ----------
# # class QTMThread(threading.Thread):
# #     def __init__(self, ip, out_pose):
# #         super().__init__(daemon=True); self.ip = ip; self.out = out_pose
# #         self.stop_flag = threading.Event()
# #     def run(self): asyncio.run(self._amain())
# #     async def _amain(self):
# #         print("[QTM] event loop…")
# #         conn = None
# #         try:
# #             conn = await qtm_rt.connect(self.ip)
# #             if not conn:
# #                 print(f"[QTM] ERROR: cannot connect {self.ip}"); return
# #             await conn.stream_frames(components=["6d"], on_packet=self._on_packet)
# #             print("[QTM] streaming '6d'")
# #             while not self.stop_flag.is_set():
# #                 await asyncio.sleep(0.01)
# #         except Exception as e:
# #             print(f"[QTM] ERROR: {e}")
# #         finally:
# #             try:
# #                 if conn: await conn.disconnect()
# #             except Exception: pass
# #     def _on_packet(self, packet):
# #         try:
# #             info, bodies = packet.get_6d()
# #             if not bodies: return
# #             pos_mm, rotm = bodies[0]
# #             self.out["pos_mm"] = np.array(pos_mm, float)
# #             self.out["rotm"]   = np.array(rotm, float).reshape(3,3)
# #             self.out["ts"]     = time.time()
# #         except Exception as e:
# #             print(f"[QTM] parse error: {e}")
# #     def stop(self): self.stop_flag.set()
# #
# #
# # def main():
# #     # Start QTM
# #     shared = {"pos_mm": None, "rotm": None, "ts": 0.0}
# #     qtm = QTMThread(QTM_IP, shared); qtm.start()
# #     t0 = time.time()
# #     while (shared["pos_mm"] is None or shared["rotm"] is None) and time.time() - t0 < 5.0:
# #         time.sleep(0.01)
# #     if shared["pos_mm"] is None:
# #         print("No QTM frames. Enable Real-Time Server + 6d."); return
# #
# #     print("First QTM frame (mm):", shared["pos_mm"])
# #     print("Using fixed origin_mm (mm):", ORIGIN_MM)
# #     print("Scale (mm->m):", SCALE_M_PER_MM)
# #
# #     # Connect CF
# #     cflib.crtp.init_drivers(enable_debug_driver=False)
# #     cf = Crazyflie(rw_cache="./cache")
# #     with SyncCrazyflie(URI, cf=cf) as scf:
# #         print("Connected to Crazyflie.")
# #         try_set_param(cf, "stabilizer.estimator", "2")
# #
# #         # Quaternion log for verification
# #         quat_log_ok = True
# #         sent_quat_last = np.array([0,0,0,1], float)
# #         recv_quat_last = np.array([0,0,0,1], float)
# #         angle_errors = []  # (t, angle_deg)
# #         def log_cb(ts, data, logconf):
# #             nonlocal recv_quat_last
# #             try:
# #                 recv_quat_last = np.array([
# #                     float(data["stateEstimate.qx"]),
# #                     float(data["stateEstimate.qy"]),
# #                     float(data["stateEstimate.qz"]),
# #                     float(data["stateEstimate.qw"]),
# #                 ], float)
# #             except KeyError:
# #                 pass
# #         logconf = LogConfig(name="quat", period_in_ms=int(1000.0/max(VERIFY_HZ,1)))
# #         for v in ("stateEstimate.qx","stateEstimate.qy","stateEstimate.qz","stateEstimate.qw"):
# #             try: logconf.add_variable(v, "float")
# #             except KeyError: quat_log_ok = False; break
# #         if quat_log_ok:
# #             try:
# #                 scf.cf.log.add_config(logconf)
# #                 logconf.data_received_cb.add_callback(log_cb)
# #                 logconf.start()
# #                 print("[LOG] Quaternion logging enabled.")
# #             except Exception as e:
# #                 print(f"[LOG] Quaternion logging unavailable: {e}")
# #                 quat_log_ok = False
# #         else:
# #             print("[LOG] Quaternion vars not in TOC; skipping verification.")
# #
# #         # Warmup (identity send, just to get data moving)
# #         print("Warming up extpose…")
# #         last_p = np.zeros(3); last_q = np.array([0,0,0,1], float)
# #         warm_end = time.time() + 0.4
# #         while time.time() < warm_end:
# #             pm = shared["pos_mm"]; Rm = shared["rotm"]
# #             if pm is not None and Rm is not None:
# #                 d = (pm - ORIGIN_MM) * SCALE_M_PER_MM
# #                 last_p = np.array([d[0], d[1], d[2]], float)
# #                 last_q = rotm_to_quat(Rm)
# #                 sent_quat_last = last_q
# #                 scf.cf.extpos.send_extpose(last_p[0], last_p[1], last_p[2],
# #                                             last_q[0], last_q[1], last_q[2], last_q[3])
# #             time.sleep(1.0/STREAM_HZ)
# #
# #         # Prepare candidate rotations B (QTM→CF)
# #         B_id     = np.eye(3)
# #         B_yaw90  = np.array([[0,-1,0],[1,0,0],[0,0,1]], float)
# #         B_yawm90 = np.array([[0, 1,0],[-1,0,0],[0,0,1]], float)
# #         B_yaw180 = np.array([[-1,0,0],[0,-1,0],[0,0,1]], float)
# #         candidates = [("identity",B_id), ("yaw+90",B_yaw90), ("yaw-90",B_yawm90), ("yaw+180",B_yaw180)]
# #
# #         # Probe B (CF flat & still!)
# #         def probe_B(B, secs=PROBE_SECONDS):
# #             nonlocal sent_quat_last
# #             errs=[]; t_end=time.time()+secs
# #             while time.time()<t_end:
# #                 pm = shared["pos_mm"]; Rm = shared["rotm"]
# #                 if pm is not None and Rm is not None:
# #                     d = (pm - ORIGIN_MM) * SCALE_M_PER_MM
# #                     p_cf = B @ d
# #                     R_cf = B @ Rm
# #                     q = rotm_to_quat(R_cf)
# #                     if np.dot(q, sent_quat_last) < 0: q = -q
# #                     scf.cf.extpos.send_extpose(p_cf[0], p_cf[1], p_cf[2], q[0], q[1], q[2], q[3])
# #                     sent_quat_last = q
# #                     if quat_log_ok:
# #                         q_recv = quat_align_sign(q, recv_quat_last)
# #                         errs.append(quat_angle_deg(q, q_recv))
# #                 time.sleep(1.0/STREAM_HZ)
# #             return float(np.mean(errs)) if errs else 1e9
# #
# #         print("Auto-mapping: probing candidates… (keep CF flat & still)")
# #         best_name, best_B, best_err = None, None, 1e9
# #         for name,B in candidates:
# #             mdeg = probe_B(B)
# #             print(f"  B={name:8s}: mean quat error ≈ {mdeg:.1f}°")
# #             if mdeg < best_err: best_name, best_B, best_err = name, B, mdeg
# #         print(f"Chosen B: {best_name}  (mean error ≈ {best_err:.1f}°)")
# #         good_alignment = (best_err <= ANGLE_OK_DEG)
# #
# #         # >>> FIX #1: recompute XY-bias IN CHOSEN FRAME <<<
# #         pm = shared["pos_mm"]; Rm = shared["rotm"]
# #         if pm is not None:
# #             d_now = (pm - ORIGIN_MM) * SCALE_M_PER_MM
# #             p_now_cf = best_B @ d_now
# #         else:
# #             p_now_cf = np.zeros(3)
# #         xy_bias = np.array([p_now_cf[0], p_now_cf[1], 0.0], float) if XY_LOCK_AT_TAKEOFF else np.zeros(3)
# #
# #         # >>> FIX #2: re-reset EKF using transformed pose (after XY-lock) <<<
# #         x0, y0, z0 = (p_now_cf[0]-xy_bias[0], p_now_cf[1]-xy_bias[1], p_now_cf[2])
# #         try_set_param(cf, "kalman.initialX", f"{x0:.3f}")
# #         try_set_param(cf, "kalman.initialY", f"{y0:.3f}")
# #         try_set_param(cf, "kalman.initialZ", f"{z0:.3f}")
# #         try_set_param(cf, "kalman.resetEstimation", "1"); time.sleep(0.12)
# #         try_set_param(cf, "kalman.resetEstimation", "0")
# #         print(f"Estimator reset at transformed pose: ({x0:.3f},{y0:.3f},{z0:.3f})")
# #
# #         # Background streaming with best_B; live verify; fallback if needed
# #         stop_evt = threading.Event()
# #         sent_mode_pose = True  # start with pose (xyz+quat); may fall back to position-only
# #
# #         sent_quat_last = np.array([0,0,0,1], float)  # re-init for stability
# #         last_print = 0.0
# #         fallback_since = None
# #
# #         def stream_loop():
# #             nonlocal sent_quat_last, sent_mode_pose, fallback_since
# #             period = 1.0/STREAM_HZ; next_t = time.time()
# #             while not stop_evt.is_set():
# #                 pm = shared["pos_mm"]; Rm = shared["rotm"]
# #                 if pm is not None and Rm is not None:
# #                     d = (pm - ORIGIN_MM) * SCALE_M_PER_MM
# #                     p_cf = (best_B @ d) - xy_bias
# #                     R_cf = best_B @ Rm
# #                     q = rotm_to_quat(R_cf)
# #                     if np.dot(q, sent_quat_last) < 0: q = -q
# #                     # Verify error
# #                     avg_err = None
# #                     if quat_log_ok:
# #                         q_recv = quat_align_sign(q, recv_quat_last)
# #                         ang = quat_angle_deg(q, q_recv)
# #                         angle_errors.append((time.time(), ang))
# #                         cutoff = time.time() - VERIFY_WINDOW_S
# #                         while angle_errors and angle_errors[0][0] < cutoff:
# #                             angle_errors.pop(0)
# #                         if angle_errors:
# #                             avg_err = sum(a for _,a in angle_errors)/len(angle_errors)
# #                     # Decide pose vs pos-only
# #                     use_pose = True
# #                     if avg_err is not None and avg_err > ANGLE_FALLBACK_DEG:
# #                         if fallback_since is None:
# #                             fallback_since = time.time()
# #                         elif time.time() - fallback_since > FALLBACK_HYST_S:
# #                             use_pose = False
# #                     else:
# #                         fallback_since = None
# #                     sent_mode_pose = use_pose
# #
# #                     if use_pose:
# #                         scf.cf.extpos.send_extpose(p_cf[0], p_cf[1], p_cf[2], q[0], q[1], q[2], q[3])
# #                         sent_quat_last = q
# #                     else:
# #                         scf.cf.extpos.send_extpos(p_cf[0], p_cf[1], p_cf[2])
# #
# #                     # debug print
# #                     nonlocal last_print
# #                     if time.time() - last_print > 1.0:
# #                         mode = "POSE" if use_pose else "POS-ONLY"
# #                         if avg_err is not None:
# #                             print(f"[verify] mean quat err {avg_err:5.1f}°  mode={mode}   p=({p_cf[0]:+.2f},{p_cf[1]:+.2f},{p_cf[2]:+.2f})")
# #                         else:
# #                             print(f"[verify] mode={mode}   p=({p_cf[0]:+.2f},{p_cf[1]:+.2f},{p_cf[2]:+.2f})")
# #                         last_print = time.time()
# #
# #                 next_t += period
# #                 dt = next_t - time.time()
# #                 if dt > 0: time.sleep(dt)
# #                 else: next_t = time.time()
# #
# #         th = threading.Thread(target=stream_loop, daemon=True); th.start()
# #         print(f"Streaming (best_B='{best_name}', xy_lock={XY_LOCK_AT_TAKEOFF})…")
# #
# #         # Decide flight
# #         can_fly = good_alignment  # require decent alignment to start
# #         if not can_fly:
# #             print("Alignment not good; flight blocked. Observe logs, adjust room axes/trackable, then re-run.")
# #         try:
# #             if DO_FLIGHT_TEST and can_fly:
# #                 try_set_param(cf, "commander.enHighLevel", "1")
# #                 print(f"Takeoff to {HOVER_HEIGHT_M:.2f} m …")
# #                 with MotionCommander(scf, default_height=HOVER_HEIGHT_M) as mc:
# #                     time.sleep(1.5)
# #                     side, speed = 0.25, 0.2
# #                     print("Small square…")
# #                     mc.forward(side, velocity=speed); time.sleep(0.2)
# #                     mc.left(side,    velocity=speed); time.sleep(0.2)
# #                     mc.back(side,    velocity=speed); time.sleep(0.2)
# #                     mc.right(side,   velocity=speed); time.sleep(0.2)
# #                     time.sleep(1.5)
# #                     print("Landing…"); mc.land(); time.sleep(2.0)
# #             else:
# #                 if DO_FLIGHT_TEST:
# #                     print("DO_FLIGHT_TEST=True but alignment not good → not flying.")
# #                 print("Press Ctrl+C to stop streaming.")
# #                 while True: time.sleep(0.5)
# #         except KeyboardInterrupt:
# #             print("Interrupted.")
# #         finally:
# #             stop_evt.set(); th.join(timeout=1.0)
# #             print("Stopped streaming.")
# #             try:
# #                 if quat_log_ok: logconf.stop()
# #             except Exception: pass
# #
# #     qtm.stop(); qtm.join(timeout=1.0)
# #     print("Exit.")
# #
# #
# # if __name__ == "__main__":
# #     main()
#
#
# import asyncio
# import time
# import threading
# import numpy as np
# from scipy.spatial.transform import Rotation as R
#
# import qtm_rt
# import cflib.crtp
# from cflib.crazyflie import Crazyflie
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.positioning.motion_commander import MotionCommander
#
# # -------- FIXED CONFIG FROM YOUR SETUP --------
# QTM_IP = "192.168.0.105"  # QTM real-time server
# URI = "radio://0/90/2M"  # Crazyflie radio URI
#
# # Origin in QTM units (mm) from your JSON
# ORIGIN_MM = np.array([
#     14131.634244791667,
#     423.228471883138,
#     -7.1656776110331215
# ], dtype=float)
#
# SCALE_M_PER_MM = 0.001  # QTM mm -> meters
# STREAM_HZ = 100  # extpose send rate
# HOVER_HEIGHT_M = 0.50  # takeoff height
# HOVER_SECONDS = 12.0  # hold duration
# DO_HOVER_DEMO = True  # False = only stream extpose
# XY_LOCK_AT_TAKEOFF = True  # make current XY -> (0,0) at reset
#
#
# # ------------------------------------------------
#
# # ---------- QTM thread (6d) ----------
# class QTMThread(threading.Thread):
#     def __init__(self, ip, out_pose):
#         super().__init__(daemon=True)
#         self.ip = ip
#         self.out = out_pose  # {"pos_mm": None, "rotm": None, "ts": 0.0}
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
#             # Stream 6d data which includes position and rotation matrix
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
#             pos_mm, rotm = bodies[0]
#             self.out["pos_mm"] = np.array(pos_mm, dtype=float)
#             self.out["rotm"] = np.array(rotm, dtype=float).reshape(3, 3)
#             self.out["ts"] = time.time()
#         except Exception as e:
#             print(f"[QTM] parse error: {e}")
#
#     def stop(self):
#         self.stop_flag.set()
#
#
# # ---------- Mapping B (identity) + helper ----------
# def map_pose_to_cf(pos_mm, rotm, origin_mm, scale=SCALE_M_PER_MM, xy_bias=np.zeros(3)):
#     """
#     Mapping B (identity) for position: CF.x = QTM.x, CF.y = QTM.y, CF.z = QTM.z
#     Rotation matrix is passed directly as QTM uses the same standard.
#     """
#     # Convert position
#     d = (pos_mm - origin_mm) * float(scale)
#     pos_cf = np.array([d[0], d[1], d[2]], dtype=float)
#     pos_cf = pos_cf - xy_bias  # apply xy lock
#
#     # Convert rotation matrix to quaternion
#     rot = R.from_matrix(rotm)
#     quat_cf = rot.as_quat()  # returns (x, y, z, w)
#
#     return pos_cf, quat_cf
#
#
# def try_set_param(cf, name, value):
#     try:
#         cf.param.set_value(name, value)
#         return True
#     except KeyError:
#         return False
#
#
# def main():
#     # 1) Start QTM thread and wait for frames
#     shared = {"pos_mm": None, "rotm": None, "ts": 0.0}
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
#
#     # 2) Connect Crazyflie
#     cflib.crtp.init_drivers(enable_debug_driver=False)
#     cf = Crazyflie(rw_cache="./cache")
#     with SyncCrazyflie(URI, cf=cf) as scf:
#         print("Connected to Crazyflie.")
#
#         # Ensure EKF (Kalman) if available
#         try_set_param(cf, "stabilizer.estimator", "2")
#         try_set_param(cf, "commander.enHighLevel", "1")
#
#         # 3) Warm-up: send a few extpose frames before reset
#         print("Warming up extpose…")
#         last_p = np.zeros(3)
#         xy_bias = np.zeros(3)
#         warm_end = time.time() + 0.4
#         while time.time() < warm_end:
#             pm = shared["pos_mm"]
#             rotm = shared["rotm"]
#             if pm is not None and rotm is not None:
#                 p, q = map_pose_to_cf(pm, rotm, ORIGIN_MM, scale=SCALE_M_PER_MM, xy_bias=np.zeros(3))
#                 last_p = p
#                 # Note the change to send_extpose which takes quaternion (x,y,z,w)
#                 cf.extpos.send_extpose(p[0], p[1], p[2], q[0], q[1], q[2], q[3])
#             time.sleep(1.0 / STREAM_HZ)
#
#         # 4) XY-lock at takeoff
#         if XY_LOCK_AT_TAKEOFF:
#             xy_bias = np.array([last_p[0], last_p[1], 0.0], dtype=float)
#
#         # 5) Reset EKF
#         x0, y0, z0 = (last_p[0] - xy_bias[0], last_p[1] - xy_bias[1], last_p[2])
#         try_set_param(cf, "kalman.initialX", f"{x0:.3f}")
#         try_set_param(cf, "kalman.initialY", f"{y0:.3f}")
#         try_set_param(cf, "kalman.initialZ", f"{z0:.3f}")
#         try_set_param(cf, "kalman.resetEstimation", "1");
#         time.sleep(0.12);
#         try_set_param(cf, "kalman.resetEstimation", "0")
#         print("Estimator reset.")
#
#         # 6) Background streaming loop
#         stop_evt = threading.Event()
#
#         def stream_loop():
#             period = 1.0 / STREAM_HZ
#             next_t = time.time()
#             while not stop_evt.is_set():
#                 pm = shared["pos_mm"]
#                 rotm = shared["rotm"]
#                 if pm is not None and rotm is not None:
#                     p, q = map_pose_to_cf(pm, rotm, ORIGIN_MM, scale=SCALE_M_PER_MM, xy_bias=xy_bias)
#                     cf.extpos.send_extpose(p[0], p[1], p[2], q[0], q[1], q[2], q[3])
#                 next_t += period
#                 dt = next_t - time.time()
#                 if dt > 0:
#                     time.sleep(dt)
#                 else:
#                     next_t = time.time()
#
#         th = threading.Thread(target=stream_loop, daemon=True)
#         th.start()
#         print(f"Streaming extpose (position + quaternion) with XY-lock…")
#
#         # 7) Optional hover demo
#         try:
#             if DO_HOVER_DEMO:
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



"""
Crazyflie ← QTM external POSE (6d-only, robust)
- QTM: streams '6d' (pos[mm] + 3x3 rotation)
- Robust rot-matrix repair (SVD, det=+1). If invalid -> send position-only for that tick.
- Auto yaw-frame mapping (identity, +90°, -90°, 180°) checked vs CF EKF on the ground
- XY-lock + EKF reset AFTER choosing mapping
"""
#
# import asyncio
# import time
# import threading
# import numpy as np
#
# try:
#     from scipy.spatial.transform import Rotation as R
#     HAVE_SCIPY = True
# except Exception:
#     HAVE_SCIPY = False
#
# import qtm_rt
# import cflib.crtp
# from cflib.crazyflie import Crazyflie
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.crazyflie.log import LogConfig
# from cflib.positioning.motion_commander import MotionCommander
#
# # ---------- USER / LAB CONFIG ----------
# QTM_IP = "192.168.0.105"
# URI    = "radio://0/90/2M"
#
# ORIGIN_MM = np.array([14131.634244791667, 423.228471883138, -7.1656776110331215], float)
# SCALE_M_PER_MM = 0.001
#
# STREAM_HZ        = 100
# VERIFY_HZ        = 50
# VERIFY_WINDOW_S  = 2.0
# ANGLE_OK_DEG     = 15.0
# PROBE_SECONDS    = 2.5
#
# DO_FLIGHT_TEST        = True
# HOVER_HEIGHT_M        = 1
# XY_LOCK_AT_TAKEOFF    = True
#
# ANGLE_FALLBACK_DEG    = 35.0
# FALLBACK_HYST_S       = 1.5
# # --------------------------------------
#
#
# # ---------- math helpers ----------
# def rotm_to_quat_safe(Rm):
#     """Return (qx,qy,qz,qw) or None. Repairs matrix with SVD and enforces det=+1."""
#     m = np.asarray(Rm, float)
#     if m.shape != (3,3) or not np.isfinite(m).all():
#         return None
#     try:
#         U, _, Vt = np.linalg.svd(m)
#         R_ = U @ Vt
#         if np.linalg.det(R_) < 0:
#             U[:, -1] *= -1
#             R_ = U @ Vt
#     except Exception:
#         return None
#     if not np.isfinite(R_).all():
#         return None
#
#     if HAVE_SCIPY:
#         try:
#             q = R.from_matrix(R_).as_quat()   # (x,y,z,w)
#             n = np.linalg.norm(q)
#             return q / n if n > 0 else None
#         except Exception:
#             return None
#
#     # Manual conversion
#     t = np.trace(R_)
#     if t > 0:
#         s = np.sqrt(t + 1.0) * 2
#         qw = 0.25 * s
#         qx = (R_[2,1] - R_[1,2]) / s
#         qy = (R_[0,2] - R_[2,0]) / s
#         qz = (R_[1,0] - R_[0,1]) / s
#     else:
#         i = int(np.argmax(np.diag(R_)))
#         if i == 0:
#             s = np.sqrt(1.0 + R_[0,0] - R_[1,1] - R_[2,2]) * 2
#             qx = 0.25 * s
#             qy = (R_[0,1] + R_[1,0]) / s
#             qz = (R_[0,2] + R_[2,0]) / s
#             qw = (R_[2,1] - R_[1,2]) / s
#         elif i == 1:
#             s = np.sqrt(1.0 + R_[1,1] - R_[0,0] - R_[2,2]) * 2
#             qx = (R_[0,1] + R_[1,0]) / s
#             qy = 0.25 * s
#             qz = (R_[1,2] + R_[2,1]) / s
#             qw = (R_[0,2] - R_[2,0]) / s
#         else:
#             s = np.sqrt(1.0 + R_[2,2] - R_[0,0] - R_[1,1]) * 2
#             qx = (R_[0,2] + R_[2,0]) / s
#             qy = (R_[1,2] + R_[2,1]) / s
#             qz = 0.25 * s
#             qw = (R_[1,0] - R_[0,1]) / s
#     q = np.array([qx,qy,qz,qw], float)
#     n = np.linalg.norm(q)
#     return q / n if n > 0 and np.isfinite(n) else None
#
#
# def quat_angle_deg(q1, q2):
#     """Angle between (x,y,z,w) quaternions, degrees."""
#     if q1 is None or q2 is None: return 180.0
#     q1 = q1/np.linalg.norm(q1); q2 = q2/np.linalg.norm(q2)
#     d = abs(float(np.dot(q1, q2)))
#     d = np.clip(d, -1.0, 1.0)
#     return float(np.degrees(2.0 * np.arccos(d)))
#
#
# def try_set_param(cf, name, value):
#     try:
#         cf.param.set_value(name, value); return True
#     except KeyError:
#         return False
#
#
# # ---------- QTM thread (6d only) ----------
# class QTMThread(threading.Thread):
#     def __init__(self, ip, out_pose):
#         super().__init__(daemon=True); self.ip = ip; self.out = out_pose
#         self.stop_flag = threading.Event()
#
#     def run(self): asyncio.run(self._amain())
#
#     async def _amain(self):
#         print("[QTM] event loop…")
#         conn = None
#         try:
#             conn = await qtm_rt.connect(self.ip)
#             if not conn:
#                 print(f"[QTM] ERROR: cannot connect {self.ip}"); return
#             await conn.stream_frames(components=["6d"], on_packet=self._on_packet)
#             print("[QTM] streaming '6d'")
#             while not self.stop_flag.is_set():
#                 await asyncio.sleep(0.01)
#         except Exception as e:
#             print(f"[QTM] ERROR: {e}")
#         finally:
#             try:
#                 if conn: await conn.disconnect()
#             except Exception: pass
#
#     def _on_packet(self, packet):
#         try:
#             info, bodies = packet.get_6d()
#             if not bodies: return
#             pos_mm, rotm = bodies[0]
#             self.out["pos_mm"] = np.array(pos_mm, float)
#             self.out["rotm"]   = np.array(rotm, float).reshape(3,3)
#             self.out["ts"]     = time.time()
#         except Exception as e:
#             print(f"[QTM] parse error: {e}")
#
#     def stop(self): self.stop_flag.set()
#
#
# # ---------- Pose mapping (with yaw mapping B) ----------
# def map_pose_to_cf(pos_mm, rotm, origin_mm, B=np.eye(3), xy_bias=np.zeros(3)):
#     """
#     Returns (p_cf, q_cf or None). Builds quaternion from rotm safely; if invalid -> None.
#     B is 3x3 applied to both position and rotation (yaw-only candidates).
#     """
#     d = (pos_mm - origin_mm) * SCALE_M_PER_MM
#     p_cf = (B @ d.reshape(3,)) - xy_bias
#
#     q_cf = None
#     if rotm is not None and np.isfinite(rotm).all():
#         R_cf = B @ rotm
#         q_cf = rotm_to_quat_safe(R_cf)
#
#     return p_cf.astype(float), (q_cf if (q_cf is None or np.isfinite(q_cf).all()) else None)
#
#
# def send_pose(cf, p_xyz, q_xyzw=None):
#     """Send external pose if available; else position-only."""
#     has_extpose = hasattr(cf.extpos, "send_extpose")
#     if q_xyzw is not None and has_extpose:
#         cf.extpos.send_extpose(float(p_xyz[0]), float(p_xyz[1]), float(p_xyz[2]),
#                                float(q_xyzw[0]), float(q_xyzw[1]), float(q_xyzw[2]), float(q_xyzw[3]))
#     else:
#         cf.extpos.send_extpos(float(p_xyz[0]), float(p_xyz[1]), float(p_xyz[2]))
#
#
# def main():
#     # Start QTM
#     shared = {"pos_mm": None, "rotm": None, "ts": 0.0}
#     qtm = QTMThread(QTM_IP, shared); qtm.start()
#
#     t0 = time.time()
#     while shared["pos_mm"] is None and time.time() - t0 < 6.0:
#         time.sleep(0.01)
#     if shared["pos_mm"] is None:
#         print("No QTM frames. Ensure Real-Time Server is ON and '6d' is enabled."); return
#
#     print("First QTM frame (mm):", shared["pos_mm"])
#     print("Using fixed origin_mm (mm):", ORIGIN_MM)
#     print("Scale (mm->m):", SCALE_M_PER_MM)
#
#     # Connect CF
#     cflib.crtp.init_drivers(enable_debug_driver=False)
#     cf = Crazyflie(rw_cache="./cache")
#     with SyncCrazyflie(URI, cf=cf) as scf:
#         print("Connected to Crazyflie.")
#         try_set_param(cf, "stabilizer.estimator", "2")
#         try_set_param(cf, "commander.enHighLevel", "1")
#
#         # Log CF EKF quaternion (for mapping probe)
#         quat_log_ok = True
#         recv_quat_last = np.array([0,0,0,1], float)
#         angle_errors = []  # (t, angle)
#
#         def log_cb(ts, data, logconf):
#             nonlocal recv_quat_last
#             try:
#                 recv_quat_last = np.array([
#                     float(data["stateEstimate.qx"]),
#                     float(data["stateEstimate.qy"]),
#                     float(data["stateEstimate.qz"]),
#                     float(data["stateEstimate.qw"]),
#                 ], float)
#             except KeyError:
#                 pass
#
#         logconf = LogConfig(name="quat", period_in_ms=int(1000.0/max(VERIFY_HZ,1)))
#         for v in ("stateEstimate.qx","stateEstimate.qy","stateEstimate.qz","stateEstimate.qw"):
#             try: logconf.add_variable(v, "float")
#             except KeyError: quat_log_ok=False; break
#         if quat_log_ok:
#             try:
#                 scf.cf.log.add_config(logconf)
#                 logconf.data_received_cb.add_callback(log_cb)
#                 logconf.start()
#                 print("[LOG] Quaternion logging enabled.")
#             except Exception as e:
#                 print(f"[LOG] Quaternion logging unavailable: {e}")
#                 quat_log_ok=False
#         else:
#             print("[LOG] Quaternion vars not in TOC; skipping verification.")
#
#         # Warmup (identity mapping, no XY-bias yet)
#         print("Warming up ext pose…")
#         last_p = np.zeros(3); last_q = np.array([0,0,0,1], float)
#         warm_end = time.time() + 0.4
#         while time.time() < warm_end:
#             pm, rm = shared["pos_mm"], shared["rotm"]
#             if pm is not None:
#                 p, q = map_pose_to_cf(pm, rm, ORIGIN_MM, B=np.eye(3), xy_bias=np.zeros(3))
#                 last_p = p; last_q = q if q is not None else last_q
#                 send_pose(cf, p, q)
#             time.sleep(1.0/STREAM_HZ)
#
#         # Candidate yaw mappings
#         B_id     = np.eye(3)
#         B_yaw90  = np.array([[0,-1,0],[1,0,0],[0,0,1]], float)
#         B_yawm90 = np.array([[0, 1,0],[-1,0,0],[0,0,1]], float)
#         B_yaw180 = np.array([[-1,0,0],[0,-1,0],[0,0,1]], float)
#         candidates = [("identity",B_id), ("yaw+90",B_yaw90), ("yaw-90",B_yawm90), ("yaw+180",B_yaw180)]
#
#         def probe_B(B, secs=PROBE_SECONDS):
#             errs=[]; t_end=time.time()+secs
#             while time.time()<t_end:
#                 pm, rm = shared["pos_mm"], shared["rotm"]
#                 if pm is not None:
#                     p, q = map_pose_to_cf(pm, rm, ORIGIN_MM, B=B, xy_bias=np.zeros(3))
#                     send_pose(cf, p, q)  # q may be None -> pos-only
#                     if quat_log_ok and q is not None and np.linalg.norm(q)>0 and np.linalg.norm(recv_quat_last)>0:
#                         ang = quat_angle_deg(q/np.linalg.norm(q), recv_quat_last/np.linalg.norm(recv_quat_last))
#                         errs.append(ang)
#                 time.sleep(1.0/STREAM_HZ)
#             return float(np.mean(errs)) if errs else 1e9
#
#         print("Auto-mapping: probing yaw candidates… (keep CF flat & still)")
#         best_name, best_B, best_err = None, None, 1e9
#         for name,B in candidates:
#             mdeg = probe_B(B)
#             print(f"  B={name:8s}: mean quat error ≈ {mdeg:.1f}°")
#             if mdeg < best_err: best_name, best_B, best_err = name, B, mdeg
#         print(f"Chosen B: {best_name}  (mean error ≈ {best_err:.1f}°)")
#         good_alignment = (best_err <= ANGLE_OK_DEG)
#
#         # XY bias in chosen frame
#         pm, rm = shared["pos_mm"], shared["rotm"]
#         if pm is not None:
#             p_now, q_now = map_pose_to_cf(pm, rm, ORIGIN_MM, B=best_B, xy_bias=np.zeros(3))
#         else:
#             p_now = np.zeros(3)
#         xy_bias = np.array([p_now[0], p_now[1], 0.0], float) if XY_LOCK_AT_TAKEOFF else np.zeros(3)
#
#         # EKF reset in chosen frame (after XY-lock)
#         x0, y0, z0 = (p_now[0]-xy_bias[0], p_now[1]-xy_bias[1], p_now[2])
#         try_set_param(cf, "kalman.initialX", f"{x0:.3f}")
#         try_set_param(cf, "kalman.initialY", f"{y0:.3f}")
#         try_set_param(cf, "kalman.initialZ", f"{z0:.3f}")
#         try_set_param(cf, "kalman.resetEstimation", "1"); time.sleep(0.12)
#         try_set_param(cf, "kalman.resetEstimation", "0")
#         print(f"Estimator reset at transformed pose: ({x0:.3f},{y0:.3f},{z0:.3f})")
#
#         # Background streaming with best_B; live verify; fallback if needed
#         stop_evt = threading.Event()
#         sent_mode_pose = True
#         last_print = 0.0
#         fallback_since = None
#
#         def stream_loop():
#             nonlocal sent_mode_pose, fallback_since, last_print
#             period = 1.0/STREAM_HZ; next_t = time.time()
#             while not stop_evt.is_set():
#                 pm, rm = shared["pos_mm"], shared["rotm"]
#                 if pm is not None:
#                     p, q = map_pose_to_cf(pm, rm, ORIGIN_MM, B=best_B, xy_bias=xy_bias)
#
#                     # Rolling mean angle vs EKF
#                     avg_err = None
#                     if (quat_log_ok and q is not None and
#                         np.linalg.norm(q)>0 and np.linalg.norm(recv_quat_last)>0):
#                         ang = quat_angle_deg(q/np.linalg.norm(q), recv_quat_last/np.linalg.norm(recv_quat_last))
#                         angle_errors.append((time.time(), ang))
#                         cutoff = time.time() - VERIFY_WINDOW_S
#                         while angle_errors and angle_errors[0][0] < cutoff:
#                             angle_errors.pop(0)
#                         if angle_errors:
#                             avg_err = sum(a for _,a in angle_errors)/len(angle_errors)
#
#                     # Decide pose vs pos-only
#                     use_pose = (q is not None)
#                     if avg_err is not None and avg_err > ANGLE_FALLBACK_DEG:
#                         if fallback_since is None:
#                             fallback_since = time.time()
#                         elif time.time() - fallback_since > FALLBACK_HYST_S:
#                             use_pose = False
#                     else:
#                         fallback_since = None
#                     sent_mode_pose = use_pose
#
#                     # Send
#                     if use_pose:
#                         send_pose(cf, p, q)
#                     else:
#                         send_pose(cf, p, None)
#
#                     # debug
#                     if time.time() - last_print > 1.0:
#                         mode = "POSE" if use_pose else "POS-ONLY"
#                         if avg_err is not None:
#                             print(f"[verify] mean quat err {avg_err:5.1f}°  mode={mode}   p=({p[0]:+.2f},{p[1]:+.2f},{p[2]:+.2f})")
#                         else:
#                             print(f"[verify] mode={mode}   p=({p[0]:+.2f},{p[1]:+.2f},{p[2]:+.2f})")
#                         last_print = time.time()
#
#                 next_t += period
#                 dt = next_t - time.time()
#                 if dt > 0: time.sleep(dt)
#                 else: next_t = time.time()
#
#         th = threading.Thread(target=stream_loop, daemon=True); th.start()
#         print(f"Streaming (best_B='{best_name}', xy_lock={XY_LOCK_AT_TAKEOFF})…")
#
#         # Takeoff only if mapping is good
#         can_fly = good_alignment
#         if not can_fly:
#             print("Alignment not good; flight blocked. Adjust axes/trackable and re-run.")
#         try:
#             if DO_FLIGHT_TEST and can_fly:
#                 print(f"Takeoff to {HOVER_HEIGHT_M:.2f} m …")
#                 with MotionCommander(scf, default_height=HOVER_HEIGHT_M) as mc:
#                     time.sleep(2.0)
#                     mc.forward(0.20, velocity=0.15); time.sleep(0.5)
#                     mc.left(0.20,    velocity=0.15); time.sleep(0.5)
#                     mc.back(0.20,    velocity=0.15); time.sleep(0.5)
#                     mc.right(0.20,   velocity=0.15); time.sleep(0.5)
#                     time.sleep(1.0)
#                     print("Landing…"); mc.land(); time.sleep(2.0)
#             else:
#                 if DO_FLIGHT_TEST:
#                     print("DO_FLIGHT_TEST=True but alignment not good → not flying.")
#                 print("Press Ctrl+C to stop streaming.")
#                 while True: time.sleep(0.5)
#         except KeyboardInterrupt:
#             print("Interrupted.")
#         finally:
#             stop_evt.set(); th.join(timeout=1.0)
#             print("Stopped streaming.")
#             try:
#                 if quat_log_ok: logconf.stop()
#             except Exception: pass
#
#     qtm.stop(); qtm.join(timeout=1.0)
#     print("Exit.")
#
#
# if __name__ == "__main__":
#     main()



import asyncio
import time
import qtm_rt
import numpy as np
from scipy.spatial.transform import Rotation as R

QTM_IP = "192.168.0.105"
frame_counter = 0  # Global frame counter

async def main():
    connection = await qtm_rt.connect(QTM_IP)
    if connection is None:
        print("❌ Failed to connect to QTM.")
        return

    print("✅ Connected. Streaming '6d'...")

    def on_packet(packet):
        global frame_counter
        frame_counter += 1

        if frame_counter % 10 != 0:
            return

        try:
            _, bodies = packet.get_6d()
        except Exception as e:
            print(f"⚠️ Could not decode 6d: {e}")
            return

        if not bodies:
            print("No rigid bodies detected.")
            return

        pos_mm, rot_mat_flat = bodies[0]
        x, y, z = [coord / 1000.0 for coord in pos_mm]  # Convert mm → m

        try:
            rot_mat = np.array(rot_mat_flat).reshape(3, 3)
            r = R.from_matrix(rot_mat)
            roll, pitch, yaw = r.as_euler("xyz", degrees=True)
        except Exception as e:
            print(f"⚠️ Rotation decoding error: {e}")
            return

        print(f"[Frame {frame_counter}] pos = ({x:.3f}, {y:.3f}, {z:.3f}), "
              f"roll = {roll:.2f}°, pitch = {pitch:.2f}°, yaw = {yaw:.2f}°")


    await connection.stream_frames(components=["6d"], on_packet=on_packet)

    # Keep streaming
    while True:
        await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())

