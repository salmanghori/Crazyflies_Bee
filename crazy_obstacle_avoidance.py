import asyncio
import json
import math
import socket
import sys
import time
import xml.etree.cElementTree as ET
from threading import Thread
import threading  # for non-blocking ENTER Event

import qtm_rt
from qtm_rt import QRTCommandException
from scipy.spatial.transform import Rotation

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils.reset_estimator import reset_estimator

# ================== CONFIG ==================
CF_URI         = "radio://0/80/2M/E7E7E7E7E1"
QTM_IP         = "192.168.0.105"
RIGID_NAME     = "cf9"      # QTM body used for EKF & takeoff/hover

# MATLAB UDP
MATLAB_HOST    = "127.0.0.1"
MATLAB_PORT    = 56120      # MATLAB listens here
PYTHON_PORT    = 56121      # Python binds here to receive replies
DT             = 0.01       # seconds

# Marker layout by *index*
OBS8_INDICES = [0, 1, 2, 3, 4]   # obstacle (5 markers) → send ALL to MATLAB
CF9_INDICES  = [5, 6, 7, 8]      # robot (4 reflectors) → send FIRST THREE (5,6,7)

# Flight
TAKEOFF_Z         = 0.800   # Target altitude (meters)
TAKEOFF_TIME      = 2.0
BURST_SEC         = 0.3     # how long to apply (vx,vy) each step
POST_STEP_HOVER   = 3.0
LAND_TIME         = 3.0     # duration of smooth low-level land (seconds)

NUM_STEPS         = 20

# Control
SPEED_MAX         = 0.8     # clamp world-frame speed
SEND_FULL_POSE    = False   # extpose vs extpos to EKF
ENTER_TIMEOUT_SEC = 5.0     # non-blocking ENTER timeout

# --- v3 IMPROVEMENTS ---
# 1) For smoothness: S-curve (smoothstep) ramping
RAMP_PERCENT = 0.35  # (0.35 = 35% up, 30% hold, 35% down)

# 2) For robustness: step-level retries on marker loss
MAX_NAN_RETRIES = 5

# 3) Cross-step & per-tick rate-limiter (max accel)
MAX_DV = 1.2 * DT   # m/s per tick (≈1.2 m/s^2)

# 4) Optional: small deadzone to ignore tiny dithers from MATLAB
DEADZONE = 0.05

# Global previous commanded velocity (for cross-step continuity)
V_PREV = {"vx": 0.0, "vy": 0.0}
# ===========================================


# -------- Helpers --------
def _finite3(t):
    """True if t is a 3-tuple with no None/NaN."""
    return (t is not None and all(
        (v is not None) and (not (isinstance(v, float) and math.isnan(v))) for v in t))

def smoothstep(t):
    """S-curve interpolation; t in [0,1]."""
    if t <= 0: return 0.0
    if t >= 1: return 1.0
    return t * t * (3.0 - 2.0 * t)

def rate_limit(v_cmd, v_prev, max_dv):
    """Cap per-tick change in command."""
    dv = v_cmd - v_prev
    if dv >  max_dv: return v_prev + max_dv
    if dv < -max_dv: return v_prev - max_dv
    return v_cmd

def deadzone(v, eps=DEADZONE):
    return 0.0 if abs(v) < eps else v


# -------- QTM helpers (index-based) --------
def _extract_markers(packet):
    """Safely extracts 3D marker data from a QTM packet."""
    try:
        _hdr, mk = packet.get_3d_markers()
        return mk
    except Exception:
        pass
    try:
        mk = packet.get_3d_markers_no_label()
        return mk
    except Exception:
        return []

def _mm_to_m(pt):
    """Converts a (x,y,z) tuple from mm to m, checking for None and NaN."""
    if pt is None:
        return None
    x, y, z = pt
    if (x is None) or (y is None) or (z is None):
        return None
    if any(isinstance(v, float) and math.isnan(v) for v in (x, y, z)):
        return None
    return (x * 0.001, y * 0.001, z * 0.001)

def get_group_points_m(markers_mm, indices):
    """Gets a list of valid (x,y,z) points in meters for a list of indices."""
    out = []
    for i in indices:
        if i < len(markers_mm):
            m = _mm_to_m(markers_mm[i])
            if m is not None:
                out.append(m)
    return out


# ---------------- UDP helper ----------------
def mk_udp_pair():
    """Creates a UDP socket pair for sending (tx) and receiving (rx)."""
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("0.0.0.0", PYTHON_PORT))
    rx.settimeout(1.0)  # 1-second timeout on receives
    return tx, rx


# -------------- QTM wrapper -----------------
class QtmWrapper(Thread):
    """Handles QTM connection and data streaming in a separate thread."""
    def __init__(self, body_name: str, qtm_ip: str):
        super().__init__()
        self.daemon = True
        self.body_name = body_name
        self.qtm_ip = qtm_ip
        self.connection = None
        self.qtm_6DoF_labels = []
        self.on_pose = None  # Callback for 6DoF pose
        self._stay_open = True
        self._last_markers_mm = []       # latest 3D markers (raw mm)
        self._last_good_pose = None      # (x,y,z) in meters
        self._has_fix = False            # first valid pose arrived
        self.start()

    def close(self):
        """Signals the thread to close and joins it."""
        self._stay_open = False
        self.join()

    def run(self):
        """Thread entrypoint."""
        asyncio.run(self._life_cycle())

    async def _life_cycle(self):
        """Main async loop for connection and streaming."""
        ok = await self._connect()
        if not ok:
            print("[QTM] ERROR: failed to connect")
            return
        try:
            while self._stay_open:
                await asyncio.sleep(1)
        finally:
            await self._close()

    async def _connect(self) -> bool:
        """Establishes connection and starts streaming."""
        print(f"[QTM] Connecting to {self.qtm_ip} ...")
        self.connection = await qtm_rt.connect(self.qtm_ip)
        if self.connection is None:
            return False

        # Get 6DoF body labels
        try:
            params = await self.connection.get_parameters(parameters=['6d'])
            xml = ET.fromstring(params)
            self.qtm_6DoF_labels = [label.text.strip() for label in xml.findall('*/Body/Name')]
            print(f"[QTM] 6D labels: {self.qtm_6DoF_labels}")
            if self.body_name not in self.qtm_6DoF_labels:
                print(f"[QTM] WARNING: Body '{self.body_name}' not found, defaulting to index 0")
        except Exception as e:
            print(f"[QTM] WARNING: could not parse labels: {e}")

        # Start streaming (try 6deuler first, fallback to 6d)
        try:
            await self.connection.stream_frames(
                frames='allframes',
                components=['6deuler', '3d'],
                on_packet=self._on_packet_both
            )
            print("[QTM] Streaming 6deuler + 3d for EKF + MATLAB payloads")
        except QRTCommandException:
            await self.connection.stream_frames(
                frames='allframes',
                components=['6d', '3d'],
                on_packet=self._on_packet_both
            )
            print("[QTM] Streaming 6d + 3d")
        return True

    def _body_index(self) -> int:
        """Finds the numerical index for our rigid body name."""
        if self.qtm_6DoF_labels and (self.body_name in self.qtm_6DoF_labels):
            return self.qtm_6DoF_labels.index(self.body_name)
        return 0

    def _on_packet_both(self, packet):
        """Callback for each QTM data packet."""
        # 1) Always update raw 3D markers (for MATLAB payloads)
        try:
            self._last_markers_mm = _extract_markers(packet)
        except Exception:
            self._last_markers_mm = []

        # 2) 6D pose for EKF feed (use last-good on error)
        pos_mm = None
        idx = self._body_index()
        try:
            _, bodies = packet.get_6d_euler()
            if bodies and idx < len(bodies):
                pos_mm, _ = bodies[idx]
        except Exception:
            try:
                _, bodies = packet.get_6d()
                if bodies and idx < len(bodies):
                    pos_mm, _ = bodies[idx]
            except Exception:
                pass

        send_now = False
        if _finite3(pos_mm):
            x, y, z = pos_mm[0]*0.001, pos_mm[1]*0.001, pos_mm[2]*0.001
            self._last_good_pose = (x, y, z)
            self._has_fix = True
            send_now = True
        else:
            # Bad frame: send last-good only if we ever had one
            if self._has_fix and (self._last_good_pose is not None):
                send_now = True

        if send_now and self.on_pose:
            rot3 = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]  # yaw ignored downstream
            self.on_pose([*self._last_good_pose, rot3, 0.0])

    def latest_markers_mm(self):
        """Public accessor for the latest 3D marker data."""
        return self._last_markers_mm

    @property
    def has_fix(self):
        return self._has_fix

    async def _close(self):
        """Stops stream and disconnects."""
        try:
            await self.connection.stream_frames_stop()
        except Exception:
            pass
        if self.connection:
            self.connection.disconnect()
        print("[QTM] Disconnected")


# -------- extpos feed ----------
def send_extpose(cf, x, y, z, rot3):
    """Sends pose data to the Crazyflie EKF."""
    if SEND_FULL_POSE:
        try:
            q = Rotation.from_matrix(rot3).as_quat()
        except Exception:
            q = [0, 0, 0, 1]  # Default quaternion
        cf.extpos.send_extpose(x, y, z, q[0], q[1], q[2], q[3])
    else:
        cf.extpos.send_extpos(x, y, z)


# ---------- control helpers ----------
def clamp(vx, vy, vmax):
    """Clamps the magnitude of a 2D velocity vector (vx, vy) to vmax."""
    s = math.hypot(vx, vy)
    if s > vmax and s > 0:
        sc = vmax / s
        return vx * sc, vy * sc
    return vx, vy

def world_to_body_yaw_zero(vx, vy):
    """Transforms world velocity to body velocity, assuming yaw=0."""
    return vx, vy


# ---------- global state populated by QTM callback ----------
STATE = {"x": 0.0, "y": 0.0, "z": 0.0}

def make_pose_cb(cf):
    """Creates a callback function to feed QTM data to the EKF."""
    last_print = {"t": 0.0}
    def _cb(pose):
        x, y, z, rot3, _yaw_ignored = pose

        # Update global state
        STATE["x"], STATE["y"], STATE["z"] = x, y, z

        # Send to Crazyflie EKF
        send_extpose(cf, x, y, z, rot3)

        # Print ~1Hz
        t = time.time()
        if t - last_print["t"] > 1.0:
            if not (math.isnan(x) or math.isnan(y) or math.isnan(z)):
                print(f"[EXTPOS] x={x:.3f} y={y:.3f} z={z:.3f}")
            else:
                print("[EXTPOS] WARNING: Receiving NaN from QTM!")
            last_print["t"] = t
    return _cb


# ---------- non-blocking ENTER with timeout ----------
def wait_for_enter_with_timeout(prompt: str, timeout_sec: float) -> bool:
    """Prints a prompt and waits for ENTER, with a timeout."""
    print(prompt, end="", flush=True)
    done = threading.Event()
    pressed = {"ok": False}

    def reader():
        try:
            input()
            pressed["ok"] = True
        except Exception:
            pressed["ok"] = False
        finally:
            done.set()

    Thread(target=reader, daemon=True).start()
    done.wait(timeout_sec)
    print()  # Newline after input or timeout
    return pressed["ok"]


# ---------- smooth low-level landing ----------
def smooth_low_level_land(low, z_start, duration_s=3.0, z_end=0.05):
    """Commands a smooth landing ramp from z_start to z_end."""
    z_start = max(0.05, float(z_start))
    z_end   = max(0.03, float(z_end))
    steps = max(1, int(duration_s / DT))

    print(f"[CF] Smooth landing from {z_start:.3f}m...")
    vx_prev = 0.0
    vy_prev = 0.0

    for k in range(steps):
        a = (k + 1) / steps
        z = z_start + a * (z_end - z_start)
        # hold X-Y with zero velocity (assuming EKF extpos OK)
        vxt = rate_limit(0.0, vx_prev, MAX_DV)
        vyt = rate_limit(0.0, vy_prev, MAX_DV)
        low.send_hover_setpoint(vxt, vyt, 0.0, z)
        vx_prev, vy_prev = vxt, vyt
        time.sleep(DT)

    # Stop commanding
    low.send_stop_setpoint()
    print("[CF] Land complete.")


# ---------- main multi-step sequence ----------
def run_multi_step(cf, qtm: QtmWrapper):
    """
    Main experiment logic:
    - Takeoff
    - Wait for ENTER
    - Loop N steps:
        - Get markers (with micro-retry)
        - Send to MATLAB
        - Get (vx, vy) from MATLAB
        - Deadzone + clamp + transform
        - **Per-tick rate-limited** S-curve ramp up/hold/down
        - Handle marker loss with step-level retry
    - Land
    """
    cmd = cf.high_level_commander
    low = cf.commander

    # TAKEOFF (HL)
    print(f"\n[CF] TAKEOFF → z={TAKEOFF_Z:.3f} over {TAKEOFF_TIME:.1f}s")
    cmd.takeoff(TAKEOFF_Z, TAKEOFF_TIME)
    time.sleep(TAKEOFF_TIME + 0.6)
    print("[CF] Hovering …")

    # Reset cross-step velocity
    V_PREV["vx"] = 0.0
    V_PREV["vy"] = 0.0

    tx, rx = mk_udp_pair()

    def build_payload_from_markers(retries=8, sleep_s=0.01):
        """
        Gathers current marker data for MATLAB.
        Retries briefly to handle transient flickers.
        Returns payload dict, or None if markers are invalid after retries.
        """
        need = max(max(OBS8_INDICES), max(CF9_INDICES)) + 1
        for _ in range(retries):
            markers_mm = qtm.latest_markers_mm()
            if not markers_mm or len(markers_mm) < need:
                time.sleep(sleep_s)
                continue  # Not enough markers, retry

            try:
                robot_pts = []
                robot_indices_to_send = CF9_INDICES[:3]
                for i in robot_indices_to_send:
                    m = _mm_to_m(markers_mm[i])
                    if m is None:
                        raise ValueError("Invalid robot marker")
                    robot_pts.append([m[0], m[1], 0.0])

                obst_pts = []
                for i in OBS8_INDICES:
                    m = _mm_to_m(markers_mm[i])
                    if m is None:
                        raise ValueError("Invalid obstacle marker")
                    obst_pts.append([m[0], m[1], 0.0])

                # If we got here, all points are valid
                return {"dt": DT, "robot_xy_res": robot_pts, "obst_xy_res": obst_pts}

            except Exception:
                pass  # Invalid markers this frame, retry

            time.sleep(sleep_s)

        return None  # Failed all retries

    def do_one_step(step_idx: int) -> bool:
        """
        Sends markers to MATLAB, gets (vx,vy), and executes an
        S-CURVE RAMPED velocity burst with per-tick rate limiting.
        Returns True on success, False on marker loss.
        """
        label = f"STEP {step_idx+1}"

        # 1) Get marker data (with micro-retry)
        payload = build_payload_from_markers()
        if payload is None:
            print(f"[{label}] ERROR: Not enough valid markers (after retries).")
            return False  # Signal marker loss

        r = payload["robot_xy_res"]; o = payload["obst_xy_res"]
        print(f"[{label}] QTM→MATLAB robot (5,6,7): {[(round(p[0],3), round(p[1],3)) for p in r]}")
        print(f"[{label}] QTM→MATLAB obstacle (0..4): {[(round(p[0],3), round(p[1],3)) for p in o]}")

        # 2) Send to MATLAB
        tx.sendto(json.dumps(payload).encode("utf-8"), (MATLAB_HOST, MATLAB_PORT))

        # 3) Get reply from MATLAB
        vx = vy = 0.0
        try:
            data, _ = rx.recvfrom(2048)
            reply = json.loads(data.decode("utf-8"))
            vx = float(reply.get("vx", 0.0))
            vy = float(reply.get("vy", 0.0))
        except socket.timeout:
            print(f"[{label}] MATLAB reply timeout → using (0,0)")

        # 4) Deadzone, clamp, transform
        vx = deadzone(vx); vy = deadzone(vy)
        print(f"[{label}] MATLAB→PY velocity: world(vx={vx:.3f}, vy={vy:.3f})")
        vx, vy = clamp(vx, vy, SPEED_MAX)
        vx_b, vy_b = world_to_body_yaw_zero(vx, vy)  # yaw ignored
        yawrate_deg = 0.0

        # 5) Prepare S-curve profile
        total_steps = max(1, int(BURST_SEC / DT))
        ramp_steps = int(total_steps * RAMP_PERCENT)
        hold_steps = total_steps - (2 * ramp_steps)
        if hold_steps < 0:
            hold_steps = 0
            ramp_steps = total_steps // 2

        # Start from previous commanded velocity
        vx_prev = V_PREV["vx"]
        vy_prev = V_PREV["vy"]

        print(f"[{label}] Execute S-curve burst: body(target={vx_b:.3f},{vy_b:.3f}), yawrate={yawrate_deg:.1f}")

        # 6) Ramp Up (S-curve) with per-tick rate limit
        for k in range(ramp_steps):
            alpha = smoothstep((k + 1) / max(1, ramp_steps))
            vx_target = vx_b * alpha
            vy_target = vy_b * alpha
            vxt = rate_limit(vx_target, vx_prev, MAX_DV)
            vyt = rate_limit(vy_target, vy_prev, MAX_DV)
            low.send_hover_setpoint(vxt, vyt, yawrate_deg, TAKEOFF_Z)
            vx_prev, vy_prev = vxt, vyt
            time.sleep(DT)

        # 7) Hold (keep per-tick limit to absorb small command deltas)
        for k in range(hold_steps):
            vx_target = vx_b
            vy_target = vy_b
            vxt = rate_limit(vx_target, vx_prev, MAX_DV)
            vyt = rate_limit(vy_target, vy_prev, MAX_DV)
            low.send_hover_setpoint(vxt, vyt, yawrate_deg, TAKEOFF_Z)
            vx_prev, vy_prev = vxt, vyt
            time.sleep(DT)

        # 8) Ramp Down (S-curve) with per-tick rate limit
        for k in range(ramp_steps):
            alpha = smoothstep(1.0 - (k + 1) / max(1, ramp_steps))
            vx_target = vx_b * alpha
            vy_target = vy_b * alpha
            vxt = rate_limit(vx_target, vx_prev, MAX_DV)
            vyt = rate_limit(vy_target, vy_prev, MAX_DV)
            low.send_hover_setpoint(vxt, vyt, yawrate_deg, TAKEOFF_Z)
            vx_prev, vy_prev = vxt, vyt
            time.sleep(DT)

        # Save end-of-step velocity for continuity
        V_PREV["vx"], V_PREV["vy"] = vx_prev, vy_prev
        return True

    # SINGLE ENTER to start loop
    if not wait_for_enter_with_timeout(
        f"[USER] Press ENTER to start ({NUM_STEPS} steps) — {ENTER_TIMEOUT_SEC}s timeout → auto-land: ",
        ENTER_TIMEOUT_SEC
    ):
        print("[USER] No ENTER → landing now.")
        current_z = STATE.get("z", TAKEOFF_Z)
        smooth_low_level_land(low, current_z, duration_s=LAND_TIME, z_end=0.05)
        return

    # Run N steps with RETRY LOGIC
    k = 0  # Current step number
    nan_retry_count = 0

    while k < NUM_STEPS:
        step_ok = do_one_step(k)

        if step_ok:
            nan_retry_count = 0  # Reset counter on success
            k += 1               # Move to the next step
        else:
            # Marker loss detected
            nan_retry_count += 1
            print(f"[CF] WARNING: Marker loss at step {k+1}. Holding position. (Attempt {nan_retry_count}/{MAX_NAN_RETRIES})")

            if nan_retry_count >= MAX_NAN_RETRIES:
                print(f"[CF] Aborting after {MAX_NAN_RETRIES} failed attempts due to persistent marker loss.")
                current_z = STATE.get("z", TAKEOFF_Z)
                smooth_low_level_land(low, current_z, duration_s=LAND_TIME, z_end=0.05)
                return  # Exit function

            # Hold in place for one burst duration, then retry same step
            t_hold_start = time.time()
            vx_prev = V_PREV["vx"]
            vy_prev = V_PREV["vy"]
            while (time.time() - t_hold_start) < BURST_SEC:
                # Bring velocities gently to zero during hold
                vxt = rate_limit(0.0, vx_prev, MAX_DV)
                vyt = rate_limit(0.0, vy_prev, MAX_DV)
                low.send_hover_setpoint(vxt, vyt, 0.0, TAKEOFF_Z)
                vx_prev, vy_prev = vxt, vyt
                time.sleep(DT)
            # Keep continuity for next attempt
            V_PREV["vx"], V_PREV["vy"] = vx_prev, vy_prev

    # Post hover & smooth low-level land
    print(f"\n[CF] All {NUM_STEPS} steps completed successfully.")
    print(f"[CF] Post-steps hover {POST_STEP_HOVER:.1f}s (active zero command)…")
    t0 = time.time()
    vx_prev = V_PREV["vx"]
    vy_prev = V_PREV["vy"]
    while (time.time() - t0) < POST_STEP_HOVER:
        # decay to zero smoothly during hover
        vxt = rate_limit(0.0, vx_prev, MAX_DV)
        vyt = rate_limit(0.0, vy_prev, MAX_DV)
        low.send_hover_setpoint(vxt, vyt, 0.0, TAKEOFF_Z)
        vx_prev, vy_prev = vxt, vyt
        time.sleep(DT)
    V_PREV["vx"], V_PREV["vy"] = vx_prev, vy_prev

    print("[CF] LAND (low-level ramp)")
    current_z = STATE.get("z", TAKEOFF_Z)
    smooth_low_level_land(low, current_z, duration_s=LAND_TIME, z_end=0.05)
    print("[CF] Done.")


# --------------- entrypoint ----------------
def main():
    print("--- CF + QTM + MATLAB (multi-step, marker-indexed) [IMPROVED v3 S-Curve + Per-tick limiter] ---")
    cflib.crtp.init_drivers()

    qtm = QtmWrapper(RIGID_NAME, QTM_IP)

    try:
        with SyncCrazyflie(CF_URI, cf=Crazyflie(rw_cache="./cache")) as scf:
            cf = scf.cf
            print(f"[CF] Connected: {CF_URI}")

            # Configure EKF + HL
            cf.param.set_value('stabilizer.estimator', '2')   # EKF
            cf.param.set_value('commander.enHighLevel', '1')  # High Level Commander
            cf.param.set_value('locSrv.extQuatStdDev', 8.0e-3)
            time.sleep(0.2)

            # Start feeding extpos from QTM
            qtm.on_pose = make_pose_cb(cf)

            # Wait for first valid QTM pose (avoid feeding dummy)
            print("[CF] Waiting for first valid QTM pose...")
            t0 = time.time()
            while not qtm.has_fix:
                time.sleep(0.01)
                if time.time() - t0 > 5.0:
                    print("[CF] Still waiting for QTM pose...")
                    t0 = time.time()

            # Reset EKF (requires extpos flowing)
            print("[CF] Resetting EKF …")
            reset_estimator(cf)
            print("[CF] EKF reset OK")

            # Run the experiment
            run_multi_step(cf, qtm)

    except Exception as e:
        print(f"\n[FATAL ERROR] {e}")
        import traceback
        traceback.print_exc()

    finally:



        print("[PY] Shutting down QTM connection...")
        qtm.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[PY] KeyboardInterrupt — exit")
        sys.exit(0)
