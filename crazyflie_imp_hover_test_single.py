import asyncio
import math
import time
import xml.etree.cElementTree as ET
from threading import Thread

import qtm_rt
from qtm_rt import QRTCommandException
from scipy.spatial.transform import Rotation

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

# ------------- CONFIG -------------
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E1')
QTM_IP = "192.168.0.105"
rigid_body_name = r'cf9'

# Flight Parameters
TAKEOFF_HEIGHT = 1.0  # Height to reach during takeoff (meters)
TAKEOFF_TIME = 2.0  # Time to reach takeoff height (seconds)
HOVER_DURATION = 5.0  # Time to hold the hover after takeoff (seconds)
LAND_TIME = 2.0  # Time to perform landing (seconds)

send_full_pose = False
orientation_std_dev = 8.0e-3
EULER_ORDER = "ZYX"
EULER_DEGREES = False


# ----------------------------------

# --- QtmWrapper and Helper Functions are the same as before, they handle localization ---

class QtmWrapper(Thread):
    """Handles connection to QTM and streams position data to the Crazyflie."""

    def __init__(self, body_name, qtm_ip: str = QTM_IP):
        super().__init__()
        self.daemon = True
        self.body_name = body_name
        self.qtm_ip = qtm_ip
        self.on_pose = None
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
        ok = await self._connect()
        if not ok: return
        try:
            while self._stay_open: await asyncio.sleep(1)
        finally:
            await self._close()

    async def _connect(self) -> bool:
        # (Connection logic remains the same)
        print(f'[QtmWrapper] Connecting to QTM on {self.qtm_ip}')
        self.connection = await qtm_rt.connect(self.qtm_ip)
        if self.connection is None:
            print('[QtmWrapper] ERROR: Failed to connect to QTM.')
            return False

        try:
            params = await self.connection.get_parameters(parameters=['6d'])
            xml = ET.fromstring(params)
            self.qtm_6DoF_labels = [label.text.strip() for label in xml.findall('*/Body/Name')]
            if self.body_name not in self.qtm_6DoF_labels:
                print(f"[QtmWrapper] WARNING: Body '{self.body_name}' not found. Available: {self.qtm_6DoF_labels}")
        except Exception as e:
            print(f"[QtmWrapper] WARNING: Could not parse 6d labels ({e}); defaulting to index 0.")
            self.qtm_6DoF_labels = []

        try:
            await self.connection.stream_frames('allframes', ['6deuler'], on_packet=self._on_packet_6deuler)
            print("[QtmWrapper] Streaming '6deuler'.")
        except QRTCommandException:
            await self.connection.stream_frames('allframes', ['6d'], on_packet=self._on_packet_6d)
            print("[QtmWrapper] Streaming '6d'.")
        return True

    def _body_index(self) -> int:
        if self.qtm_6DoF_labels and self.body_name in self.qtm_6DoF_labels:
            return self.qtm_6DoF_labels.index(self.body_name)
        return 0

    def _on_packet_6deuler(self, packet):
        header, bodies = packet.get_6d_euler()
        if not bodies: return
        idx = self._body_index()
        if idx >= len(bodies): return
        pos_mm, euler = bodies[idx]
        if pos_mm is None or any(math.isnan(v) for v in pos_mm): return
        x, y, z = pos_mm[0] / 1000.0, pos_mm[1] / 1000.0, pos_mm[2] / 1000.0

        rot3 = None
        if euler is not None and not any(math.isnan(v) for v in euler):
            roll, pitch, yaw = euler
            try:
                if EULER_ORDER.upper() == 'ZYX':
                    rot3 = Rotation.from_euler('ZYX', [yaw, pitch, roll], degrees=EULER_DEGREES).as_matrix()
                else:
                    rot3 = Rotation.from_euler(EULER_ORDER, [roll, pitch, yaw], degrees=EULER_DEGREES).as_matrix()
            except Exception:
                pass

        if rot3 is None:
            rot3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

        if self.on_pose: self.on_pose([x, y, z, rot3])

    def _on_packet_6d(self, packet):
        header, bodies = packet.get_6d()
        if not bodies: return
        idx = self._body_index()
        if idx >= len(bodies): return
        pos_mm, rot = bodies[idx]
        if pos_mm is None or any(math.isnan(v) for v in pos_mm): return
        x, y, z = pos_mm[0] / 1000.0, pos_mm[1] / 1000.0, pos_mm[2] / 1000.0
        r = rot.matrix
        rot3 = [[r[0], r[3], r[6]], [r[1], r[4], r[7]], [r[2], r[5], r[8]]]

        if self.on_pose: self.on_pose([x, y, z, rot3])

    async def _close(self):
        try:
            await self.connection.stream_frames_stop()
        except Exception:
            pass
        if self.connection: self.connection.disconnect()
        print("[QtmWrapper] Disconnected from QTM.")


def send_extpose_rot_matrix(cf, x, y, z, rot):
    """Sends position and optional attitude data to the Crazyflie's EKF."""
    try:
        quat = Rotation.from_matrix(rot).as_quat()
    except Exception:
        quat = [0.0, 0.0, 0.0, 1.0]

    if send_full_pose:
        cf.extpos.send_extpose(x, y, z, quat[0], quat[1], quat[2], quat[3])
    else:
        cf.extpos.send_extpos(x, y, z)


def setup_crazyflie(cf):
    """Configures the Crazyflie for external position tracking and high-level commands."""
    print("[CF] Setting parameters for MoCap localization...")

    # 1. Set the stabilizer to use the EKF ('2')
    cf.param.set_value('stabilizer.estimator', '2')

    # 2. Adjust EKF sensitivity
    cf.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)
    cf.param.set_value('locSrv.extQuatStdDev', 0.06)

    # 3. Enable high-level commander (needed for takeoff/land)
    cf.param.set_value('commander.enHighLevel', '1')

    # 4. Reset the EKF estimate using the current position from QTM
    print("[CF] Resetting EKF. Waiting for stable position...")
    reset_estimator(cf)
    print("[CF] EKF Reset OK.")

    # 5. Arm the motors
    cf.platform.send_arming_request(True)
    time.sleep(0.5)


# --- NEW SIMPLIFIED FLIGHT SEQUENCE ---

def run_simple_sequence(cf):
    """Executes the simple Takeoff, Hover, and Land commands."""
    commander = cf.high_level_commander

    # 1. Takeoff
    print(f"🚀 TAKEOFF: to z={TAKEOFF_HEIGHT:.2f} m over {TAKEOFF_TIME:.1f}s")
    commander.takeoff(TAKEOFF_HEIGHT, TAKEOFF_TIME)
    time.sleep(TAKEOFF_TIME + 0.5)  # Wait for ascent + buffer
    print("✅ Takeoff complete. Holding hover.")

    # 2. Hover
    print(f"🕰️ HOVERING for {HOVER_DURATION:.1f}s at {TAKEOFF_HEIGHT:.2f}m")
    time.sleep(HOVER_DURATION)

    # 3. Land
    print(f"🛬 LANDING: over {LAND_TIME:.1f}s")
    # Land to z=0.0 to ensure the drone cuts motors when it hits the ground
    commander.land(0.0, LAND_TIME)
    time.sleep(LAND_TIME + 2.0)  # Wait for landing + buffer for motor cut

    # 4. Stop
    commander.stop()
    print("🛑 Sequence complete. Motors stopped.")


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # 1. Start QTM Connection and Streaming
    qtm_wrapper = QtmWrapper(rigid_body_name, qtm_ip=QTM_IP)

    try:
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            cf = scf.cf

            # 2. Link QTM data stream to Crazyflie EKF
            qtm_wrapper.on_pose = lambda pose: send_extpose_rot_matrix(
                cf, pose[0], pose[1], pose[2], pose[3])

            # 3. Setup and Arm
            setup_crazyflie(cf)

            # 4. Run the simplified flight
            run_simple_sequence(cf)

    except Exception as e:
        print(f"\n[ERROR] An error occurred: {e}")

    finally:
        # 5. Clean up QTM connection
        qtm_wrapper.close()


# ================================================================================


