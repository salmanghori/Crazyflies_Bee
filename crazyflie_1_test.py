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
from cflib.crazyflie.mem import MemoryElement
from cflib.crazyflie.mem import Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

# ------------- CONFIG -------------
# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E1')   #'radio://0/80/2M/E7E7E7E7E1': 'cf7'

# QTM IP (known to work in your setup)
QTM_IP = "192.168.0.105"

# The name of the rigid body in QTM that represents the Crazyflie
rigid_body_name = r'cf9'

# True: send position and orientation; False: send position only
send_full_pose = False

# When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90 deg.
orientation_std_dev = 8.0e-3

# Euler conversion settings for 6deuler → rotation matrix
EULER_ORDER = "ZYX"   # yaw(Z), pitch(Y), roll(X) typical for aerospace; adjust if needed
EULER_DEGREES = False  # QTM Euler is typically degrees
# ----------------------------------

# Duration,x^0,...,yaw^7
figure8 = [
    [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493, -0.000000, 0.000000, -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049, -0.445604, -0.684403, 0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514, 0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722, -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627, 0.450742, -0.385534, -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130, 0.001593, -1.031457, 0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354, -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682, 0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294, 0.288570, 0.873350, -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567, 0.447039, -0.532729, -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
]


class QtmWrapper(Thread):
    """
    Connects to QTM at QTM_IP, streams 6DoF data, and invokes self.on_pose([x,y,z,3x3rot]).
    Prefers '6deuler' for robustness; falls back to '6d'.
    """
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
        if not ok:
            return
        try:
            while self._stay_open:
                await asyncio.sleep(1)
        finally:
            await self._close()

    async def _connect(self) -> bool:
        print(f'[QtmWrapper] Connecting to QTM on {self.qtm_ip}')
        self.connection = await qtm_rt.connect(self.qtm_ip)
        if self.connection is None:
            print('[QtmWrapper] ERROR: Failed to connect to QTM.')
            return False

        # Parse 6d labels so we can index the requested rigid body
        try:
            params = await self.connection.get_parameters(parameters=['6d'])
            xml = ET.fromstring(params)
            self.qtm_6DoF_labels = [label.text.strip() for label in xml.findall('*/Body/Name')]
            if self.body_name not in self.qtm_6DoF_labels:
                print(f"[QtmWrapper] WARNING: Body '{self.body_name}' not found. Available: {self.qtm_6DoF_labels}")
        except Exception as e:
            print(f"[QtmWrapper] WARNING: Could not parse 6d labels ({e}); defaulting to index 0.")
            self.qtm_6DoF_labels = []

        # Try 6deuler; if not supported, fall back to 6d
        try:
            await self.connection.stream_frames(
                frames='allframes',
                components=['6deuler'],
                on_packet=self._on_packet_6deuler
            )
            print("[QtmWrapper] Streaming '6deuler'.")
        except QRTCommandException:
            print("[QtmWrapper] '6deuler' not supported. Falling back to '6d'.")
            await self.connection.stream_frames(
                frames='allframes',
                components=['6d'],
                on_packet=self._on_packet_6d
            )
            print("[QtmWrapper] Streaming '6d'.")
        return True

    def _body_index(self) -> int:
        if self.qtm_6DoF_labels and self.body_name in self.qtm_6DoF_labels:
            return self.qtm_6DoF_labels.index(self.body_name)
        return 0  # default to first body

    # ----- Packet handlers -----

    def _on_packet_6deuler(self, packet):
        if not hasattr(packet, 'get_6d_euler'):
            return
        header, bodies = packet.get_6d_euler()
        if not bodies:
            return

        idx = self._body_index()
        if idx >= len(bodies):
            return

        pos_mm, euler = bodies[idx]  # euler typically [roll, pitch, yaw] in degrees
        if pos_mm is None or any(math.isnan(v) for v in pos_mm):
            return

        x = pos_mm[0] / 1000.0
        y = pos_mm[1] / 1000.0
        z = pos_mm[2] / 1000.0

        # Build rotation from Euler (fallback to identity if something is off)
        rot3 = None
        if euler is not None and not any(math.isnan(v) for v in euler):
            roll, pitch, yaw = euler
            try:
                if EULER_ORDER.upper() == 'ZYX':
                    # QTM gives [roll, pitch, yaw]; map to yaw-pitch-roll for ZYX composition
                    rot3 = Rotation.from_euler('ZYX', [yaw, pitch, roll], degrees=EULER_DEGREES).as_matrix()
                else:
                    rot3 = Rotation.from_euler(EULER_ORDER, [roll, pitch, yaw], degrees=EULER_DEGREES).as_matrix()
            except Exception:
                rot3 = None

        if rot3 is None:
            rot3 = [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ]

        if self.on_pose:
            self.on_pose([x, y, z, rot3])

    def _on_packet_6d(self, packet):
        header, bodies = packet.get_6d()
        if not bodies:
            return

        idx = self._body_index()
        if idx >= len(bodies):
            return

        temp_cf_pos = bodies[idx]
        pos_mm = temp_cf_pos[0]
        rot = temp_cf_pos[1]

        if pos_mm is None or any(math.isnan(v) for v in pos_mm):
            return

        x = pos_mm[0] / 1000.0
        y = pos_mm[1] / 1000.0
        z = pos_mm[2] / 1000.0

        # Keep your original reshaping convention (works with your CF setup)
        r = rot.matrix  # flat list length 9
        rot3 = [
            [r[0], r[3], r[6]],
            [r[1], r[4], r[7]],
            [r[2], r[5], r[8]],
        ]

        if self.on_pose:
            self.on_pose([x, y, z, rot3])

    async def _close(self):
        try:
            await self.connection.stream_frames_stop()
        except Exception:
            pass
        if self.connection:
            self.connection.disconnect()
        print("[QtmWrapper] Disconnected from QTM.")


def _sqrt(a):
    """
    There might be rounding errors making 'a' slightly negative.
    Make sure we don't throw an exception.
    """
    if a < 0.0:
        return 0.0
    return math.sqrt(a)


def send_extpose_rot_matrix(cf, x, y, z, rot):
    """
    Send the current Crazyflie X, Y, Z position and attitude as a (3x3)
    rotation matrix. This is going to be forwarded to the Crazyflie's
    position estimator.
    """
    # Even if send_full_pose=False, we can compute quat safely; it will be ignored
    try:
        quat = Rotation.from_matrix(rot).as_quat()
    except Exception:
        quat = [0.0, 0.0, 0.0, 1.0]

    if send_full_pose:
        cf.extpos.send_extpose(x, y, z, quat[0], quat[1], quat[2], quat[3])
    else:
        cf.extpos.send_extpos(x, y, z)


def adjust_orientation_sensitivity(cf):
    cf.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)


def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')
    # Set the std deviation for the quaternion data pushed into the kalman filter.
    cf.param.set_value('locSrv.extQuatStdDev', 0.06)


def activate_mellinger_controller(cf):
    cf.param.set_value('stabilizer.controller', '2')


def upload_trajectory(cf, trajectory_id, trajectory):
    trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
    trajectory_mem.trajectory = []

    total_duration = 0
    for row in trajectory:
        duration = row[0]
        x = Poly4D.Poly(row[1:9])
        y = Poly4D.Poly(row[9:17])
        z = Poly4D.Poly(row[17:25])
        yaw = Poly4D.Poly(row[25:33])
        trajectory_mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
        total_duration += duration

    trajectory_mem.write_data_sync()
    cf.high_level_commander.define_trajectory(trajectory_id, 0, len(trajectory_mem.trajectory))
    return total_duration


def run_sequence(cf, trajectory_id, duration):
    commander = cf.high_level_commander

    commander.takeoff(1.0, 2.0)
    time.sleep(3.0)
    relative = True
    commander.start_trajectory(trajectory_id, 1.0, relative)
    time.sleep(duration)
    commander.land(0.0, 2.0)
    time.sleep(2)
    commander.stop()


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # Connect to QTM
    qtm_wrapper = QtmWrapper(rigid_body_name, qtm_ip=QTM_IP)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        trajectory_id = 1

        # Set up a callback to handle data from QTM
        qtm_wrapper.on_pose = lambda pose: send_extpose_rot_matrix(
            cf, pose[0], pose[1], pose[2], pose[3])

        adjust_orientation_sensitivity(cf)
        activate_kalman_estimator(cf)
        # activate_mellinger_controller(cf)
        duration = upload_trajectory(cf, trajectory_id, figure8)
        print('The sequence is {:.1f} seconds long'.format(duration))
        reset_estimator(cf)

        # Arm the Crazyflie
        cf.platform.send_arming_request(True)
        time.sleep(1.0)

        run_sequence(cf, trajectory_id, duration)

    qtm_wrapper.close()
#
#
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
# from cflib.crazyflie.mem import MemoryElement
# from cflib.crazyflie.mem import Poly4D
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.utils import uri_helper
# from cflib.utils.reset_estimator import reset_estimator
#
# # ------------- CONFIG -------------
# # URI to the Crazyflie to connect to
# uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
#
# # QTM IP (known to work in your setup)
# QTM_IP = "192.168.0.105"
#
# # The name of the rigid body in QTM that represents the Crazyflie
# rigid_body_name = r'cf_1'
#
# # True: send position and orientation; False: send position only
# send_full_pose = False
#
# # When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90 deg.
# orientation_std_dev = 8.0e-3
#
# # Euler conversion settings for 6deuler → rotation matrix
# EULER_ORDER = "ZYX"   # yaw(Z), pitch(Y), roll(X) typical for aerospace; adjust if needed
# EULER_DEGREES = True  # QTM Euler is typically degrees
# # ----------------------------------
#
# # Duration,x^0,...,yaw^7
# # [MODIFIED] This is the new trajectory definition. It consists of two segments:
# # 1. Move 1m in the x-direction.
# # 2. Move -1m in the x-direction (return to origin).
# # The coefficients are for a 4D polynomial, with the first value being the duration.
# new_trajectory = [
#     [3.0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
#     [3.0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
# ]
#
#
# class QtmWrapper(Thread):
#     """
#     Connects to QTM at QTM_IP, streams 6DoF data, and invokes self.on_pose([x,y,z,3x3rot]).
#     Prefers '6deuler' for robustness; falls back to '6d'.
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
#         # Parse 6d labels so we can index the requested rigid body
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
#         # Try 6deuler; if not supported, fall back to 6d
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
#         return 0  # default to first body
#
#     # ----- Packet handlers -----
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
#         pos_mm, euler = bodies[idx]  # euler typically [roll, pitch, yaw] in degrees
#         if pos_mm is None or any(math.isnan(v) for v in pos_mm):
#             return
#
#         x = pos_mm[0] / 1000.0
#         y = pos_mm[1] / 1000.0
#         z = pos_mm[2] / 1000.0
#
#         # Build rotation from Euler (fallback to identity if something is off)
#         rot3 = None
#         if euler is not None and not any(math.isnan(v) for v in euler):
#             roll, pitch, yaw = euler
#             try:
#                 if EULER_ORDER.upper() == 'ZYX':
#                     # QTM gives [roll, pitch, yaw]; map to yaw-pitch-roll for ZYX composition
#                     rot3 = Rotation.from_euler('ZYX', [yaw, pitch, roll], degrees=EULER_DEGREES).as_matrix()
#                 else:
#                     rot3 = Rotation.from_euler(EULER_ORDER, [roll, pitch, yaw], degrees=EULER_DEGREES).as_matrix()
#             except Exception:
#                 rot3 = None
#
#         if rot3 is None:
#             rot3 = [
#                 [1.0, 0.0, 0.0],
#                 [0.0, 1.0, 0.0],
#                 [0.0, 0.0, 1.0],
#             ]
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
#         temp_cf_pos = bodies[idx]
#         pos_mm = temp_cf_pos[0]
#         rot = temp_cf_pos[1]
#
#         if pos_mm is None or any(math.isnan(v) for v in pos_mm):
#             return
#
#         x = pos_mm[0] / 1000.0
#         y = pos_mm[1] / 1000.0
#         z = pos_mm[2] / 1000.0
#
#         # Keep your original reshaping convention (works with your CF setup)
#         r = rot.matrix  # flat list length 9
#         rot3 = [
#             [r[0], r[3], r[6]],
#             [r[1], r[4], r[7]],
#             [r[2], r[5], r[8]],
#         ]
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
#
#
# def _sqrt(a):
#     """
#     There might be rounding errors making 'a' slightly negative.
#     Make sure we don't throw an exception.
#     """
#     if a < 0.0:
#         return 0.0
#     return math.sqrt(a)
#
#
# def send_extpose_rot_matrix(cf, x, y, z, rot):
#     """
#     Send the current Crazyflie X, Y, Z position and attitude as a (3x3)
#     rotation matrix. This is going to be forwarded to the Crazyflie's
#     position estimator.
#     """
#     # Even if send_full_pose=False, we can compute quat safely; it will be ignored
#     try:
#         quat = Rotation.from_matrix(rot).as_quat()
#     except Exception:
#         quat = [0.0, 0.0, 0.0, 1.0]
#
#     if send_full_pose:
#         cf.extpos.send_extpose(x, y, z, quat[0], quat[1], quat[2], quat[3])
#     else:
#         cf.extpos.send_extpos(x, y, z)
#
#
# def adjust_orientation_sensitivity(cf):
#     cf.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)
#
#
# def activate_kalman_estimator(cf):
#     cf.param.set_value('stabilizer.estimator', '2')
#     # Set the std deviation for the quaternion data pushed into the kalman filter.
#     cf.param.set_value('locSrv.extQuatStdDev', 0.06)
#
#
# def activate_mellinger_controller(cf):
#     cf.param.set_value('stabilizer.controller', '2')
#
#
# def upload_trajectory(cf, trajectory_id, trajectory):
#     trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
#     trajectory_mem.trajectory = []
#
#     total_duration = 0
#     for row in trajectory:
#         duration = row[0]
#         x = Poly4D.Poly(row[1:9])
#         y = Poly4D.Poly(row[9:17])
#         z = Poly4D.Poly(row[17:25])
#         yaw = Poly4D.Poly(row[25:33])
#         trajectory_mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
#         total_duration += duration
#
#     trajectory_mem.write_data_sync()
#     cf.high_level_commander.define_trajectory(trajectory_id, 0, len(trajectory_mem.trajectory))
#     return total_duration
#
#
# def run_sequence(cf, trajectory_id, duration):
#     commander = cf.high_level_commander
#
#     # [MODIFIED] Set the takeoff height to 2.0 meters.
#     commander.takeoff(2.0, 2.0)
#     time.sleep(3.0)
#     relative = True
#     commander.start_trajectory(trajectory_id, 1.0, relative)
#     time.sleep(duration)
#     # [MODIFIED] Set the landing height to 0.0 meters.
#     commander.land(0.0, 2.0)
#     time.sleep(2)
#     commander.stop()
#
#
# if __name__ == '__main__':
#     cflib.crtp.init_drivers()
#
#     # Connect to QTM
#     qtm_wrapper = QtmWrapper(rigid_body_name, qtm_ip=QTM_IP)
#
#     with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
#         cf = scf.cf
#         trajectory_id = 1
#
#         # Set up a callback to handle data from QTM
#         qtm_wrapper.on_pose = lambda pose: send_extpose_rot_matrix(
#             cf, pose[0], pose[1], pose[2], pose[3])
#
#         adjust_orientation_sensitivity(cf)
#         activate_kalman_estimator(cf)
#         # activate_mellinger_controller(cf)
#         # [MODIFIED] Use the new_trajectory instead of figure8.
#         duration = upload_trajectory(cf, trajectory_id, new_trajectory)
#         print('The sequence is {:.1f} seconds long'.format(duration))
#         reset_estimator(cf)
#
#         # Arm the Crazyflie
#         cf.platform.send_arming_request(True)
#         time.sleep(1.0)
#
#         run_sequence(cf, trajectory_id, duration)
#
#     qtm_wrapper.close()


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
# from cflib.crazyflie.mem import MemoryElement
# from cflib.crazyflie.mem import Poly4D
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.utils import uri_helper
# from cflib.utils.reset_estimator import reset_estimator
#
# # ------------- CONFIG -------------
# # URI to the Crazyflie to connect to
# uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
#
# # QTM IP (known to work in your setup)
# QTM_IP = "192.168.0.105"
#
# # The name of the rigid body in QTM that represents the Crazyflie
# rigid_body_name = r'crzfly'
#
# # True: send position and orientation; False: send position only
# send_full_pose = False
#
# # When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90 deg.
# orientation_std_dev = 8.0e-3
#
# # Euler conversion settings for 6deuler → rotation matrix
# EULER_ORDER = "ZYX"  # yaw(Z), pitch(Y), roll(X) typical for aerospace; adjust if needed
# EULER_DEGREES = True  # QTM Euler is typically degrees
#
#
# # ----------------------------------
#
# # [REMOVED] The figure8 and new_trajectory variables are no longer needed for this flight sequence.
#
# class QtmWrapper(Thread):
#     """
#     Connects to QTM at QTM_IP, streams 6DoF data, and invokes self.on_pose([x,y,z,3x3rot]).
#     Prefers '6deuler' for robustness; falls back to '6d'.
#     """
#
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
#         # Parse 6d labels so we can index the requested rigid body
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
#         # Try 6deuler; if not supported, fall back to 6d
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
#         return 0  # default to first body
#
#     # ----- Packet handlers -----
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
#         pos_mm, euler = bodies[idx]  # euler typically [roll, pitch, yaw] in degrees
#         if pos_mm is None or any(math.isnan(v) for v in pos_mm):
#             return
#
#         x = pos_mm[0] / 1000.0
#         y = pos_mm[1] / 1000.0
#         z = pos_mm[2] / 1000.0
#
#         # Build rotation from Euler (fallback to identity if something is off)
#         rot3 = None
#         if euler is not None and not any(math.isnan(v) for v in euler):
#             roll, pitch, yaw = euler
#             try:
#                 if EULER_ORDER.upper() == 'ZYX':
#                     # QTM gives [roll, pitch, yaw]; map to yaw-pitch-roll for ZYX composition
#                     rot3 = Rotation.from_euler('ZYX', [yaw, pitch, roll], degrees=EULER_DEGREES).as_matrix()
#                 else:
#                     rot3 = Rotation.from_euler(EULER_ORDER, [roll, pitch, yaw], degrees=EULER_DEGREES).as_matrix()
#             except Exception:
#                 rot3 = None
#
#         if rot3 is None:
#             rot3 = [
#                 [1.0, 0.0, 0.0],
#                 [0.0, 1.0, 0.0],
#                 [0.0, 0.0, 1.0],
#             ]
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
#         temp_cf_pos = bodies[idx]
#         pos_mm = temp_cf_pos[0]
#         rot = temp_cf_pos[1]
#
#         if pos_mm is None or any(math.isnan(v) for v in pos_mm):
#             return
#
#         x = pos_mm[0] / 1000.0
#         y = pos_mm[1] / 1000.0
#         z = pos_mm[2] / 1000.0
#
#         # Keep your original reshaping convention (works with your CF setup)
#         r = rot.matrix  # flat list length 9
#         rot3 = [
#             [r[0], r[3], r[6]],
#             [r[1], r[4], r[7]],
#             [r[2], r[5], r[8]],
#         ]
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
#
#
# def _sqrt(a):
#     """
#     There might be rounding errors making 'a' slightly negative.
#     Make sure we don't throw an exception.
#     """
#     if a < 0.0:
#         return 0.0
#     return math.sqrt(a)
#
#
# def send_extpose_rot_matrix(cf, x, y, z, rot):
#     """
#     Send the current Crazyflie X, Y, Z position and attitude as a (3x3)
#     rotation matrix. This is going to be forwarded to the Crazyflie's
#     position estimator.
#     """
#     # Even if send_full_pose=False, we can compute quat safely; it will be ignored
#     try:
#         quat = Rotation.from_matrix(rot).as_quat()
#     except Exception:
#         quat = [0.0, 0.0, 0.0, 1.0]
#
#     if send_full_pose:
#         cf.extpos.send_extpose(x, y, z, quat[0], quat[1], quat[2], quat[3])
#     else:
#         cf.extpos.send_extpos(x, y, z)
#
#
# def adjust_orientation_sensitivity(cf):
#     cf.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)
#
#
# def activate_kalman_estimator(cf):
#     cf.param.set_value('stabilizer.estimator', '2')
#     # Set the std deviation for the quaternion data pushed into the kalman filter.
#     cf.param.set_value('locSrv.extQuatStdDev', 0.06)
#
#
# def activate_mellinger_controller(cf):
#     cf.param.set_value('stabilizer.controller', '2')
#
#
# def upload_trajectory(cf, trajectory_id, trajectory):
#     trajectory_mem = cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
#     trajectory_mem.trajectory = []
#
#     total_duration = 0
#     for row in trajectory:
#         duration = row[0]
#         x = Poly4D.Poly(row[1:9])
#         y = Poly4D.Poly(row[9:17])
#         z = Poly4D.Poly(row[17:25])
#         yaw = Poly4D.Poly(row[25:33])
#         trajectory_mem.trajectory.append(Poly4D(duration, x, y, z, yaw))
#         total_duration += duration
#
#     trajectory_mem.write_data_sync()
#     cf.high_level_commander.define_trajectory(trajectory_id, 0, len(trajectory_mem.trajectory))
#     return total_duration
#
#
# def run_sequence(cf, trajectory_id, duration):
#     commander = cf.high_level_commander
#
#     # [MODIFIED] Takeoff to 2m, with a flight duration of 3 seconds.
#     commander.takeoff(1.0, 3.0)
#     time.sleep(3.5)  # Wait for takeoff to complete
#
#     # [MODIFIED] Go to the current position (hover) at 2m height for 4 seconds.
#     # The 'go_to' command with the same x,y,z is an XY lock command.
#     commander.go_to(0.0, 0.0, 1.0, 0.0, 4.0)  # x, y, z, yaw, duration
#     time.sleep(4.5)  # Wait for hover to complete
#
#     # [MODIFIED] Land at the current x,y position.
#     commander.land(0.0, 3.0)  # height, duration
#     time.sleep(3.5)  # Wait for landing to complete
#
#     commander.stop()
#
#
# if __name__ == '__main__':
#     cflib.crtp.init_drivers()
#
#     # Connect to QTM
#     qtm_wrapper = QtmWrapper(rigid_body_name, qtm_ip=QTM_IP)
#
#     with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
#         cf = scf.cf
#         trajectory_id = 1
#
#         # Set up a callback to handle data from QTM
#         qtm_wrapper.on_pose = lambda pose: send_extpose_rot_matrix(
#             cf, pose[0], pose[1], pose[2], pose[3])
#
#         adjust_orientation_sensitivity(cf)
#         activate_kalman_estimator(cf)
#         # activate_mellinger_controller(cf)
#
#         # [REMOVED] The upload_trajectory and duration logic are no longer needed.
#         # duration = upload_trajectory(cf, trajectory_id, figure8)
#         # print('The sequence is {:.1f} seconds long'.format(duration))
#
#         reset_estimator(cf)
#
#         # Arm the Crazyflie
#         cf.platform.send_arming_request(True)
#         time.sleep(1.0)
#
#         # [MODIFIED] Run the new sequence without a pre-uploaded trajectory.
#         run_sequence(cf, trajectory_id, 0)  # Duration argument is no longer used.
#
#     qtm_wrapper.close()