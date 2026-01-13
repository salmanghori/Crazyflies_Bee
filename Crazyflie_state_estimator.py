# import asyncio
# import math
# import time
# import xml.etree.cElementTree as ET
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
# # ------------- CONFIG -------------
# # URI to the Crazyflie to connect to
# URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E5')
#
# # QTM IP (known to work in your setup)
# QTM_IP = "192.168.0.105"
#
# # The name of the rigid body in QTM that represents the Crazyflie
# RIGID_BODY_NAME = r'cf5'
#
# # --- Flight Parameters ---
# # True: send position and orientation; False: send position only
# # It is SAFER to start with False until you have confirmed your coordinate systems match perfectly.
# SEND_FULL_POSE = False
# TAKEOFF_HEIGHT = 0.5  # meters
# TAKEOFF_DURATION = 2.0  # seconds
# LAND_DURATION = 3.0  # seconds
#
# # --- Estimator & Controller Configuration ---
# # See explanation below for these values
# # How much we trust the QTM position data. Lower value = more trust.
# # A good starting point is the measured static noise of your QTM setup.
# # Units are meters. 0.001 = 1mm standard deviation.
# EXTPOS_STD_DEV = 0.002
#
# # How much we trust the QTM orientation data. Lower value = more trust.
# # This is only used if SEND_FULL_POSE is True.
# EXTATT_STD_DEV_RAD = 0.05  # radians, a reasonable starting point
#
# # --- QTM Data Handling ---
# # Rate at which we send pose packets to the Crazyflie (in Hz)
# # The radio can handle ~100Hz. Higher rates from QTM are fine, but we will throttle.
# SEND_RATE_HZ = 100
# # If we don't receive a QTM packet for this duration, we assume tracking is lost
# QTM_TIMEOUT_S = 0.5
#
# # --- QTM Coordinate System / Euler Conversion ---
# # This MUST match your physical setup and QTM's axis definitions.
# # Crazyflie default: X-Forward, Y-Left, Z-Up
# EULER_ORDER = "ZYX"
# EULER_DEGREES = True  # QTM Euler is typically degrees
# # ----------------------------------
#
# # Your trajectory data remains the same
# figure8 = [
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
#
# class QtmWrapper(Thread):
#     # This class is mostly unchanged, it's already well-written.
#     # We just ensure it's robust.
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
#         self.start()
#
#     def close(self):
#         self._stay_open = False
#         self.join(timeout=1.0)
#
#     def run(self):
#         asyncio.run(self._life_cycle())
#
#     async def _life_cycle(self):
#         print(f'[QtmWrapper] Starting connection to QTM on {self.qtm_ip}')
#         while self._stay_open:
#             try:
#                 self.connection = await qtm_rt.connect(self.qtm_ip, version='1.22')
#                 if self.connection:
#                     print('[QtmWrapper] Connected to QTM.')
#                     await self._main_loop()
#             except asyncio.TimeoutError:
#                 print(f'[QtmWrapper] Timeout while connecting to {self.qtm_ip}. Retrying in 2s.')
#             except ConnectionRefusedError:
#                 print(f'[QtmWrapper] Connection refused by {self.qtm_ip}. Is QTM running? Retrying in 2s.')
#             except Exception as e:
#                 print(f'[QtmWrapper] An unexpected error occurred: {e}. Retrying in 2s.')
#
#             if self.connection and not self.connection.is_connected:
#                 print('[QtmWrapper] Disconnected. Reconnecting...')
#             await asyncio.sleep(2)
#         print("[QtmWrapper] Lifecycle ended.")
#
#     async def _main_loop(self):
#         await self._discover_bodies()
#         await self._stream_data()
#         while self._stay_open and self.connection and self.connection.is_connected:
#             await asyncio.sleep(0.1)
#         print("[QtmWrapper] Main loop finished.")
#
#     async def _discover_bodies(self):
#         try:
#             params = await self.connection.get_parameters(parameters=['6d'])
#             xml = ET.fromstring(params)
#             self.qtm_6DoF_labels = [label.text.strip() for label in xml.findall('*/Body/Name')]
#             print(f"[QtmWrapper] Available bodies: {self.qtm_6DoF_labels}")
#             if self.body_name not in self.qtm_6DoF_labels:
#                 print(f"[QtmWrapper] WARNING: Body '{self.body_name}' not found!")
#         except Exception as e:
#             print(f"[QtmWrapper] WARNING: Could not parse 6d labels ({e}).")
#
#     async def _stream_data(self):
#         components = ['6deuler' if SEND_FULL_POSE else '6d']
#         print(f"[QtmWrapper] Streaming components: {components}")
#         try:
#             await self.connection.stream_frames(
#                 frames='allframes',
#                 components=components,
#                 on_packet=self._on_packet
#             )
#         except QRTCommandException as e:
#             print(f"[QtmWrapper] Error starting stream: {e}. Trying fallback.")
#             # Fallback for older QTM versions or different configurations
#             components = ['6d']
#             await self.connection.stream_frames(
#                 frames='allframes', components=components, on_packet=self._on_packet)
#
#     def _body_index(self) -> int:
#         if self.qtm_6DoF_labels and self.body_name in self.qtm_6DoF_labels:
#             return self.qtm_6DoF_labels.index(self.body_name)
#         if not self.qtm_6DoF_labels:  # If discovery failed, assume it's the first one
#             return 0
#         return -1  # Body not found
#
#     def _on_packet(self, packet):
#         body_index = self._body_index()
#         if body_index == -1: return
#
#         header, bodies = (packet.get_6d_euler() if SEND_FULL_POSE else packet.get_6d())
#
#         if bodies is None or body_index >= len(bodies):
#             return  # No data for our body in this packet
#
#         pos, rot_data = bodies[body_index]
#         if pos is None or any(math.isnan(v) for v in pos):
#             return  # Invalid position data
#
#         x = pos[0] / 1000.0
#         y = pos[1] / 1000.0
#         z = pos[2] / 1000.0
#
#         rot3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]  # Identity matrix
#
#         if SEND_FULL_POSE and rot_data is not None:
#             # Handling 6d Euler data
#             if isinstance(rot_data, tuple) and not any(math.isnan(v) for v in rot_data):
#                 roll, pitch, yaw = rot_data
#                 try:
#                     rot3 = Rotation.from_euler(EULER_ORDER, [yaw, pitch, roll], degrees=EULER_DEGREES).as_matrix()
#                 except Exception as e:
#                     print(f"Euler conversion error: {e}")  # Keep identity
#             # Handling 6d Rotation Matrix data
#             elif hasattr(rot_data, 'matrix'):
#                 r = rot_data.matrix
#                 rot3 = [[r[0], r[3], r[6]], [r[1], r[4], r[7]], [r[2], r[5], r[8]]]
#
#         if self.on_pose:
#             self.on_pose([x, y, z, rot3])
#
#
# class CrazyflieManager:
#     """Manages Crazyflie connection, configuration, and flight sequences."""
#
#     def __init__(self, uri):
#         self._uri = uri
#         self._cf = None
#         self._scf = None
#         self.is_connected = False
#
#         self.last_pose_timestamp = 0
#         self.is_tracking_lost = True
#
#     def __enter__(self):
#         self._scf = SyncCrazyflie(self._uri, cf=Crazyflie(rw_cache='./cache'))
#         self._scf.open_link()
#         self._cf = self._scf.cf
#         self.is_connected = True
#         return self
#
#     def __exit__(self, exc_type, exc_val, exc_tb):
#         if self._cf and self.is_connected:
#             try:
#                 # Issue a stop command on exit to cut motors
#                 self._cf.commander.send_stop_setpoint()
#             except Exception as e:
#                 print(f"Error sending stop setpoint on exit: {e}")
#         if self._scf:
#             self._scf.close_link()
#         self.is_connected = False
#
#     def get_cf(self):
#         return self._cf
#
#     def set_estimator_and_controller(self):
#         print("Configuring onboard estimator and controller...")
#         # Activate the EKF (estimator type 2)
#         self._cf.param.set_value('stabilizer.estimator', '2')
#
#         # Set the standard deviation for the external position measurement.
#         self._cf.param.set_value('extpos.stdDev', EXTPOS_STD_DEV)
#
#         if SEND_FULL_POSE:
#             # Set the standard deviation for the external attitude measurement.
#             self._cf.param.set_value('locSrv.extQuatStdDev', EXTATT_STD_DEV_RAD)
#
#         # Activate the Mellinger controller (controller type 2) for trajectory tracking
#         self._cf.param.set_value('stabilizer.controller', '2')
#         print("Configuration complete.")
#
#     def upload_trajectory(self, trajectory_id, trajectory_data):
#         print(f"Uploading trajectory {trajectory_id}...")
#         traj_mem = self._cf.mem.get_mems(MemoryElement.TYPE_TRAJ)[0]
#
#         total_duration = 0
#         pieces = []
#         for row in trajectory_data:
#             duration = row[0]
#             x = Poly4D.Poly(row[1:9])
#             y = Poly4D.Poly(row[9:17])
#             z = Poly4D.Poly(row[17:25])
#             yaw = Poly4D.Poly(row[25:33])
#             pieces.append(Poly4D(duration, x, y, z, yaw))
#             total_duration += duration
#
#         traj_mem.upload_polynomials(pieces, 0)
#         self._cf.high_level_commander.define_trajectory(trajectory_id, 0, len(pieces))
#         print(f"Upload complete. Trajectory duration: {total_duration:.2f}s")
#         return total_duration
#
#     def send_external_pose(self, x, y, z, rot_matrix):
#         self.last_pose_timestamp = time.time()
#         if self.is_tracking_lost:
#             print("[REACQUIRED] QTM tracking.")
#             self.is_tracking_lost = False
#
#         quat = Rotation.from_matrix(rot_matrix).as_quat()
#         if SEND_FULL_POSE:
#             self._cf.extpos.send_extpose(x, y, z, quat[0], quat[1], quat[2], quat[3])
#         else:
#             self._cf.extpos.send_extpos(x, y, z)
#
#     def check_tracking_timeout(self):
#         if time.time() - self.last_pose_timestamp > QTM_TIMEOUT_S:
#             if not self.is_tracking_lost:
#                 print("[CRITICAL] QTM tracking lost!")
#                 self.is_tracking_lost = True
#         return self.is_tracking_lost
#
#     def run_flight_sequence(self, trajectory_id, duration):
#         commander = self._cf.high_level_commander
#         print("--- Starting Flight Sequence ---")
#
#         print("Waiting for stable QTM signal before takeoff...")
#         while self.is_tracking_lost:
#             if self.check_tracking_timeout():
#                 time.sleep(0.1)
#         print("QTM signal stable. Ready to fly.")
#
#         print(f"Taking off to {TAKEOFF_HEIGHT}m...")
#         commander.takeoff(TAKEOFF_HEIGHT, TAKEOFF_DURATION)
#         time.sleep(TAKEOFF_DURATION + 1.0)
#
#         print(f"Starting trajectory {trajectory_id}...")
#         commander.start_trajectory(trajectory_id, 1.0, relative=True)
#         time.sleep(duration)
#
#         print("Trajectory finished. Landing...")
#         commander.land(0.0, LAND_DURATION)
#         time.sleep(LAND_DURATION)
#
#         print("Stopping motors.")
#         commander.stop()
#         print("--- Flight Sequence Complete ---")
#
#
# def main():
#     cflib.crtp.init_drivers()
#     stop_event = Event()
#
#     qtm_thread = QtmWrapper(RIGID_BODY_NAME, qtm_ip=QTM_IP)
#
#     try:
#         with CrazyflieManager(URI) as cf_manager:
#             cf = cf_manager.get_cf()
#
#             # --- Setup Phase ---
#             cf_manager.set_estimator_and_controller()
#
#             # Reset the estimator to ensure a good starting state
#             reset_estimator(cf)
#
#             trajectory_id = 1
#             duration = cf_manager.upload_trajectory(trajectory_id, figure8)
#
#             # --- Throttled Pose Sending Loop ---
#             def pose_callback(pose_data):
#                 # This function is called from the QTM thread at high frequency
#                 # We store the latest pose in a shared variable
#                 nonlocal latest_pose
#                 latest_pose = pose_data
#
#             latest_pose = None
#             qtm_thread.on_pose = pose_callback
#
#             def send_pose_loop():
#                 # This function runs in the main thread and sends data at a fixed rate
#                 while not stop_event.is_set() and cf_manager.is_connected:
#                     if latest_pose:
#                         cf_manager.send_external_pose(*latest_pose)
#                     time.sleep(1.0 / SEND_RATE_HZ)
#
#             # Start the sender loop in a separate thread
#             sender_thread = Thread(target=send_pose_loop, daemon=True)
#             sender_thread.start()
#
#             # --- Flight Execution ---
#             cf_manager.run_flight_sequence(trajectory_id, duration)
#
#     except KeyboardInterrupt:
#         print("Keyboard interrupt received. Landing...")
#     except Exception as e:
#         print(f"\nAn error occurred: {e}")
#     finally:
#         print("Shutting down...")
#         stop_event.set()
#         qtm_thread.close()
#
#
# if __name__ == '__main__':
#     main()






import asyncio
import math
import time
import xml.etree.cElementTree as ET
from threading import Thread, Event

import qtm_rt
from qtm_rt import QRTCommandException
from scipy.spatial.transform import Rotation

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.mem import MemoryElement, Poly4D
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

# ------------- CONFIG -------------
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

QTM_IP = "192.168.0.105"
RIGID_BODY_NAME = r'cf8'

# --- Flight Parameters ---
SEND_FULL_POSE = False
TAKEOFF_HEIGHT = 0.5  # m
TAKEOFF_DURATION = 2.0  # s
LAND_DURATION = 3.0  # s

# --- Estimator & Controller Configuration ---
EXTPOS_STD_DEV = 0.002       # m (1–5 mm is typical for mocap)
EXTATT_STD_DEV_RAD = 0.05    # rad (used only if SEND_FULL_POSE=True)

# --- QTM Data Handling ---
SEND_RATE_HZ = 100           # throttle to Crazyradio capacity
QTM_TIMEOUT_S = 0.5          # declare loss if no pose for this long

# --- QTM Euler conversion (only if sending full pose) ---
EULER_ORDER = "ZYX"
EULER_DEGREES = True
# ----------------------------------

figure8 = [
    [1.050000, 0.000000, -0.000000, 0.000000, -0.000000, 0.830443, -0.276140, -0.384219, 0.180493, -0.000000, 0.000000,
     -0.000000, 0.000000, -1.356107, 0.688430, 0.587426, -0.329106, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.710000, 0.396058, 0.918033, 0.128965, -0.773546, 0.339704, 0.034310, -0.026417, -0.030049, -0.445604, -0.684403,
     0.888433, 1.493630, -1.361618, -0.139316, 0.158875, 0.095799, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.620000, 0.922409, 0.405715, -0.582968, -0.092188, -0.114670, 0.101046, 0.075834, -0.037926, -0.291165, 0.967514,
     0.421451, -1.086348, 0.545211, 0.030109, -0.050046, -0.068177, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.700000, 0.923174, -0.431533, -0.682975, 0.177173, 0.319468, -0.043852, -0.111269, 0.023166, 0.289869, 0.724722,
     -0.512011, -0.209623, -0.218710, 0.108797, 0.128756, -0.055461, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.560000, 0.405364, -0.834716, 0.158939, 0.288175, -0.373738, -0.054995, 0.036090, 0.078627, 0.450742, -0.385534,
     -0.954089, 0.128288, 0.442620, 0.055630, -0.060142, -0.076163, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.560000, 0.001062, -0.646270, -0.012560, -0.324065, 0.125327, 0.119738, 0.034567, -0.063130, 0.001593, -1.031457,
     0.015159, 0.820816, -0.152665, -0.130729, -0.045679, 0.080444, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.700000, -0.402804, -0.820508, -0.132914, 0.236278, 0.235164, -0.053551, -0.088687, 0.031253, -0.449354,
     -0.411507, 0.902946, 0.185335, -0.239125, -0.041696, 0.016857, 0.016709, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000],
    [0.620000, -0.921641, -0.464596, 0.661875, 0.286582, -0.228921, -0.051987, 0.004669, 0.038463, -0.292459, 0.777682,
     0.565788, -0.432472, -0.060568, -0.082048, -0.009439, 0.041158, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [0.710000, -0.923935, 0.447832, 0.627381, -0.259808, -0.042325, -0.032258, 0.001420, 0.005294, 0.288570, 0.873350,
     -0.515586, -0.730207, -0.026023, 0.288755, 0.215678, -0.148061, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
    [1.053185, -0.398611, 0.850510, -0.144007, -0.485368, -0.079781, 0.176330, 0.234482, -0.153567, 0.447039, -0.532729,
     -0.855023, 0.878509, 0.775168, -0.391051, -0.713519, 0.391628, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,
     0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000],
]


class QtmWrapper(Thread):
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
        self.join(timeout=1.0)

    def run(self):
        asyncio.run(self._life_cycle())

    async def _life_cycle(self):
        print(f'[QtmWrapper] Starting connection to QTM on {self.qtm_ip}')
        while self._stay_open:
            try:
                self.connection = await qtm_rt.connect(self.qtm_ip, version='1.22')
                if self.connection:
                    print('[QtmWrapper] Connected to QTM.')
                    await self._main_loop()
            except asyncio.TimeoutError:
                print(f'[QtmWrapper] Timeout while connecting to {self.qtm_ip}. Retrying in 2s.')
            except ConnectionRefusedError:
                print(f'[QtmWrapper] Connection refused by {self.qtm_ip}. Is QTM running? Retrying in 2s.')
            except Exception as e:
                print(f'[QtmWrapper] An unexpected error occurred: {e}. Retrying in 2s.')

            if self.connection and not self.connection.is_connected:
                print('[QtmWrapper] Disconnected. Reconnecting...')
            await asyncio.sleep(2)
        print("[QtmWrapper] Lifecycle ended.")

    async def _main_loop(self):
        await self._discover_bodies()
        await self._stream_data()
        while self._stay_open and self.connection and self.connection.is_connected:
            await asyncio.sleep(0.1)
        print("[QtmWrapper] Main loop finished.")

    async def _discover_bodies(self):
        try:
            params = await self.connection.get_parameters(parameters=['6d'])
            xml = ET.fromstring(params)
            self.qtm_6DoF_labels = [label.text.strip() for label in xml.findall('*/Body/Name')]
            print(f"[QtmWrapper] Available bodies: {self.qtm_6DoF_labels}")
            if self.body_name not in self.qtm_6DoF_labels:
                print(f"[QtmWrapper] WARNING: Body '{self.body_name}' not found!")
        except Exception as e:
            print(f"[QtmWrapper] WARNING: Could not parse 6d labels ({e}).")

    async def _stream_data(self):
        components = ['6deuler' if SEND_FULL_POSE else '6d']
        print(f"[QtmWrapper] Streaming components: {components}")
        try:
            await self.connection.stream_frames(
                frames='allframes',
                components=components,
                on_packet=self._on_packet
            )
        except QRTCommandException as e:
            print(f"[QtmWrapper] Error starting stream: {e}. Trying fallback.")
            components = ['6d']
            await self.connection.stream_frames(
                frames='allframes', components=components, on_packet=self._on_packet)

    def _body_index(self) -> int:
        if self.qtm_6DoF_labels and self.body_name in self.qtm_6DoF_labels:
            return self.qtm_6DoF_labels.index(self.body_name)
        if not self.qtm_6DoF_labels:
            return 0
        return -1

    def _on_packet(self, packet):
        body_index = self._body_index()
        if body_index == -1:
            return

        header, bodies = (packet.get_6d_euler() if SEND_FULL_POSE else packet.get_6d())
        if bodies is None or body_index >= len(bodies):
            return

        pos, rot_data = bodies[body_index]
        if pos is None or any(math.isnan(v) for v in pos):
            return

        x = pos[0] / 1000.0
        y = pos[1] / 1000.0
        z = pos[2] / 1000.0

        rot3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        if SEND_FULL_POSE and rot_data is not None:
            if isinstance(rot_data, tuple) and not any(math.isnan(v) for v in rot_data):
                roll, pitch, yaw = rot_data
                try:
                    rot3 = Rotation.from_euler(EULER_ORDER, [yaw, pitch, roll], degrees=EULER_DEGREES).as_matrix()
                except Exception as e:
                    print(f"Euler conversion error: {e}")
            elif hasattr(rot_data, 'matrix'):
                r = rot_data.matrix
                rot3 = [[r[0], r[3], r[6]], [r[1], r[4], r[7]], [r[2], r[5], r[8]]]

        if self.on_pose:
            self.on_pose([x, y, z, rot3])


class CrazyflieManager:
    """Manages Crazyflie connection, configuration, and flight sequences."""

    def __init__(self, uri):
        self._uri = uri
        self._cf = None
        self._scf = None
        self.is_connected = False

        self.last_pose_timestamp = 0
        self.is_tracking_lost = True

    def __enter__(self):
        self._scf = SyncCrazyflie(self._uri, cf=Crazyflie(rw_cache='./cache'))
        self._scf.open_link()
        self._cf = self._scf.cf
        self.is_connected = True
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self._cf and self.is_connected:
            try:
                self._cf.commander.send_stop_setpoint()
            except Exception as e:
                print(f"Error sending stop setpoint on exit: {e}")
        if self._scf:
            self._scf.close_link()
        self.is_connected = False

    def get_cf(self):
        return self._cf

    def _try_set(self, name, val):
        """Minimal, necessary param fallback setter."""
        try:
            self._cf.param.set_value(name, str(val))
            print(f"Param set: {name}={val}")
            return True
        except Exception:
            return False

    def set_estimator_and_controller(self):
        print("Configuring onboard estimator and controller...")

        # NECESSARY: enable HL commander for trajectories
        self._cf.param.set_value('commander.enHighLevel', '1')

        # EKF on
        self._cf.param.set_value('stabilizer.estimator', '2')

        # Position stddev (firmware names vary; try both)
        if not self._try_set('locSrv.extPosStdDev', EXTPOS_STD_DEV):
            self._try_set('extpos.stdDev', EXTPOS_STD_DEV)

        # Quaternion stddev (only if using full pose)
        if SEND_FULL_POSE:
            self._try_set('locSrv.extQuatStdDev', EXTATT_STD_DEV_RAD)

        # Mellinger for Poly4D
        self._cf.param.set_value('stabilizer.controller', '2')

        print("Configuration complete.")

    def upload_trajectory(self, trajectory_id, trajectory_data):
        print(f"Uploading trajectory {trajectory_id}...")

        # NECESSARY: wait for trajectory memory availability
        mems = []
        for _ in range(50):  # ~1 s
            mems = self._cf.mem.get_mems(MemoryElement.TYPE_TRAJ)
            if mems:
                break
            time.sleep(0.02)
        if not mems:
            raise RuntimeError("Trajectory memory not available")

        traj_mem = mems[0]

        total_duration = 0
        pieces = []
        for row in trajectory_data:
            duration = row[0]
            x = Poly4D.Poly(row[1:9])
            y = Poly4D.Poly(row[9:17])
            z = Poly4D.Poly(row[17:25])
            yaw = Poly4D.Poly(row[25:33])
            pieces.append(Poly4D(duration, x, y, z, yaw))
            total_duration += duration

        traj_mem.upload_polynomials(pieces, 0)
        self._cf.high_level_commander.define_trajectory(trajectory_id, 0, len(pieces))
        print(f"Upload complete. Trajectory duration: {total_duration:.2f}s")
        return total_duration

    def send_external_pose(self, x, y, z, rot_matrix):
        self.last_pose_timestamp = time.time()
        if self.is_tracking_lost:
            print("[REACQUIRED] QTM tracking.")
            self.is_tracking_lost = False

        if SEND_FULL_POSE:
            quat = Rotation.from_matrix(rot_matrix).as_quat()
            self._cf.extpos.send_extpose(x, y, z, quat[0], quat[1], quat[2], quat[3])
        else:
            self._cf.extpos.send_extpos(x, y, z)

    def check_tracking_timeout(self):
        # NECESSARY: safe action on QTM loss
        if time.time() - self.last_pose_timestamp > QTM_TIMEOUT_S:
            if not self.is_tracking_lost:
                print("[CRITICAL] QTM tracking lost! Stopping trajectory & landing.")
                try:
                    self._cf.high_level_commander.stop()
                    self._cf.high_level_commander.land(0.0, 2.0)
                except Exception:
                    pass
                self.is_tracking_lost = True
        return self.is_tracking_lost

    def run_flight_sequence(self, trajectory_id, duration):
        commander = self._cf.high_level_commander
        print("--- Starting Flight Sequence ---")

        # NECESSARY: ensure fresh QTM for >= 0.2s before takeoff
        print("Waiting for stable QTM (≥0.2 s)...")
        stable_since = None
        while True:
            now = time.time()
            if now - self.last_pose_timestamp < 0.02:  # a frame within 20 ms
                stable_since = stable_since or now
                if now - stable_since > 0.2:
                    break
            else:
                stable_since = None
            time.sleep(0.01)
        print("QTM signal stable. Ready to fly.")

        print(f"Taking off to {TAKEOFF_HEIGHT}m...")
        commander.takeoff(TAKEOFF_HEIGHT, TAKEOFF_DURATION)
        time.sleep(TAKEOFF_DURATION + 1.0)

        print(f"Starting trajectory {trajectory_id}...")
        commander.start_trajectory(trajectory_id, 1.0, relative=True)
        # While running, also monitor timeout to react immediately
        t0 = time.time()
        while time.time() - t0 < duration:
            self.check_tracking_timeout()
            time.sleep(0.01)

        print("Trajectory finished. Landing...")
        commander.land(0.0, LAND_DURATION)
        time.sleep(LAND_DURATION)

        print("Stopping motors.")
        commander.stop()
        print("--- Flight Sequence Complete ---")


def main():
    cflib.crtp.init_drivers()
    stop_event = Event()

    qtm_thread = QtmWrapper(RIGID_BODY_NAME, qtm_ip=QTM_IP)

    try:
        with CrazyflieManager(URI) as cf_manager:
            cf = cf_manager.get_cf()

            # --- Setup Phase ---
            cf_manager.set_estimator_and_controller()
            reset_estimator(cf)  # clean estimator start

            trajectory_id = 1
            duration = cf_manager.upload_trajectory(trajectory_id, figure8)

            # --- Throttled Pose Sending Loop ---
            latest_pose = {"data": None}

            def pose_callback(pose_data):
                latest_pose["data"] = pose_data

            qtm_thread.on_pose = pose_callback

            def send_pose_loop():
                next_t = time.time()
                while not stop_event.is_set() and cf_manager.is_connected:
                    # safe-land on loss
                    cf_manager.check_tracking_timeout()
                    if latest_pose["data"] and time.time() >= next_t:
                        cf_manager.send_external_pose(*latest_pose["data"])
                        next_t += 1.0 / SEND_RATE_HZ
                    time.sleep(0.001)

            sender_thread = Thread(target=send_pose_loop, daemon=True)
            sender_thread.start()

            # --- Flight Execution ---
            cf_manager.run_flight_sequence(trajectory_id, duration)

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Landing...")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        print("Shutting down...")
        stop_event.set()
        qtm_thread.close()


if __name__ == '__main__':
    main()
