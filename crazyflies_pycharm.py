# import cflib.crtp
#
# # Initialize the low-level drivers
# cflib.crtp.init_drivers()
#
# # Get a list of available Crazyflies
# uris = cflib.crtp.scan_interfaces()
#
# if uris:
#     print('Crazyflies found:')
#     for uri in uris:
#         print(f'  - {uri}')
# else:
#     print('No Crazyflies found.')


#
# import wmi
#
# # Connect to the WMI service
# c = wmi.WMI()
#
# # Query the Win32_USBControllerDevice class
# # This class represents a logical connection to a physical USB device.
# for usb in c.Win32_USBControllerDevice():
#     print(f"Device: {usb.Dependent}")

# import usb.core
# import usb.util
#
# # Find the Crazyradio PA using its Vendor and Product IDs
# dev = usb.core.find(idVendor=0x1915, idProduct=0x7777)
#
# # Check if the device was found
# if dev is None:
#     print("Crazyradio PA not found.")
# else:
#     print("Crazyradio PA found successfully!")
#     print(f"Device information: {dev}")
#     # You can get more info like its manufacturer string
#     print(f"Manufacturer: {usb.util.get_string(dev, dev.iManufacturer)}")
#     print(f"Product: {usb.util.get_string(dev, dev.iProduct)}")



# import cflib.crtp
#
# # Initialize the low-level drivers
# cflib.crtp.init_drivers()
#
# # Get a list of available Crazyflies
# uris = cflib.crtp.scan_interfaces()
#
# if uris:
#     print('Crazyflies found:')
#     for uri in uris:
#         print(f'  - {uri}')
# else:
#     print('No Crazyflies found.')


# import cflib.crtp
# import time
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.positioning.motion_commander import MotionCommander
# from cflib.crazyflie import Crazyflie
#
# # The URI for your Crazyflie, found in your previous output
# URI = 'radio://0/80/2M'
#
#
# def take_off_and_land():
#     # Initialize the low-level drivers
#     cflib.crtp.init_drivers()
#
#     print('Connecting to Crazyflie...')
#     with SyncCrazyflie(URI, cf=Crazyflie()) as scf:
#         print('Crazyflie connected! Taking off...')
#
#         with MotionCommander(scf) as mc:
#             # Command the Crazyflie to take off
#             mc.take_off(0.5)  # Take off to a height of 0.5 meters
#
#             # Hover in place for 3 seconds
#             print('Hovering...')
#             time.sleep(3)
#
#             # Command the Crazyflie to land
#             print('Landing...')
#             mc.land()
#
#         print('MotionCommander session ended.')
#
#     print('Disconnected.')
#
#
# if __name__ == '__main__':
#     take_off_and_land()


# import cflib.crtp
# import time
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.positioning.motion_commander import MotionCommander
# from cflib.crazyflie import Crazyflie
#
# # The URI for your Crazyflie
# URI = 'radio://0/80/2M'
#
#
# def take_off_and_land():
#     cflib.crtp.init_drivers()
#     print('Connecting to Crazyflie...')
#
#     # Give a short delay to ensure Crazyflie is ready
#     time.sleep(2)
#
#     with SyncCrazyflie(URI, cf=Crazyflie()) as scf:
#         print('Crazyflie connected! Taking off...')
#
#         with MotionCommander(scf) as mc:
#             # Command the Crazyflie to take off
#             mc.take_off(0.5)
#
#             # Hover in place for 3 seconds
#             print('Hovering...')
#             time.sleep(3)
#
#             # Command the Crazyflie to land
#             print('Landing...')
#             mc.land()
#
#         print('MotionCommander session ended.')
#
#     print('Disconnected.')
#
#
# if __name__ == '__main__':
#     take_off_and_land()

# =================================================================================================
# import cflib.crtp
# import time
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.crazyflie import Crazyflie
# from cflib.crazyflie.log import LogConfig
#
# # The URI for your Crazyflie
# URI = 'radio://0/80/2M'
#
# # Dictionary to hold the latest sensor data
# log_data = {}
#
#
# def log_callback(timestamp, data, logconf):
#     """Callback function for the logging data"""
#     global log_data
#     log_data.update(data)
#     print(
#         f"Timestamp: {timestamp} | Accel_x: {data.get('acc.x'):.2f} | Accel_y: {data.get('acc.y'):.2f} | Accel_z: {data.get('acc.z'):.2f}")
#     print(
#         f"            | Gyro_x:  {data.get('gyro.x'):.2f} | Gyro_y:  {data.get('gyro.y'):.2f} | Gyro_z:  {data.get('gyro.z'):.2f}")
#
#
# def start_logging():
#     cflib.crtp.init_drivers()
#     print('Connecting to Crazyflie...')
#
#     with SyncCrazyflie(URI, cf=Crazyflie()) as scf:
#         print('Crazyflie connected!')
#         cf = scf.cf
#
#         # Set up a logging configuration
#         log_conf = LogConfig(name='SensorLog', period_in_ms=100)
#         log_conf.add_variable('acc.x')
#         log_conf.add_variable('acc.y')
#         log_conf.add_variable('acc.z')
#         log_conf.add_variable('gyro.x')
#         log_conf.add_variable('gyro.y')
#         log_conf.add_variable('gyro.z')
#
#         try:
#             cf.log.add_config(log_conf)
#             log_conf.data_received_cb.add_callback(log_callback)
#             log_conf.start()
#
#             print("Logging sensor data for 10 seconds. Keep the Crazyflie still.")
#             time.sleep(60)
#
#             log_conf.stop()
#         except Exception as e:
#             print(f"Error starting logging: {e}")
#
#     print('Disconnected.')
#
# if __name__ == '__main__':
#     start_logging()
#===================================================================================================================


#====================================================================================================================
# ***********$$$$$$$$*********> all log variables (and, when available, their C types) that works across cflib versions
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = 'radio://0/80/2M'

def list_log_vars():
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie()) as scf:
        toc = scf.cf.log.toc

        # toc.toc is a dict: { group_name: [var_name, var_name, ...], ... }
        for group, var_names in sorted(toc.toc.items()):
            for var in sorted(var_names):
                # Try to fetch element metadata (ctype) if supported
                ctype = "?"
                try:
                    elem = toc.get_element_by_complete_name(f"{group}.{var}")
                    if elem is not None and hasattr(elem, "ctype"):
                        ctype = elem.ctype
                except Exception:
                    pass
                print(f"{group}.{var} ({ctype})")

if __name__ == '__main__':
    list_log_vars()





#====================================================================================================================

#====================================================================================================================
#***********$$$$$$$$*********> reading IMU and pos, vel, acc, gyro. battery and zrange
# import csv
# import time
# import threading
# from collections import deque
#
# import cflib.crtp
# from cflib.crazyflie import Crazyflie
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.crazyflie.log import LogConfig
#
# URI = 'radio://0/90/2M'
# WRITE_CSV = False                      # set True to record to CSV
# CSV_PATH  = 'cf_telemetry_100hz.csv'   # output file if WRITE_CSV=True
#
# # Shared state
# latest = {
#     't': 0.0,
#     'x': None, 'y': None, 'z': None,
#     'vx': None, 'vy': None, 'vz': None,
#     'ax': None, 'ay': None, 'az': None,
#     'gx': None, 'gy': None, 'gz': None,
#     'vbat': None, 'zrange': None,
# }
# lock = threading.Lock()
# csv_queue = deque()   # to decouple logging from callbacks
# t0 = None
#
# def now_rel():
#     return time.time() - t0
#
# def cb_pose(ts, data, _):
#     # stateEstimate.x/y/z (meters)
#     with lock:
#         latest['t']  = now_rel()
#         latest['x']  = data.get('stateEstimate.x')
#         latest['y']  = data.get('stateEstimate.y')
#         latest['z']  = data.get('stateEstimate.z')
#         if WRITE_CSV:
#             csv_queue.append(('pose', latest['t'], latest['x'], latest['y'], latest['z']))
#
# def cb_vel(ts, data, _):
#     # stateEstimate.vx/vy/vz (m/s)
#     with lock:
#         latest['t']  = now_rel()
#         latest['vx'] = data.get('stateEstimate.vx')
#         latest['vy'] = data.get('stateEstimate.vy')
#         latest['vz'] = data.get('stateEstimate.vz')
#         if WRITE_CSV:
#             csv_queue.append(('vel', latest['t'], latest['vx'], latest['vy'], latest['vz']))
#
# def cb_acc(ts, data, _):
#     # accelerometer (m/s^2)
#     with lock:
#         latest['t']  = now_rel()
#         latest['ax'] = data.get('acc.x')
#         latest['ay'] = data.get('acc.y')
#         latest['az'] = data.get('acc.z')
#         if WRITE_CSV:
#             csv_queue.append(('acc', latest['t'], latest['ax'], latest['ay'], latest['az']))
#
# def cb_gyro(ts, data, _):
#     # gyroscope (rad/s)
#     with lock:
#         latest['t']  = now_rel()
#         latest['gx'] = data.get('gyro.x')
#         latest['gy'] = data.get('gyro.y')
#         latest['gz'] = data.get('gyro.z')
#         if WRITE_CSV:
#             csv_queue.append(('gyro', latest['t'], latest['gx'], latest['gy'], latest['gz']))
#
# def cb_diag(ts, data, _):
#     # battery (V), ToF zrange (mm)
#     with lock:
#         latest['t']     = now_rel()
#         latest['vbat']  = data.get('pm.vbat')
#         latest['zrange']= data.get('range.zrange')
#         if WRITE_CSV:
#             csv_queue.append(('diag', latest['t'], latest['vbat'], latest['zrange']))
#
# def writer_thread_fn():
#     # Write one wide CSV row per time we have a complete snapshot
#     # (simple approach: just write any queued items with tags)
#     with open(CSV_PATH, 'w', newline='') as f:
#         w = csv.writer(f)
#         w.writerow(['tag','t',
#                     'x','y','z','vx','vy','vz',
#                     'ax','ay','az','gx','gy','gz',
#                     'vbat','zrange_mm'])
#         while running[0]:
#             try:
#                 tag, t, *vals = csv_queue.popleft()
#                 # Build a wide row using current latest snapshot
#                 with lock:
#                     row = [
#                         tag, f'{t:.4f}',
#                         latest['x'], latest['y'], latest['z'],
#                         latest['vx'], latest['vy'], latest['vz'],
#                         latest['ax'], latest['ay'], latest['az'],
#                         latest['gx'], latest['gy'], latest['gz'],
#                         latest['vbat'], latest['zrange'],
#                     ]
#                 w.writerow(row)
#             except IndexError:
#                 time.sleep(0.001)
#
# def main():
#     global t0
#     cflib.crtp.init_drivers()
#     print('Connecting to Crazyflie...')
#     with SyncCrazyflie(URI, cf=Crazyflie()) as scf:
#         print('Crazyflie connected!')
#         cf = scf.cf
#         t0 = time.time()
#
#         # --- Define small 10 ms blocks ---
#         # 1) Pose (3 floats)
#         lc_pose = LogConfig(name='POSE_10ms', period_in_ms=10)
#         lc_pose.add_variable('stateEstimate.x', 'float')
#         lc_pose.add_variable('stateEstimate.y', 'float')
#         lc_pose.add_variable('stateEstimate.z', 'float')
#
#         # 2) Velocity (3 floats)
#         lc_vel = LogConfig(name='VEL_10ms', period_in_ms=10)
#         lc_vel.add_variable('stateEstimate.vx', 'float')
#         lc_vel.add_variable('stateEstimate.vy', 'float')
#         lc_vel.add_variable('stateEstimate.vz', 'float')
#
#         # 3) IMU acc (3 floats)
#         lc_acc = LogConfig(name='ACC_10ms', period_in_ms=10)
#         lc_acc.add_variable('acc.x', 'float')
#         lc_acc.add_variable('acc.y', 'float')
#         lc_acc.add_variable('acc.z', 'float')
#
#         # 4) IMU gyro (3 floats)
#         lc_gyro = LogConfig(name='GYRO_10ms', period_in_ms=10)
#         lc_gyro.add_variable('gyro.x', 'float')
#         lc_gyro.add_variable('gyro.y', 'float')
#         lc_gyro.add_variable('gyro.z', 'float')
#
#         # 5) Diagnostics (keep tiny)
#         lc_diag = LogConfig(name='DIAG_10ms', period_in_ms=10)
#         lc_diag.add_variable('pm.vbat', 'float')
#         lc_diag.add_variable('range.zrange', 'uint16_t')
#
#         # Register and start
#         for cfg, cb in [
#             (lc_pose, cb_pose),
#             (lc_vel,  cb_vel),
#             (lc_acc,  cb_acc),
#             (lc_gyro, cb_gyro),
#             (lc_diag, cb_diag),
#         ]:
#             cf.log.add_config(cfg)
#             cfg.data_received_cb.add_callback(cb)
#             cfg.start()
#
#         # Optional CSV writer
#         if WRITE_CSV:
#             wt = threading.Thread(target=writer_thread_fn, daemon=True)
#             wt.start()
#
#         # Console print at 10 Hz (not 100 Hz, to keep output readable)
#         last_print = 0.0
#         while True:
#             time.sleep(0.01)  # 10 ms tick
#             if time.time() - last_print >= 0.1:  # every 100 ms
#                 with lock:
#                     print(
#                         f"t={latest['t']:.2f}s | "
#                         f"pos=({fmt(latest['x'])},{fmt(latest['y'])},{fmt(latest['z'])}) m  "
#                         f"vel=({fmt(latest['vx'])},{fmt(latest['vy'])},{fmt(latest['vz'])}) m/s  "
#                         f"acc=({fmt(latest['ax'])},{fmt(latest['ay'])},{fmt(latest['az'])})  "
#                         f"gyro=({fmt(latest['gx'])},{fmt(latest['gy'])},{fmt(latest['gz'])})  "
#                         f"vbat={fmt(latest['vbat'])} V  zrange={fmt(latest['zrange'])} mm"
#                     )
#                 last_print = time.time()
#
# def fmt(v):
#     return '—' if v is None else f'{v:.3f}'
#
# # simple running flag for CSV worker
# running = [True]
#
# if __name__ == '__main__':
#     try:
#         main()
#     except KeyboardInterrupt:
#         print('\nStopping...')
#     finally:
#         running[0] = False

# ================================================================================================================
# ==================================================================================================================

# import time
# import cflib.crtp
# from cflib.crazyflie import Crazyflie
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.crazyflie.log import LogConfig
#
# URI = 'radio://0/80/2M'
#
# def main():
#     cflib.crtp.init_drivers()
#     print('Connecting...')
#     with SyncCrazyflie(URI, cf=Crazyflie()) as scf:
#         cf = scf.cf
#
#         # Small 100 Hz block (10 ms)
#         acc = LogConfig(name='ACC_10ms', period_in_ms=10)
#         acc.add_variable('acc.x', 'float')
#         acc.add_variable('acc.y', 'float')
#         acc.add_variable('acc.z', 'float')
#
#         # Second block at 10–20 ms (keep payloads small)
#         gyro = LogConfig(name='GYRO_10ms', period_in_ms=10)
#         gyro.add_variable('gyro.x', 'float')
#         gyro.add_variable('gyro.y', 'float')
#         gyro.add_variable('gyro.z', 'float')
#
#         acc_count = 0
#         gyro_count = 0
#         t0 = None
#
#         def acc_cb(ts, data, _):
#             nonlocal acc_count
#             acc_count += 1
#
#         def gyro_cb(ts, data, _):
#             nonlocal gyro_count
#             gyro_count += 1
#
#         try:
#             cf.log.add_config(acc); acc.data_received_cb.add_callback(acc_cb); acc.start()
#             cf.log.add_config(gyro); gyro.data_received_cb.add_callback(gyro_cb); gyro.start()
#
#             print('Logging at 100 Hz for 5 s...')
#             t0 = time.time(); time.sleep(5); t1 = time.time()
#
#             gyro.stop(); acc.stop()
#             dt = t1 - t0
#             print(f'ACC:  {acc_count/dt:.1f} Hz,  GYRO: {gyro_count/dt:.1f} Hz')
#         except Exception as e:
#             print('Error starting logging:', e)
#
# if __name__ == '__main__':
#     main()






#====================================================================================================================

# import cflib.crtp
# import time
# import asyncio
# import qtm_rt
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.crazyflie.high_level_commander import HighLevelCommander
# from cflib.crazyflie import Crazyflie
#
# # Crazyflie URI and QTM IP
# URI = 'radio://0/80/2M'
# QTM_IP = "192.168.0.105"
#
# # Global variable to store the latest marker position
# # You may need to identify which markers belong to the drone and calculate its center
# latest_crazyflie_position = None
#
#
# # Callback function for the QTM stream
# def on_qtm_packet(packet):
#     global latest_crazyflie_position
#
#     # Process the packet to find the rigid body and its position
#     # The QTM SDK might group markers into rigid bodies.
#     # You will need to check your QTM project setup for the correct object name.
#
#     # This is a simplified example. You will need to adapt it
#     # based on how you have defined the rigid body in QTM.
#     info, markers = packet.get_3d_markers()
#
#     if markers and len(markers) == 4:
#         # Assuming your 4 markers define a single rigid body
#         # Calculate the center of the rigid body from the 4 marker positions
#         x_avg = sum(m[0] for m in markers) / 4.0
#         y_avg = sum(m[1] for m in markers) / 4.0
#         z_avg = sum(m[2] for m in markers) / 4.0
#
#         latest_crazyflie_position = (x_avg, y_avg, z_avg)
#         # print(f"Crazyflie Position: X={x_avg:.2f}, Y={y_avg:.2f}, Z={z_avg:.2f}")
#
#
# async def start_mocap_stream(connection):
#     """Starts the QTM stream and keeps it running in the background."""
#     await connection.stream_frames(components=["3d"], on_packet=on_qtm_packet)
#     while True:
#         await asyncio.sleep(1)  # Keep the async loop alive
#
#
# async def fly_with_mocap():
#     """Main flight control loop."""
#     cflib.crtp.init_drivers()
#
#     print("Connecting to Crazyflie...")
#     with SyncCrazyflie(URI, cf=Crazyflie()) as scf:
#         print("Crazyflie connected!")
#         cf = scf.cf
#
#         # === Step 1: Tell the Crazyflie to use an external position estimate ===
#         # This is a critical step.
#         cf.param.set_value('kalman.external_mode', '1')
#         time.sleep(0.1)
#         cf.param.set_value('kalman.resetEstimation', '1')
#         time.sleep(0.1)
#         cf.param.set_value('kalman.resetEstimation', '0')
#         time.sleep(2)  # Give it time to reset
#
#         # You should also set the initial position
#         # For this to work, you need to make sure the QTM's coordinate system
#         # aligns with the Crazyflie's (usually a forward-facing +X, left-facing +Y).
#         initial_pos = (0.0, 0.0, 0.5)  # Example target position
#
#         print("Starting take-off...")
#         cf.high_level_commander.takeoff(initial_pos[2], 0.5)
#         time.sleep(3)
#
#         # === Step 2: The Main Control Loop ===
#         try:
#             while latest_crazyflie_position is None:
#                 # Wait for the first position packet
#                 await asyncio.sleep(0.1)
#
#             # Start controlling the position
#             while True:
#                 # Send the drone's current position back to it
#                 cf.extpos.send_extpos(
#                     latest_crazyflie_position[0],
#                     latest_crazyflie_position[1],
#                     latest_crazyflie_position[2]
#                 )
#
#                 # Send a setpoint to a desired target position
#                 target_x, target_y, target_z = 0.0, 0.0, 0.5
#                 cf.commander.send_position_setpoint(target_x, target_y, target_z)
#
#                 await asyncio.sleep(0.01)  # Loop at 100 Hz
#
#         except KeyboardInterrupt:
#             print("Landing...")
#             cf.high_level_commander.land(0.0, 1.0)
#             time.sleep(2)
#
#
# async def main():
#     qtm_connection = None
#     try:
#         print(f"Attempting to connect to QTM at {QTM_IP}...")
#         qtm_connection = await qtm_rt.connect(QTM_IP)
#         if qtm_connection is None:
#             print("Failed to connect to QTM. Cannot proceed.")
#             return
#
#         print("Connected to QTM.")
#
#         # Run both tasks concurrently
#         await asyncio.gather(
#             start_mocap_stream(qtm_connection),
#             fly_with_mocap()
#         )
#
#     except Exception as e:
#         print(f"An error occurred: {e}")
#     finally:
#         if qtm_connection:
#             print("Closing QTM connection.")
#             await qtm_connection.close()
#
#
# if __name__ == "__main__":
#     asyncio.run(main())


# import cflib.crtp
# import time
# import sys
#
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.positioning.motion_commander import MotionCommander
# from cflib.crazyflie import Crazyflie
# from cflib.crazyflie.log import LogConfig
#
# # The URI for your Crazyflie
# URI = 'radio://0/80/2M'
#
#
# def main_program():
#     """Encapsulates the core flight logic."""
#     try:
#         cflib.crtp.init_drivers()
#         print('Connecting to Crazyflie...')
#         time.sleep(1)
#
#         with SyncCrazyflie(URI, cf=Crazyflie()) as scf:
#             print('Crazyflie connected!')
#
#             with MotionCommander(scf) as mc:
#                 try:
#                     print('Taking off...')
#                     time.sleep(1)
#                     mc.take_off(0.5)
#
#                     print('Hovering...')
#                     time.sleep(10)
#
#                     print('Landing...')
#                     mc.land()
#
#                 except Exception as e:
#                     print(f"An error occurred: {e}")
#                     print("Attempting to land the Crazyflie...")
#                     mc.land()
#                     raise
#
#             print('MotionCommander session ended.')
#
#     except cflib.crazyflie.ConnectionLost:
#         print("Connection to Crazyflie was lost. Please check the radio.")
#     except Exception as e:
#         print(f"An unhandled error occurred: {e}")
#     finally:
#         print('Disconnected.')
#         # Add a delay here to give the drone time to land and disarm
#         print("Waiting 5 seconds for the Crazyflie to stabilize...")
#         time.sleep(5)
#         sys.exit(0)
#
#
# if __name__ == '__main__':
#     try:
#         main_program()
#     except KeyboardInterrupt:
#         print("\nProgram interrupted by user. Exiting gracefully.")

#########################working code
# import time
# import sys
# import cflib.crtp
#
# from cflib.crazyflie import Crazyflie
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.crazyflie.log import LogConfig
# from cflib.crazyflie.syncLogger import SyncLogger
# from cflib.crazyflie.syncLogger import SyncLogger
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.utils import uri_helper
# from cflib.crazyflie.param import Param
#
# from cflib.positioning.motion_commander import MotionCommander
#
# URI = 'radio://0/80/2M'  # adapt if needed
#
# def reset_kalman(scf):
#     # Ensure Kalman estimator is used
#     scf.cf.param.set_value('stabilizer.estimator', '2')  # 2=Kalman
#     # Reset sequence recommended by Bitcraze docs
#     scf.cf.param.set_value('kalman.resetEstimation', '1')
#     time.sleep(0.1)
#     scf.cf.param.set_value('kalman.resetEstimation', '0')
#     # give it time to converge
#     time.sleep(1.0)
#
# def wait_for_flow_and_height(scf, timeout=3.0):
#     """
#     Rough health check: ensure we see sane z-range and battery.
#     """
#     lg = LogConfig(name='health', period_in_ms=100)
#     # If Flow deck is present, range.zrange should be >0 and plausible
#     lg.add_variable('range.zrange', 'uint16_t')
#     lg.add_variable('pm.vbat', 'float')
#
#     start = time.time()
#     with SyncLogger(scf, lg) as logger:
#         for log_entry in logger:
#             data = log_entry[1]
#             zrange = data.get('range.zrange', 0)
#             vbat = data.get('pm.vbat', 0.0)
#             # zrange is in mm. On the ground you might see ~30–200 mm; >0 means sensor alive.
#             if vbat < 3.6:
#                 print(f'Warning: low battery: {vbat:.2f} V (may cause oscillations)')
#             if zrange > 0:
#                 return True
#             if time.time() - start > timeout:
#                 print('Warning: No valid zrange reading from Flow/ToF (check deck, surface, height).')
#                 return False
#
# def main():
#     cflib.crtp.init_drivers()
#
#     print('Connecting...')
#     try:
#         with SyncCrazyflie(URI, cf=Crazyflie()) as scf:
#             print('Connected.')
#
#             # Stop any previous setpoints
#             scf.cf.commander.send_stop_setpoint()
#             time.sleep(0.1)
#
#             # Prepare estimator & sanity checks
#             reset_kalman(scf)
#             wait_for_flow_and_height(scf)
#
#             # Enable position set mode when using Flow/Lighthouse
#             scf.cf.param.set_value('flightmode.posSet', '1')
#
#             # (Optional) pick Mellinger controller which works nicely with pos/vel setpoints
#             # scf.cf.param.set_value('stabilizer.controller', '2')  # 1=PID, 2=Mellinger
#
#             # Take off a bit higher to avoid ground effect (0.8–1.0 m is good)
#             default_h = 0.8
#             hover_s   = 5.0
#
#             with MotionCommander(scf, default_height=default_h) as mc:
#                 print(f'Taking off to {default_h} m...')
#                 mc.take_off(height=default_h, velocity=0.5)  # gentle ascent
#
#                 print('Stabilizing hover...')
#                 time.sleep(hover_s)
#
#                 print('Landing...')
#                 mc.land(velocity=0.4)
#
#             print('Done, disarming...')
#             scf.cf.commander.send_stop_setpoint()
#             time.sleep(0.1)
#
#     except KeyboardInterrupt:
#         print('Interrupted by user, sending stop...')
#         try:
#             # Only works if a link was actually open
#             scf.cf.commander.send_stop_setpoint()  # scf may not exist if connect failed
#         except Exception:
#             pass
#     except Exception as e:
#         print(f'Error: {e}')
#         # Do not reference scf here if connection failed
#     finally:
#         # No scf references here to avoid NameError when connection fails
#         print('Exit.')
#
# if __name__ == '__main__':
#     main()


# Perfect — here’s a clean, no-residual QTM → Crazyflie bridge that uses 6deuler only.
#
# It converts mm→m, turns Euler (deg) into a quaternion, and feeds the CF
# Kalman estimator at ~100 Hz.
#
# Flip DO_FLIGHT_DEMO to True if you want it to take off, hover, do a small
# move, and land.

# import asyncio
# import math
# import threading
# import time
# import sys
#
# import qtm_rt
# import cflib.crtp
# from cflib.crazyflie import Crazyflie
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.positioning.motion_commander import MotionCommander
#
# # -------------------- USER SETTINGS --------------------
# # IMPORTANT: Make sure this URI matches your Crazyflie's channel and data rate
# URI = "radio://0/80/2M/E7E7E7E7E7"  # Example: radio://<dongle>/<channel>/<datarate>
#
# QTM_IP = "192.168.0.105"
# TARGET_RATE_HZ = 100.0  # Send pose to CF at ~100 Hz
# EULER_ORDER = "xyz"  # Euler order from QTM (adjust if needed: "zxy", "zyx", etc.)
# USE_EXTPOSE = False  # True: send full pose (pos+quat); False: position only
# DO_FLIGHT_DEMO = False  # True: takeoff/hover/move/land after pose lock
# TAKEOFF_HEIGHT_M = 0.6
# # -------------------------------------------------------
#
# PERIOD = 1.0 / TARGET_RATE_HZ
#
# # Shared state between QTM callback and control thread
# STATE = {
#     "last_sent_time": 0.0,
#     "pose_count": 0,  # number of poses sent to CF
# }
# STATE_LOCK = threading.Lock()
#
#
# # ---------- Math Helpers ----------
# def euler_deg_to_quat(rx_deg, ry_deg, rz_deg, order="xyz"):
#     """Convert Euler angles (degrees) to a quaternion (x, y, z, w)."""
#     rx = math.radians(rx_deg)
#     ry = math.radians(ry_deg)
#     rz = math.radians(rz_deg)
#
#     # Individual rotation quaternions
#     qx = (math.sin(rx / 2), 0, 0, math.cos(rx / 2))
#     qy = (0, math.sin(ry / 2), 0, math.cos(ry / 2))
#     qz = (0, 0, math.sin(rz / 2), math.cos(rz / 2))
#
#     def q_mult(q1, q2):
#         x1, y1, z1, w1 = q1
#         x2, y2, z2, w2 = q2
#         return (
#             w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
#             w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
#             w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
#             w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
#         )
#
#     q = (0, 0, 0, 1.0)  # Identity quaternion
#     # Apply rotations in the specified order
#     for axis in order.lower():
#         if axis == "x":
#             q = q_mult(q, qx)
#         elif axis == "y":
#             q = q_mult(q, qy)
#         elif axis == "z":
#             q = q_mult(q, qz)
#         else:
#             raise ValueError(f"Invalid Euler order axis: {axis}")
#
#     # Normalize the final quaternion
#     x, y, z, w = q
#     norm = math.sqrt(x * x + y * y + z * z + w * w)
#     if norm == 0:
#         return (0, 0, 0, 1.0)
#     return (x / norm, y / norm, z / norm, w / norm)
#
#
# # ----------------------------------
#
# def start_control_thread(scf):
#     """Optional flight demo after the estimator gets external pose."""
#
#     def run():
#         # Wait for ~0.5s of pose data (50 packets at 100 Hz) before takeoff
#         print("[CTRL] Waiting for pose lock from QTM...")
#         while True:
#             time.sleep(0.1)
#             with STATE_LOCK:
#                 if STATE["pose_count"] >= 50:
#                     break
#
#         print("[CTRL] Pose lock detected. Taking off…")
#         try:
#             with MotionCommander(scf, default_height=TAKEOFF_HEIGHT_M) as mc:
#                 mc.take_off(height=TAKEOFF_HEIGHT_M, velocity=0.5)
#                 time.sleep(3.0)
#                 print("[CTRL] Moving forward and right...")
#                 mc.forward(0.3)
#                 mc.right(0.3)
#                 time.sleep(3.0)
#                 print("[CTRL] Landing...")
#                 mc.land(velocity=0.4)
#         finally:
#             print("[CTRL] Control sequence finished.")
#
#     th = threading.Thread(target=run, daemon=True)
#     th.start()
#     return th
#
#
# async def main():
#     """Main function to connect to QTM and Crazyflie."""
#     cflib.crtp.init_drivers()
#     scf = SyncCrazyflie(URI, cf=Crazyflie())
#
#     try:
#         # ---- Connect Crazyflie ----
#         print(f"[CF] Connecting to {URI}...")
#         scf.open_link()
#         cf = scf.cf
#         print("[CF] Connected.")
#
#         # Set estimator and mode
#         cf.param.set_value('stabilizer.estimator', '2')  # 2 for Kalman
#         cf.param.set_value('flightmode.posSet', '1')  # Enable position setpoints
#
#         # Reset Kalman estimator
#         cf.param.set_value('kalman.resetEstimation', '1')
#         time.sleep(0.1)
#         cf.param.set_value('kalman.resetEstimation', '0')
#         print("[CF] Estimator reset. Waiting for QTM pose...")
#         time.sleep(0.5)  # Give it time to stabilize
#
#         # ---- Connect QTM ----
#         print(f"[QTM] Connecting to {QTM_IP}...")
#         conn = await qtm_rt.connect(QTM_IP)
#         if conn is None:
#             print("[QTM] Failed to connect.")
#             return
#
#         def on_packet(packet):
#             """Callback for each packet received from QTM."""
#             _, bodies = packet.get_6d_euler()
#             if not bodies:
#                 return
#
#             # Assuming the first rigid body is the Crazyflie
#             pos_mm, euler_deg = bodies[0]
#             x_m = pos_mm[0] / 1000.0  # mm -> m
#             y_m = pos_mm[1] / 1000.0
#             z_m = pos_mm[2] / 1000.0
#             rx_deg, ry_deg, rz_deg = euler_deg
#
#             # ---- Axis mapping (QTM -> CF) if needed ----
#             # Example to swap X/Y and invert Z:
#             # x, y, z = y_m, x_m, -z_m
#             # You must also adjust the Euler angles/quaternion to match!
#             # For now, we assume frames are aligned.
#             x, y, z = x_m, y_m, z_m
#             # -------------------------------------------
#
#             # Convert orientation to a quaternion
#             qx, qy, qz, qw = euler_deg_to_quat(rx_deg, ry_deg, rz_deg, order=EULER_ORDER)
#
#             # Rate-limit to TARGET_RATE_HZ
#             now = time.time()
#             with STATE_LOCK:
#                 if now - STATE["last_sent_time"] >= PERIOD:
#                     if USE_EXTPOSE:
#                         cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)
#                     else:
#                         cf.extpos.send_extpos(x, y, z)
#
#                     STATE["pose_count"] += 1
#                     STATE["last_sent_time"] = now
#
#         # Start streaming 6-DoF Euler data from QTM
#         await conn.stream_frames(
#             frames="allframes", components=["6deuler"], on_packet=on_packet
#         )
#         print(f"[QTM] Streaming. Feeding pose to CF at ~{TARGET_RATE_HZ:.0f} Hz.")
#
#         # Optional autonomous flight demo
#         ctrl_thread = None
#         if DO_FLIGHT_DEMO:
#             ctrl_thread = start_control_thread(scf)
#
#         # Keep the script alive
#         try:
#             while True if not ctrl_thread else ctrl_thread.is_alive():
#                 await asyncio.sleep(0.1)
#         except KeyboardInterrupt:
#             print("\n[MAIN] Stopping (Ctrl+C).")
#
#     except Exception as e:
#         print(f"\n[ERROR] An error occurred: {e}")
#         # This will catch the "Too many packets lost" error and provide context
#         print("Please check the troubleshooting guide for radio link issues.")
#
#     finally:
#         print("[CLEANUP] Stopping QTM stream and closing CF link...")
#         if 'conn' in locals() and conn.is_connected:
#             await conn.stream_frames_stop()
#         if scf.is_connected():
#             scf.close_link()
#         print("[CLEANUP] Done.")
#
#
# if __name__ == "__main__":
#     try:
#         asyncio.run(main())
#     except KeyboardInterrupt:
#         sys.exit(0)


# Perfect — here’s a clean, no-residual QTM → Crazyflie bridge that uses 6deuler only.
#
# It converts mm→m, turns Euler (deg) into a quaternion, and feeds the CF
# Kalman estimator.
#
# Flip DO_FLIGHT_DEMO to True if you want it to take off, hover, do a small
# move, and land.

# import asyncio
# import math
# import threading
# import time
# import sys
#
# import qtm_rt
# import cflib.crtp
# from cflib.crazyflie import Crazyflie
# from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
# from cflib.positioning.motion_commander import MotionCommander
#
# # -------------------- USER SETTINGS & TROUBLESHOOTING --------------------
# # Radio URI for your Crazyflie
# URI = "radio://0/80/2M/E7E7E7E7E7"
#
# # QTM Server IP
# QTM_IP = "192.168.0.105"
#
# # --- RADIO STABILITY TUNING ---
# # If you get "Too many packets lost", the radio link is saturated or has interference.
# # 1. Lower TARGET_RATE_HZ. Start with a very safe value like 30.
# # 2. If 30 Hz is stable, you can try increasing it to find the limit (e.g., 40, 50, 60).
# # 3. If even 30 Hz is NOT stable, the issue is environmental. Check for Wi-Fi interference
# #    and move the Crazyradio dongle away from other electronics using a USB extension cable.
# TARGET_RATE_HZ = 30.0  # <<< DIAGNOSTIC RATE: Start low to confirm stability.
#
# EULER_ORDER = "xyz"  # Euler order from QTM (adjust if needed: "zxy", "zyx", etc.)
# USE_EXTPOSE = False  # True: send full pose (pos+quat); False: position only
# DO_FLIGHT_DEMO = False  # True: takeoff/hover/move/land after pose lock
# TAKEOFF_HEIGHT_M = 0.6
# # -------------------------------------------------------------------------
#
# PERIOD = 1.0 / TARGET_RATE_HZ
#
# # Shared state
# STATE = {"last_sent_time": 0.0, "pose_count": 0}
# STATE_LOCK = threading.Lock()
#
#
# def euler_deg_to_quat(rx_deg, ry_deg, rz_deg, order="xyz"):
#     """Convert Euler angles (degrees) to a quaternion (x, y, z, w)."""
#     rx, ry, rz = math.radians(rx_deg), math.radians(ry_deg), math.radians(rz_deg)
#     qx = (math.sin(rx / 2), 0, 0, math.cos(rx / 2))
#     qy = (0, math.sin(ry / 2), 0, math.cos(ry / 2))
#     qz = (0, 0, math.sin(rz / 2), math.cos(rz / 2))
#
#     def q_mult(q1, q2):
#         x1, y1, z1, w1 = q1
#         x2, y2, z2, w2 = q2
#         return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
#                 w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)
#
#     q = (0, 0, 0, 1.0)
#     for axis in order.lower():
#         if axis == "x":
#             q = q_mult(q, qx)
#         elif axis == "y":
#             q = q_mult(q, qy)
#         elif axis == "z":
#             q = q_mult(q, qz)
#         else:
#             raise ValueError(f"Invalid Euler order axis: {axis}")
#
#     x, y, z, w = q
#     norm = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
#     return (x / norm, y / norm, z / norm, w / norm)
#
#
# def start_control_thread(scf):
#     """Optional flight demo after the estimator gets external pose."""
#
#     def run():
#         required_poses = int(TARGET_RATE_HZ * 0.5)
#         print(f"[CTRL] Waiting for {required_poses} pose packets from QTM...")
#         while True:
#             time.sleep(0.1)
#             with STATE_LOCK:
#                 if STATE["pose_count"] >= required_poses: break
#
#         print("[CTRL] Pose lock detected. Taking off…")
#         try:
#             with MotionCommander(scf, default_height=TAKEOFF_HEIGHT_M) as mc:
#                 mc.take_off(height=TAKEOFF_HEIGHT_M, velocity=0.5)
#                 time.sleep(3.0)
#                 print("[CTRL] Moving forward and right...")
#                 mc.forward(0.3);
#                 mc.right(0.3)
#                 time.sleep(3.0)
#                 print("[CTRL] Landing...")
#                 mc.land(velocity=0.4)
#         finally:
#             print("[CTRL] Control sequence finished.")
#
#     th = threading.Thread(target=run, daemon=True)
#     th.start()
#     return th
#
#
# async def main():
#     """Main function to connect to QTM and Crazyflie."""
#     cflib.crtp.init_drivers()
#     scf = SyncCrazyflie(URI, cf=Crazyflie())
#     conn = None
#
#     try:
#         print(f"[CF] Connecting to {URI}...")
#         scf.open_link()
#         cf = scf.cf
#         print("[CF] Connected.")
#
#         cf.param.set_value('stabilizer.estimator', '2')
#         cf.param.set_value('flightmode.posSet', '1')
#         cf.param.set_value('kalman.resetEstimation', '1');
#         time.sleep(0.1)
#         cf.param.set_value('kalman.resetEstimation', '0');
#         time.sleep(0.5)
#         print("[CF] Estimator reset. Waiting for QTM pose...")
#
#         print(f"[QTM] Connecting to {QTM_IP}...")
#         conn = await qtm_rt.connect(QTM_IP)
#         if conn is None:
#             print("[QTM] Failed to connect.");
#             return
#
#         def on_packet(packet):
#             _, bodies = packet.get_6d_euler()
#             if not bodies: return
#
#             pos_mm, euler_deg = bodies[0]
#             x, y, z = [p / 1000.0 for p in pos_mm]
#             qx, qy, qz, qw = euler_deg_to_quat(*euler_deg, order=EULER_ORDER)
#
#             now = time.time()
#             with STATE_LOCK:
#                 if now - STATE["last_sent_time"] >= PERIOD:
#                     if USE_EXTPOSE:
#                         cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)
#                     else:
#                         cf.extpos.send_extpos(x, y, z)
#                     STATE["pose_count"] += 1
#                     STATE["last_sent_time"] = now
#
#         await conn.stream_frames(components=["6deuler"], on_packet=on_packet)
#         print(f"[QTM] Streaming. Feeding pose to CF at ~{TARGET_RATE_HZ:.0f} Hz.")
#
#         ctrl_thread = None
#         if DO_FLIGHT_DEMO: ctrl_thread = start_control_thread(scf)
#
#         while True if not ctrl_thread else ctrl_thread.is_alive():
#             await asyncio.sleep(0.1)
#
#     except KeyboardInterrupt:
#         print("\n[MAIN] Stopping (Ctrl+C).")
#     except Exception as e:
#         print(f"\n[ERROR] An error occurred: {e}")
#     finally:
#         print("[CLEANUP] Stopping QTM stream and closing CF link...")
#         # <<< ROBUST CLEANUP BLOCK >>>
#         # This will now shut down cleanly without errors.
#         if conn is not None:
#             try:
#                 await conn.stream_frames_stop()
#             except:
#                 pass  # Ignore errors if stream is already stopped
#
#         # The SyncCrazyflie object handles its own connection state.
#         # Just call close_link() and it will handle whether it's open or not.
#         try:
#             scf.close_link()
#         except:
#             pass  # Ignore errors if link is already closed
#
#         print("[CLEANUP] Done.")
#
#
# if __name__ == "__main__":
#     asyncio.run(main())
