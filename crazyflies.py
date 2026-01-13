# import logging, time
# import cflib.crtp
# from cflib.crazyflie import Crazyflie
# from cflib.crazyflie.log import LogConfig
# from cflib.utils import uri_helper
# from cflib.drivers.crazyradio import Crazyradio
#
# # 1. Init drivers
# cflib.crtp.init_drivers(enable_debug_driver=False)
#
# # 2. (PA only) boost power
# try:
#     radio = Crazyradio(devid=0)
#     radio.set_power(Crazyradio.P_0DBM)
#     radio.close()
# except Exception:
#     pass
#
# # 3. Use a quieter channel + unique address
# URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')
#
# cf = Crazyflie(rw_cache='./cache')
# cf.connected.add_callback(lambda uri: on_connect(cf, uri))
# cf.connection_failed.add_callback(lambda uri, msg: logging.error(f"Conn failed: {msg}"))
# cf.open_link(URI)
#
# def on_connect(cf, uri):
#     print(f"✅ Connected to {uri}")
#     log = LogConfig(name='IMU', period_in_ms=20)
#     for var in ('acc.x','acc.y','acc.z','gyro.x','gyro.y','gyro.z'):
#         log.add_variable(var, 'float')
#     cf.log.add_config(log)
#     log.data_received_cb.add_callback(lambda t,d,l: print(d))
#     log.error_cb.add_callback(lambda lc, msg: print(f"Log error: {msg}"))
#     log.start()
#
# try:
#     while True:
#         time.sleep(1)
# except KeyboardInterrupt:
#     cf.close_link()
#     print("Stopped.")


# from cflib.crtp import init_drivers, scan_interfaces
#
# # 1. Initialize the drivers (must be called before anything else)
# init_drivers(enable_debug_driver=False)
#
# # 2. Scan for all Crazyradio/Crazyflie interfaces
# radios = scan_interfaces()
# print("Found interfaces:")
# for uri, comment in radios:
#     print(f"  {uri}  →  {comment}")



# list_usb.py
# list_usb_backend.py
# import usb.core, usb.util
# from usb.backend import libusb1
#
# backend = libusb1.get_backend()
# if backend is None:
#     print("❌ Could not find a libusb1 backend DLL!")
# else:
#     found = False
#     for dev in usb.core.find(find_all=True, backend=backend):
#         found = True
#         vid, pid = hex(dev.idVendor), hex(dev.idProduct)
#         try:
#             name = usb.util.get_string(dev, dev.iProduct)
#         except Exception:
#             name = "<no product string>"
#         print(f"{vid}:{pid} → {name}")
#     if not found:
#         print("⚠️  libusb1 sees *no* USB devices. Something is still wrong with your driver/backend.")


# import logging
# import time
#
# import cflib.crtp
# from cflib.crazyflie import Crazyflie
# from cflib.crazyflie.log import LogConfig
#
# # -----------------------------------------------------------
# # 1. SETUP LOGGING (optional, but recommended)
# # -----------------------------------------------------------
# logging.basicConfig(level=logging.INFO)
# logger = logging.getLogger(__name__)
#
# # -----------------------------------------------------------
# # 2. INITIALIZE CRTP DRIVER
# # -----------------------------------------------------------
# # This must be called before opening any Crazyflie links.
# cflib.crtp.init_drivers(enable_debug_driver=False)
#
# # -----------------------------------------------------------
# # 3. DEFINE YOUR RADIO URI
# # -----------------------------------------------------------
# # According to your settings:
# #   • Channel: 80
# #   • Bandwidth: 2Mbit/s
# #   • Address: 0xE7E7E7E7E7  →  use “E7E7E7E7E7” (five bytes in hex)
# #
# # Full URI format: "radio://<port>/<channel>/<datarate>/<address>"
# #
# # Here we leave <port> blank so the library picks the first available Crazyradio PA.
# URI = 'radio://0/80/2M/E7E7E7E7E7'
#
# # -----------------------------------------------------------
# # 4. PREPARE CALLBACKS & LOG CONFIGURATION
# # -----------------------------------------------------------
# class IMULogger:
#     def __init__(self, uri):
#         self._cf = Crazyflie(rw_cache='./cache')  # you can choose a cache directory
#         self._uri = uri
#         self._log_config = None
#         self._connected = False
#
#         # Register callbacks
#         self._cf.connected.add_callback(self._on_connect)
#         self._cf.disconnected.add_callback(self._on_disconnect)
#         self._cf.connection_failed.add_callback(self._on_connection_failed)
#
#         # Open the link
#         logger.info(f'Trying to connect to CF with URI: {self._uri}')
#         self._cf.open_link(self._uri)
#
#     def _on_connect(self, link_uri):
#         """Called when the Crazyflie is successfully connected."""
#         logger.info(f'Connected to {link_uri}')
#         self._connected = True
#
#         # 4.1. Create a logging configuration
#         # We name it “IMU” and set period = 100 ms → 10 Hz
#         self._log_config = LogConfig(name='IMU', period_in_ms=100)
#
#         # 4.2. Add the IMU variables you want to log
#         #    (accelerometer + gyroscope + (optional) magnetometer)
#         self._log_config.add_variable('acc.x', 'float')
#         self._log_config.add_variable('acc.y', 'float')
#         self._log_config.add_variable('acc.z', 'float')
#         self._log_config.add_variable('gyro.x', 'float')
#         self._log_config.add_variable('gyro.y', 'float')
#         self._log_config.add_variable('gyro.z', 'float')
#         # If you do want magnetometer data as well, you can also add:
#         # self._log_config.add_variable('mag.x', 'float')
#         # self._log_config.add_variable('mag.y', 'float')
#         # self._log_config.add_variable('mag.z', 'float')
#
#         # 4.3. Register a callback to receive the logged packets
#         self._log_config.data_received_cb.add_callback(self._on_log_data)
#
#         # 4.4. Start logging
#         try:
#             self._cf.log.add_config(self._log_config)
#             self._log_config.start()
#             logger.info('Log configuration started (IMU → 10 Hz).')
#         except Exception as e:
#             logger.error(f'Failed to add/start log configuration: {e}')
#
#     def _on_disconnect(self, link_uri):
#         """Called when the Crazyflie disconnects."""
#         logger.info(f'Disconnected from {link_uri}')
#         self._connected = False
#
#     def _on_connection_failed(self, link_uri, msg):
#         """Called when the Crazyflie connection fails."""
#         logger.error(f'Connection to {link_uri} failed: {msg}')
#         self._connected = False
#
#     def _on_log_data(self, timestamp, data, logconf):
#         """
#         Called every time a new packet arrives for the 'IMU' logconf.
#         ‣ timestamp: time in ms (CF’s system time)
#         ‣ data:    dict of variable_name → value
#         ‣ logconf: reference to the LogConfig object
#         """
#         # Example: print a nicely formatted line of all six IMU values
#         ax = data.get('acc.x')
#         ay = data.get('acc.y')
#         az = data.get('acc.z')
#         gx = data.get('gyro.x')
#         gy = data.get('gyro.y')
#         gz = data.get('gyro.z')
#         print(f'[{timestamp:8d} ms]  '
#               f'Acc = ({ax:+6.3f}, {ay:+6.3f}, {az:+6.3f})  '
#               f'Gyro= ({gx:+6.3f}, {gy:+6.3f}, {gz:+6.3f})')
#
#     def run(self, duration_s=10.0):
#         """
#         Keep the script alive for duration_s seconds so that we actually
#         receive log packets. After that, disconnect cleanly.
#         """
#         start = time.time()
#         try:
#             while time.time() - start < duration_s:
#                 # Do nothing here; callbacks handle everything
#                 time.sleep(0.01)
#         except KeyboardInterrupt:
#             logger.info('Interrupted by user.')
#         finally:
#             # Stop logging (if active) and disconnect
#             if self._log_config:
#                 try:
#                     self._log_config.stop()
#                 except Exception:
#                     pass
#             logger.info('Closing link...')
#             self._cf.close_link()
#
# # -----------------------------------------------------------
# # 5. MAIN ENTRY POINT
# # -----------------------------------------------------------
# if __name__ == '__main__':
#     imu_logger = IMULogger(URI)
#
#     # Wait up to 5 seconds for connection
#     max_wait = 5.0
#     t0 = time.time()
#     while not imu_logger._connected and time.time() - t0 < max_wait:
#         time.sleep(0.1)
#
#     if not imu_logger._connected:
#         logger.error('Could not connect within 5 seconds → exiting.')
#     else:
#         # Log IMU data for 10 seconds (adjust as needed)
#         imu_logger.run(duration_s=10.0)
#         logger.info('Done reading IMU.')

# import time
# import cflib.crtp
#
# # Initialize drivers without the deprecated debug driver
# cflib.crtp.init_drivers(enable_debug_driver=False)
#
# print("Scanning for Crazyflies …")
# # This returns a list of all URIs that the Crazyradio sees
# all_uris = cflib.crtp.scan_interfaces()
# print("All discovered URIs:", all_uris)
#
# # Now filter for your specific settings (ch = 80, dr = 2M)
# matching = [uri for uri in all_uris if uri.startswith('radio://') and '/80/2M/' in uri]
# print("URIs on channel 80, 2 Mbit/s:", matching)
#



import logging, time
import cflib.crtp
from cflib.crazyflie import Crazyflie

# 1) Setup logging and radio drivers
logging.basicConfig(level=logging.ERROR)
cflib.crtp.init_drivers(enable_debug_driver=False)

# 2) Find and connect to the first Crazyflie
available = cflib.crtp.scan_interfaces()
if not available:
    raise RuntimeError("No Crazyflie found!")
uri = available[0][0]

cf = Crazyflie(rw_cache='./cache')

def on_connected(link_uri):
    print("**Connected – enabling altitude hold**")
    # 3) Turn on altitude‐hold mode
    cf.param.set_value('flightmode.althold', True)
    time.sleep(0.1)  # give the parameter server a moment

    # -- TAKEOFF --
    ascend_thrust = 45000  # >32767 gives upward velocity
    t0 = time.time()
    while time.time() - t0 < 2.0:  # 2 s of ascent
        cf.commander.send_setpoint(0, 0, 0, ascend_thrust)
        time.sleep(0.01)

    # -- HOVER --
    hover_thrust = 32767      # hold altitude
    t0 = time.time()
    while time.time() - t0 < 4.0:  # 4 s hover
        cf.commander.send_setpoint(0, 0, 0, hover_thrust)
        time.sleep(0.01)

    # -- LAND --
    descend_thrust = 20000   # <32767 gives downward velocity
    t0 = time.time()
    while time.time() - t0 < 2.0:  # 2 s descent
        cf.commander.send_setpoint(0, 0, 0, descend_thrust)
        time.sleep(0.01)

    # Cut motors
    cf.commander.send_setpoint(0, 0, 0, 0)
    print("**Landed**")
    cf.close_link()

# Register callbacks and open link
cf.connected.add_callback(on_connected)
cf.open_link(uri)

# Keep the script alive until flight is done
time.sleep(10)
