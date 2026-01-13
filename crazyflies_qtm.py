# -*- coding: utf-8 -*-
"""
crazyflie_qtm_controller.py

Combines QTM motion capture data with Crazyflie control to achieve stable,
drift-free flight.

How it works:
1.  A background thread connects to the QTM server and streams 6-DoF data for a
    rigid body (the Crazyflie).
2.  The latest position data is stored in a thread-safe shared variable.
3.  The main thread connects to the Crazyflie and configures it to use an
    external position estimator.
4.  A control loop continuously sends the position data from QTM to the
    Crazyflie, allowing it to know its precise location.
5.  The script then commands the Crazyflie to hover at a fixed target position,
    using the QTM data for absolute accuracy.

"""
import time
import sys
import threading
import asyncio

# --- Crazyflie Imports ---
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# --- QTM Imports ---
import qtm_rt
from qtm_rt.protocol import QRTCommandException

# --- Drone & Mocap Configuration ---new_cr
# URI for the Crazyflie Radio
URI = 'radio://0/80/2M'

# IP address of the machine running QTM
QTM_IP = "192.168.0.105"

# --- Flight Parameters ---
# The target Z height in meters for the drone to hover at.
# The X and Y will be determined by the drone's starting position.
TARGET_HEIGHT = 0.5

# Flight duration in seconds
FLIGHT_DURATION_S = 15.0

# --- Global Shared State for Position ---
# A thread-safe structure to hold the latest position data from QTM.
# Format: [x, y, z, timestamp]
latest_position_data = {
    "pos": None,
    "lock": threading.Lock()
}


class QTMThread(threading.Thread):
    """
    A dedicated thread to handle the asynchronous connection to QTM
    and update the shared position data.
    """

    def __init__(self, qtm_ip):
        super().__init__()
        self.daemon = True  # Thread will exit when main program exits
        self._qtm_ip = qtm_ip
        self.name = "QTMThread"
        self._loop = asyncio.new_event_loop()

    def run(self):
        """The main entry point for the thread."""
        print(f"[{self.name}] Starting asyncio event loop.")
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._qtm_streaming_loop())
        except Exception as e:
            print(f"[{self.name}] Error in asyncio loop: {e}")
        finally:
            print(f"[{self.name}] Event loop finished.")
            self._loop.close()

    async def _qtm_streaming_loop(self):
        """Connects to QTM and starts streaming frames."""
        print(f"[{self.name}] Attempting to connect to QTM at {self._qtm_ip}...")
        connection = await qtm_rt.connect(self._qtm_ip)
        if connection is None:
            print(f"[{self.name}] Failed to connect to QTM.")
            # Signal main thread that connection failed
            with latest_position_data["lock"]:
                latest_position_data["pos"] = "CONNECTION_FAILED"
            return

        print(f"[{self.name}] Successfully connected to QTM.")

        # --- Define the packet callback ---
        def on_packet(packet):
            # Try to get Euler data first as it's directly useful
            if hasattr(packet, "get_6d_euler"):
                info, bodies = packet.get_6d_euler()
                if bodies:
                    # Assuming the first rigid body is the Crazyflie
                    pos_mm, _ = bodies[0]
                    # Convert from mm to meters
                    x = pos_mm[0] / 1000.0
                    y = pos_mm[1] / 1000.0
                    z = pos_mm[2] / 1000.0

                    # Update the shared state safely
                    with latest_position_data["lock"]:
                        latest_position_data["pos"] = [x, y, z, time.time()]

        # Try to start streaming with '6deuler', fall back to '6d'
        try:
            await connection.stream_frames(
                frames="allframes",
                components=["6deuler"],
                on_packet=on_packet
            )
        except QRTCommandException:
            print(f"[{self.name}] '6deuler' component not supported. Trying '6d'.")
            await connection.stream_frames(
                frames="allframes",
                components=["6d"],
                on_packet=on_packet
            )

        try:
            print(f"[{self.name}] Streaming data from QTM...")
            # Keep the connection alive indefinitely.
            # This task will run until the program is interrupted.
            while True:
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            print(f"[{self.name}] Stream cancelled.")
        finally:
            print(f"[{self.name}] Stopping stream...")
            await connection.stream_frames_stop()


def reset_estimator(scf):
    """
    Resets the Crazyflie's position estimator.
    This is crucial when switching to an external positioning system.
    """
    # Set the estimator to use external position data
    scf.cf.param.set_value('stabilizer.estimator', '3')
    time.sleep(0.1)

    # Reset the Kalman filter
    scf.cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    scf.cf.param.set_value('kalman.resetEstimation', '0')
    print("Estimator reset and configured for external position.")
    # Give it a moment to converge
    time.sleep(0.5)


def main():
    """Main flight control logic."""
    cflib.crtp.init_drivers()

    # --- Start the QTM connection in the background ---
    qtm_thread = QTMThread(QTM_IP)
    qtm_thread.start()

    # --- Wait for the first QTM data packet to establish a starting position ---
    print("Waiting for first position frame from QTM...")
    initial_pos = None
    while True:
        with latest_position_data["lock"]:
            current_pos_data = latest_position_data["pos"]
        if current_pos_data == "CONNECTION_FAILED":
            print("Error: QTM connection failed. Aborting.")
            return
        if isinstance(current_pos_data, list):
            initial_pos = current_pos_data[0:3]  # Store initial [x, y, z]
            print(f"First QTM frame received: {[f'{p:.3f}' for p in initial_pos]}")
            break
        time.sleep(0.2)

    if initial_pos is None:
        print("Error: Could not obtain initial position from QTM. Aborting.")
        return

    # --- Define the flight target based on the starting position ---
    # We will hover at the starting X/Y and the configured target height
    flight_target_pos = [initial_pos[0], initial_pos[1], TARGET_HEIGHT]

    # --- Connect to the Crazyflie ---
    print(f'Connecting to Crazyflie at {URI}...')
    try:
        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
            print('Connected to Crazyflie.')

            # Reset the estimator to use our QTM data
            reset_estimator(scf)

            # --- Main Control Loop ---
            print(f"Starting flight. Target: {[f'{p:.2f}' for p in flight_target_pos]}. Duration: {FLIGHT_DURATION_S}s")
            start_time = time.time()

            while time.time() - start_time < FLIGHT_DURATION_S:
                with latest_position_data["lock"]:
                    current_pos_data = latest_position_data["pos"]

                # Safety check: ensure QTM data is fresh
                if current_pos_data and (time.time() - current_pos_data[3] < 0.5):
                    x, y, z, _ = current_pos_data

                    # Send the external position update to the drone
                    scf.cf.extpos.send_extpos(x, y, z)

                    # Send the command to fly to the target position
                    # Yaw is set to 0.0 degrees
                    target_x, target_y, target_z = flight_target_pos
                    scf.cf.commander.send_position_setpoint(target_x, target_y, target_z, 0.0)

                else:
                    print("Warning: QTM data is stale. Pausing setpoint updates.")

                # Loop at ~50 Hz
                time.sleep(0.02)

            print("Flight time elapsed. Landing...")
            # Land by commanding a lower Z position at the current X/Y
            land_start_time = time.time()
            # Get the last known X/Y to land at
            last_known_xy = flight_target_pos[0:2]
            with latest_position_data["lock"]:
                current_pos_data = latest_position_data["pos"]
                if isinstance(current_pos_data, list):
                    last_known_xy = current_pos_data[0:2]

            while time.time() - land_start_time < 2.0:
                with latest_position_data["lock"]:
                    current_pos_data = latest_position_data["pos"]
                if isinstance(current_pos_data, list):
                    x, y, z, _ = current_pos_data
                    scf.cf.extpos.send_extpos(x, y, z)
                    # Command to 10cm height at the last known X/Y
                    scf.cf.commander.send_position_setpoint(last_known_xy[0], last_known_xy[1], 0.1, 0.0)
                time.sleep(0.02)

            print('Done, disarming...')
            scf.cf.commander.send_stop_setpoint()
            time.sleep(0.1)

    except KeyboardInterrupt:
        print('Interrupted by user.')
    except Exception as e:
        print(f'An error occurred: {e}')
    finally:
        print('Exit.')


if __name__ == '__main__':
    # IMPORTANT:
    # Before running, ensure the Crazyflie's coordinate system aligns with
    # the QTM system.
    # Crazyflie default: X is forward, Y is left, Z is up.
    # You may need to swap or negate axes when calling send_extpos() if your
    # QTM setup is different.
    main()

