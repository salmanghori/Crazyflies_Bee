# import asyncio
# import qtm_rt
# import time
#
#
# async def main():
#     QTM_IP = "192.168.0.105"
#
#     try:
#         print(f"Attempting to connect to QTM at {QTM_IP}...")
#         connection = await qtm_rt.connect(QTM_IP)
#
#         if connection is None:
#             print("Failed to connect to QTM")
#             return
#
#         # Get system settings to determine the streaming frequency
#         parameters = await connection.get_parameters(components=['3d'])
#         if '3d' in parameters and 'frequency' in parameters['3d']:
#             stream_frequency = parameters['3d']['frequency']
#             print(f"Connected to QTM. Streaming at {stream_frequency} Hz.")
#         else:
#             stream_frequency = None
#             print("Connected to QTM, but could not determine streaming frequency.")
#
#         # Dictionary to keep a count of frames and marker counts
#         frame_counts = {'frames_received': 0, 'markers_per_frame': 0}
#         start_time = time.time()
#
#         # --- IMPORTANT: Get the names of the rigid bodies ---
#         # You need to configure the rigid body in your QTM software first.
#         # It's better to read the rigid body data than raw markers.
#         await connection.stream_frames(components=['3d', '6d'], on_packet=lambda p: on_packet(p, frame_counts))
#
#     except asyncio.TimeoutError:
#         print("Connection timeout - check network and QTM is streaming.")
#     except ConnectionRefusedError:
#         print("Connection refused - check QTM is running and IP is correct.")
#     except KeyboardInterrupt:
#         print("Stopped by user.")
#     except Exception as e:
#         print(f"Unexpected error: {e}")
#     finally:
#         print("Closing connection...")
#         if connection:
#             await connection.close()
#
#
# def on_packet(packet, frame_counts):
#     """Packet handler to process 3D and 6D data."""
#     frame_counts['frames_received'] += 1
#
#     # Get 3D marker data
#     _, markers = packet.get_3d_markers()
#     if markers:
#         # Check if you are consistently seeing 4 markers
#         if len(markers) == 4:
#             frame_counts['markers_per_frame'] += 1
#             # You can print them here if you want to verify positions
#             # print(f"Frame {packet.framenumber}: Found 4 markers.")
#
#     # Get 6D rigid body data (better for your use case)
#     info, rigid_bodies = packet.get_6d_bodies()
#     if rigid_bodies:
#         for i, body in enumerate(rigid_bodies):
#             # Print the rigid body name, which is more reliable than marker count
#             print(f"Found Rigid Body: '{body.name}'")
#             # This is where you would check if the body name matches your Crazyflie's body
#             # For example: if body.name == 'CrazyflieBody': ...
#             pos = body.position
#             rot = body.rotation
#             print(f"  Position: X={pos.x:.2f}, Y={pos.y:.2f}, Z={pos.z:.2f}")
#             # You can use the rotation matrix for orientation if needed
#             # print(f"  Rotation: {rot}")
#
#     # Print a summary every few seconds for a clearer view
#     if time.time() - start_time > 2 and frame_counts['frames_received'] > 0:
#         print("\n--- Summary ---")
#         print(
#             f"Average marker count: {frame_counts['markers_per_frame'] / frame_counts['frames_received']:.2f} per frame.")
#         print(f"Frames received: {frame_counts['frames_received']}\n")
#         frame_counts['frames_received'] = 0
#         frame_counts['markers_per_frame'] = 0
#         start_time = time.time()
#
#
# if __name__ == "__main__":
#     start_time = time.time()
#     asyncio.run(main())

#
# import asyncio
# import qtm_rt
#
#
# async def main():
#     QTM_IP = "192.168.0.105"
#
#     try:
#         print(f"Attempting to connect to QTM at {QTM_IP}...")
#         connection = await qtm_rt.connect(QTM_IP)
#
#         if connection is None:
#             print("Failed to connect to QTM")
#             return
#
#         print("Connected to QTM, starting 3D marker stream...")
#
#         # Define a packet handler
#         def on_packet(packet):
#             frame_number = packet.framenumber
#
#             info, markers = packet.get_3d_markers()
#             if markers is not None and markers:
#                 print(f"Frame {frame_number} with {len(markers)} markers:")
#                 for i, marker in enumerate(markers):
#                     print(f"  Marker {i + 1}: X={marker[0]:.2f}, Y={marker[1]:.2f}, Z={marker[2]:.2f}")
#
#         # Start streaming and set callback
#         await connection.stream_frames(components=["3d"], on_packet=on_packet)
#
#         # Keep the connection alive
#         while True:
#             await asyncio.sleep(1)
#
#     except asyncio.TimeoutError:
#         print("Connection timeout - check network and QTM is streaming")
#     except ConnectionRefusedError:
#         print("Connection refused - check QTM is running and IP is correct")
#     except KeyboardInterrupt:
#         print("Stopped by user")
#     except Exception as e:
#         print(f"Unexpected error: {e}")
#     finally:
#         print("Closing connection... (manual stop might be required if not master)")
#         # Only try closing if you're the master (optional)
#         # await connection.close()
#
#
# if __name__ == "__main__":
#     asyncio.run(main())


# import asyncio
# import time
# import qtm_rt
#
# QTM_IP = "192.168.0.105"
#
# # Shared state for simple FPS/summary
# class Stats:
#     def __init__(self):
#         self.frames = 0
#         self.marker_frames = 0
#         self.last_print = time.time()
#
# stats = Stats()
#
# def on_packet(packet):
#     """Handle incoming QTM packets: 3D markers + 6D rigid bodies (quaternion)."""
#     stats.frames += 1
#
#     # ---- 3D markers ----
#     try:
#         _, markers = packet.get_3d_markers()
#         if markers and len(markers) == 4:
#             stats.marker_frames += 1
#         # You can print markers if needed:
#         # print(f"3D markers: {len(markers)}")
#     except Exception:
#         pass  # If 3D component not present in this packet
#
#     # ---- 6D rigid bodies (quat) ----
#     try:
#         info, bodies = packet.get_6d_quat()
#         if bodies:
#             # If you have exactly one trackable:
#             pos, quat = bodies[0]  # pos=(x,y,z) [mm], quat=(qx,qy,qz,qw)
#             # Convert mm -> m for convenience
#             x, y, z = pos[0] / 1000.0, pos[1] / 1000.0, pos[2] / 1000.0
#             qx, qy, qz, qw = quat
#             print(f"RB pose: pos=({x:.3f},{y:.3f},{z:.3f}) m, quat=({qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f})")
#     except Exception:
#         pass  # If 6d_quat not present
#
#     # ---- periodic summary (FPS & marker consistency) ----
#     now = time.time()
#     if now - stats.last_print >= 2.0:
#         fps = stats.frames / (now - stats.last_print)
#         avg_has_4 = stats.marker_frames / max(stats.frames, 1)
#         print("\n--- Summary (last 2s) ---")
#         print(f"FPS (approx): {fps:.1f}")
#         print(f"Frames with exactly 4 markers: {avg_has_4*100:.1f}%")
#         print("-------------------------\n")
#         stats.frames = 0
#         stats.marker_frames = 0
#         stats.last_print = now
#
# async def main():
#     print(f"Attempting to connect to QTM at {QTM_IP}...")
#     # Use context manager to avoid explicit 'close()' (master) command
#     async with qtm_rt.connect(QTM_IP) as connection:
#         if connection is None:
#             print("Failed to connect to QTM.")
#             return
#         print("Connected to QTM.")
#
#         # If you want to enforce a streaming rate, you can specify it:
#         # frequency can be "AllFrames" or an int (Hz), or "FrequencyDivisor:n"
#         # Examples:
#         # frequency = "AllFrames"
#         # frequency = 100
#         # frequency = "FrequencyDivisor:2"  # every 2nd frame
#         frequency = "AllFrames"
#
#         # Start streaming desired components
#         # Use '6d_quat' for pose; include '3d' if you also want raw markers
#         await connection.stream_frames(
#             components=["6d_quat", "3d"],
#             on_packet=on_packet,
#             frequency=frequency
#         )
#
#         # Keep running until Ctrl+C
#         try:
#             while True:
#                 await asyncio.sleep(0.1)
#         except asyncio.CancelledError:
#             pass
#         except KeyboardInterrupt:
#             print("Stopped by user.")
#         finally:
#             # Stop streaming (does NOT require master)
#             try:
#                 await connection.stream_frames_stop()
#             except Exception:
#                 pass
#             print("Stream stopped.")
#
# if __name__ == "__main__":
#     asyncio.run(main())


# import asyncio
# import qtm_rt
#
# QTM_IP = "192.168.0.105"
#
# async def main():
#     try:
#         print(f"Attempting to connect to QTM at {QTM_IP}...")
#         connection = await qtm_rt.connect(QTM_IP)
#         if connection is None:
#             print("Failed to connect to QTM")
#             return
#
#         print("Connected to QTM, starting 3D marker + residual stream...")
#
#         def on_packet(packet):
#             frame_number = packet.framenumber
#
#             # --- 3D markers with residuals ---
#             info3d, markers = packet.get_3d_markers_residual()
#             if markers:
#                 print(f"Frame {frame_number} | {len(markers)} markers (with residuals):")
#                 for i, (x, y, z, res) in enumerate(markers, start=1):
#                     # QTM units are mm; convert to meters if you prefer
#                     print(f"  M{i}: X={x:.2f} Y={y:.2f} Z={z:.2f}  res={res:.2f} mm")
#
#             # --- OPTIONAL: 6DOF rigid-bodies with residuals ---
#             # info6d, bodies = packet.get_6d_residual()
#             # if bodies:
#             #     for i, body in enumerate(bodies, start=1):
#             #         # body is (position_mm, rotation_matrix, residual_mm)
#             #         pos_mm, rot, res6d = body
#             #         x, y, z = (v / 1000.0 for v in pos_mm)  # mm -> m
#             #         print(f"  RB{i}: pos=({x:.3f},{y:.3f},{z:.3f}) m  res={res6d:.2f} mm")
#
#         # Start streaming. Use frames='allframes' (not "frequency=").
#         await connection.stream_frames(
#             frames="allframes",
#             components=["3dres"],      # add "6dres" or "6deulerres" if you want those too
#             on_packet=on_packet
#         )
#
#         # Keep the connection alive
#         while True:
#             await asyncio.sleep(1)
#
#     except KeyboardInterrupt:
#         print("Stopped by user")
#     except Exception as e:
#         print(f"Unexpected error: {e}")
#     finally:
#         print("Closing connection... (manual stop may require master)")
#         # If you have master control and want to stop streaming explicitly:
#         # await connection.stream_frames_stop()
#         # await connection.disconnect()
#
# if __name__ == "__main__":
#     asyncio.run(main())

import asyncio
import time
import qtm_rt
from qtm_rt.protocol import QRTCommandException

QTM_IP = "192.168.0.105"

def fps_counter(window_s=2.0):
    last = time.time()
    n = 0
    def tick():
        nonlocal last, n
        n += 1
        now = time.time()
        if now - last >= window_s:
            fps = n / (now - last)
            print(f"[QTM] ~{fps:.1f} FPS")
            n = 0
            last = now
    return tick

async def stream_with_component(connection, component, on_packet):
    """Start stream with a single component; raise if invalid."""
    await connection.stream_frames(frames="allframes",
                                   components=[component],
                                   on_packet=on_packet)

async def main():
    print(f"Attempting to connect to QTM at {QTM_IP}...")
    connection = await qtm_rt.connect(QTM_IP)
    if connection is None:
        print("Failed to connect to QTM")
        return

    # Prefer quat, then euler, then rotation matrix
    components_try = ["6d_quat", "6deuler", "6d"]
    chosen = None
    tick = fps_counter()

    def on_packet(packet):
        tick()
        # Try in order of richest data available on this packet instance
        if hasattr(packet, "get_6d_quat"):
            info, bodies = packet.get_6d_quat()
            if not bodies:
                return
            for i, (pos_mm, quat) in enumerate(bodies, start=1):
                x, y, z = (pos_mm[0]/1000.0, pos_mm[1]/1000.0, pos_mm[2]/1000.0)
                qx, qy, qz, qw = quat
                print(f"RB{i}: pos=({x:.3f},{y:.3f},{z:.3f}) m  "
                      f"quat=({qx:.4f},{qy:.4f},{qz:.4f},{qw:.4f})")
            return

        if hasattr(packet, "get_6d_euler"):
            info, bodies = packet.get_6d_euler()
            if not bodies:
                return
            for i, (pos_mm, euler_deg) in enumerate(bodies, start=1):
                x, y, z = (pos_mm[0]/1000.0, pos_mm[1]/1000.0, pos_mm[2]/1000.0)
                rx, ry, rz = euler_deg
                print(f"RB{i}: pos=({x:.3f},{y:.3f},{z:.3f}) m  "
                      f"euler=({rx:.1f},{ry:.1f},{rz:.1f}) deg")
            return

        if hasattr(packet, "get_6d"):
            info, bodies = packet.get_6d()
            if not bodies:
                return
            for i, (pos_mm, rot) in enumerate(bodies, start=1):
                x, y, z = (pos_mm[0]/1000.0, pos_mm[1]/1000.0, pos_mm[2]/1000.0)
                # rot is a 3x3 rotation matrix; print just first row to keep it short
                r00, r01, r02 = rot[0]
                print(f"RB{i}: pos=({x:.3f},{y:.3f},{z:.3f}) m  "
                      f"R[0,*]=({r00:.3f},{r01:.3f},{r02:.3f})")
            return

    # Try components in order; fall back if this qtm_rt doesn’t accept the name
    for comp in components_try:
        try:
            print(f"Connected. Starting 6-DoF stream ({comp})...")
            await stream_with_component(connection, comp, on_packet)
            chosen = comp
            break
        except QRTCommandException as e:
            print(f"Component '{comp}' not supported ({e}). Trying next...")
        except Exception as e:
            print(f"Failed starting stream with '{comp}': {e}")

    if not chosen:
        print("No compatible 6-DoF component found (tried 6d_quat, 6deuler, 6d).")
        return

    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print("Stopped by user")
    finally:
        print("Closing connection...")
        try:
            await connection.stream_frames_stop()
        except Exception:
            pass

if __name__ == "__main__":
    asyncio.run(main())
