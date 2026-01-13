# qtm_dump.py
import asyncio, math, time, xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation
import qtm_rt

QTM_IP = "192.168.0.105"
RIGID_BODY_NAME = "cf8"
PRINT_HZ = 10

last_print = 0
dt = 1.0 / PRINT_HZ
body_indices = {}
pkt_count = 0

def on_packet(packet):
    global last_print, pkt_count
    header, bodies = packet.get_6d()
    if not bodies: return
    if RIGID_BODY_NAME not in body_indices: return
    idx = body_indices[RIGID_BODY_NAME]
    pos, rotm = bodies[idx]
    if pos is None or any(math.isnan(v) for v in pos): return

    x, y, z = pos[0]/1000.0, pos[1]/1000.0, pos[2]/1000.0
    r = rotm.matrix
    rot = [[r[0], r[3], r[6]],
           [r[1], r[4], r[7]],
           [r[2], r[5], r[8]]]
    try:
        yaw = Rotation.from_matrix(rot).as_euler('zyx', degrees=True)[0]
    except Exception:
        yaw = float('nan')

    now = time.time()
    pkt_count += 1
    if now - last_print >= dt:
        print(f"[QTM] {RIGID_BODY_NAME} pos=({x:+.3f},{y:+.3f},{z:+.3f}) yaw={yaw:+6.1f}° pkts={pkt_count}")
        last_print = now

async def main():
    print(f"[QTM] Connecting to {QTM_IP} ...")
    conn = await qtm_rt.connect(QTM_IP)
    if conn is None:
        print("[QTM] Failed to connect")
        return
    print("[QTM] Connected.")

    params = await conn.get_parameters(parameters=['6d'])
    xml = ET.fromstring(params)
    labels = [n.text.strip() for n in xml.findall('*/Body/Name')]
    for i, n in enumerate(labels):
        body_indices[n] = i
    print(f"[QTM] Bodies: {labels}")
    if RIGID_BODY_NAME not in body_indices:
        print(f"[QTM] Body '{RIGID_BODY_NAME}' not found.")
        return

    await conn.stream_frames(components=['6d'], on_packet=on_packet)
    try:
        while True:
            await asyncio.sleep(0.5)
    finally:
        try: await conn.stream_frames_stop()
        except: pass
        conn.disconnect()

if __name__ == "__main__":
    asyncio.run(main())
