import socket
import struct

STATE_PORT = 9091
STATE_FMT = "<14d"
STATE_SIZE = struct.calcsize(STATE_FMT)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", STATE_PORT))

print(f"Listening on UDP port {STATE_PORT}...")

while True:
    data, addr = sock.recvfrom(2048)

    if len(data) < STATE_SIZE:
        print(f"Short packet from {addr}: {len(data)} bytes")
        continue

    state = struct.unpack(STATE_FMT, data[:STATE_SIZE])

    x, y, z = state[0:3]
    r, p, yaw = state[3:6]
    g = state[6]
    joints = state[7:14]

    print(f"\nFrom {addr}")
    print(f"EE: x={x:.3f}, y={y:.3f}, z={z:.3f}")
    print(f"RPY: r={r:.3f}, p={p:.3f}, yaw={yaw:.3f}")
    print(f"Gripper: {g:.3f}")
    print(f"Joints: {[round(j, 3) for j in joints]}")