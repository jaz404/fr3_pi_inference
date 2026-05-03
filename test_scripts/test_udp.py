#!/usr/bin/env python3
import socket
import struct
import time

# CONFIG
ROBOT_IP = "192.168.1.161"   # machine/container host running udp_to_franky_ros
ACTION_PORT = 9090           # robot listens here for <8d>
STATE_PORT = 9091            # this script listens here for <14d>

ACTION_FMT = "<8d"
STATE_FMT = "<14d"

ACTION_SIZE = struct.calcsize(ACTION_FMT)
STATE_SIZE = struct.calcsize(STATE_FMT)

# SOCKETS
send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.bind(("0.0.0.0", STATE_PORT))
recv_sock.settimeout(2.0)

print(f"Listening for robot state on 0.0.0.0:{STATE_PORT}")
print(f"Will send one action to {ROBOT_IP}:{ACTION_PORT}")

# 1. RECEIVE ONE STATE

print("\nWaiting for state packet...")

data, addr = recv_sock.recvfrom(2048)

if len(data) < STATE_SIZE:
    raise RuntimeError(f"Short state packet: {len(data)} bytes, expected {STATE_SIZE}")

state = struct.unpack(STATE_FMT, data[:STATE_SIZE])

x, y, z = state[0:3]
r, p, yaw = state[3:6]
g = state[6]
joints = state[7:14]

print("\n----- RECEIVED STATE -----")
print(f"From: {addr}")
print(f"EE: x={x:.3f}, y={y:.3f}, z={z:.3f}")
print(f"RPY: r={r:.3f}, p={p:.3f}, yaw={yaw:.3f}")
print(f"Gripper: {g:.3f}")
print(f"Joints: {[round(j, 3) for j in joints]}")

# 2. SEND ONE ACTION
# <8d>: dq1..dq7 normalized, gripper command
action = [
    0.5, 0.5, 0.5, 0.0, 0.0, 0.0, 0.0,
    0.0   # 0=open, 1=close
]

pkt = struct.pack(ACTION_FMT, *action)
send_sock.sendto(pkt, (ROBOT_IP, ACTION_PORT))

print("\n----- SENT ONE ACTION -----")
print(f"To: {ROBOT_IP}:{ACTION_PORT}")
print(f"Action: {action}")

# 3. RECEIVE UPDATED STATE
time.sleep(0.2)

print("\nWaiting for updated state packet...")

data, addr = recv_sock.recvfrom(2048)

if len(data) < STATE_SIZE:
    raise RuntimeError(f"Short state packet: {len(data)} bytes, expected {STATE_SIZE}")

state = struct.unpack(STATE_FMT, data[:STATE_SIZE])

x, y, z = state[0:3]
r, p, yaw = state[3:6]
g = state[6]
joints = state[7:14]

print("\n----- UPDATED STATE -----")
print(f"From: {addr}")
print(f"EE: x={x:.3f}, y={y:.3f}, z={z:.3f}")
print(f"RPY: r={r:.3f}, p={p:.3f}, yaw={yaw:.3f}")
print(f"Gripper: {g:.3f}")
print(f"Joints: {[round(j, 3) for j in joints]}")

recv_sock.close()
send_sock.close()