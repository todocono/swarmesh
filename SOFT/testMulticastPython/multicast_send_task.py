import socket
import struct
import json

TASK_GROUP = ("224.3.29.2", 10002)
SOCK_TASK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
coord = []

while True:
    x = input("Please input coordinate for x\n")
    y = input("Please input coordinate for y\n")
    coord.append([x, y])
    proceed = input("Type 0 to proceed, 1 to quit\n")
    if proceed:
        break

dict = {"Task": coord,
        "Num": len(coord)}

TTL = struct.pack('b', 8)
SOCK_TASK.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, TTL)
SOCK_TASK.sendto(json.dumps(dict).encode(), TASK_GROUP)
