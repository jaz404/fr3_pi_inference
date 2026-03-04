import time

import panda_py

ROBOT_IP = "192.168.1.105"
USER = "jagersand"
PASS = "jagersand"

desk = panda_py.Desk(ROBOT_IP, USER, PASS, platform="fr3")
desk.take_control(force=True)


desk.release_control()
desk.lock()

print("FCI locked")
