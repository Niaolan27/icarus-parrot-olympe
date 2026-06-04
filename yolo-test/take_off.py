#!/usr/bin/env python3


import time


import os
import time

os.environ.setdefault("PYOPENGL_PLATFORM", "glx")

import olympe


from olympe.messages.ardrone3.Piloting import TakeOff

drone = olympe.Drone("10.202.0.1")
drone.connect()

assert drone(TakeOff()).wait()

time.sleep(2000)

drone.disconnect()
