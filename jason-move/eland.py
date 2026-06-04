#!/usr/bin/env python3

import os
import time

os.environ.setdefault("PYOPENGL_PLATFORM", "glx")

import olympe

from olympe.messages.developer.Command import AirSdkLog

from olympe.messages.ardrone3.Piloting import Landing

drone = olympe.Drone('10.202.0.1')
drone.connect()

drone(Landing()).wait()
print("Landed successful")

drone.disconnect()