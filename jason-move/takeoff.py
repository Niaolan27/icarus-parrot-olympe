#!/usr/bin/env python3

import os
import time

os.environ.setdefault("PYOPENGL_PLATFORM", "glx")

import olympe

from olympe.messages.developer.Command import AirSdkLog

from olympe.messages.ardrone3.Piloting import TakeOff
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged

drone = olympe.Drone('10.202.0.1')
drone.connect()

# assert drone(AirSdkLog(enable=True)).wait()
# print("AirSDK mission logs enabled")

# print(f"Drone state: {drone.get_state(FlyingStateChanged)}")

assert drone(TakeOff()).wait()
print("Take off successful")
# print(f"Drone state after takeoff: {drone.get_state(FlyingStateChanged)}")
time.sleep(30)
# print(f"Drone state after 10 seconds: {drone.get_state(FlyingStateChanged)}")

drone.disconnect()
