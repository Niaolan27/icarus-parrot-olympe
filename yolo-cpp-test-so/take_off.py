#!/usr/bin/env python3


import time


import os
import time

os.environ.setdefault("PYOPENGL_PLATFORM", "glx")

import olympe


from olympe.messages.ardrone3.Piloting import TakeOff
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, AlertStateChanged

drone = olympe.Drone("10.202.0.1")
print("Connected:", drone.connect())
print("Flying before:", drone.get_state(FlyingStateChanged))
# print("Alert before:", drone.get_state(AlertStateChanged))

result = drone(
    TakeOff()
    >> FlyingStateChanged(state="hovering", _timeout=10)
).wait(_timeout=15)

print("Success:", result.success())
print("Timed out:", result.timedout())
print(result.explain())
print("Flying after:", drone.get_state(FlyingStateChanged))
# print("Alert after:", drone.get_state(AlertStateChanged))

time.sleep(2000)

print(drone.get_state(FlyingStateChanged))

drone.disconnect()
