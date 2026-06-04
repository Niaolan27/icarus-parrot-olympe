#!/usr/bin/env python3

import os
import time
import threading

os.environ.setdefault("PYOPENGL_PLATFORM", "glx")

import olympe

from olympe.messages.developer.Command import AirSdkLog

from olympe.messages.ardrone3.Piloting import Landing, TakeOff, Emergency
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged

drone = olympe.Drone('10.202.0.1')
drone.connect()

# assert drone(AirSdkLog(enable=True)).wait()
# print("AirSDK mission logs enabled")

# print(f"Drone state: {drone.get_state(FlyingStateChanged)}")

assert drone(TakeOff()).wait()
print("Take off successful")

land_requested = threading.Event()


def wait_for_land_request():
	input("Press Enter to land immediately, or Ctrl+C to disconnect...\n")
	land_requested.set()


threading.Thread(target=wait_for_land_request, daemon=True).start()

try:
	while not land_requested.is_set():
		time.sleep(0.1)

	drone(Emergency()).wait()
	print("Emergency landing successful")
except KeyboardInterrupt:
	print("Disconnect requested")

drone.disconnect()
