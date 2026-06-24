import os

os.environ.setdefault("PYOPENGL_PLATFORM", "glx")

import olympe
from olympe.messages.ardrone3.Piloting import TakeOff, Landing
from olympe.messages.ardrone3.PilotingState import FlyingStateChanged, AlertStateChanged


def print_state(label, message):
    try:
        print(f"{label}:", drone.get_state(message))
    except ValueError as exc:
        print(f"{label}: unavailable ({exc})")


drone = olympe.Drone("192.168.42.1")
assert drone.connect(retry=3)

print_state("Flying before", FlyingStateChanged)
print_state("Alert before", AlertStateChanged)

takeoff = drone(
    TakeOff()
    >> FlyingStateChanged(state="takingoff", _policy="check_wait", _timeout=5)
    >> FlyingStateChanged(state="hovering", _policy="wait", _timeout=30)
).wait(_timeout=35)

print("Takeoff success:", takeoff.success())
print(takeoff.explain())
print_state("Flying after", FlyingStateChanged)
print_state("Alert after", AlertStateChanged)

input("Press Enter to land...")
drone(Landing() >> FlyingStateChanged(state="landed", _timeout=30)).wait(_timeout=35)
drone.disconnect()
