# icarus-parrot-olympe
## sphinx setup
```
sudo systemctl start firmwared.service

sphinx "/opt/parrot-sphinx/usr/share/sphinx/drones/anafi_ai.drone"::firmware="https://firmware.parrot.com/Versions/anafi2/pc/%23latest/images/anafi2-pc.ext2.zip"::wifi_iface=""

parrot-ue4-empty
```

The following code is to gain shell access to the Anafi.
```
adb connect anafi-ai.local:9050
adb shell
```

## jason-move
I am testing out using the AirSDK directly to create a mission. I build the mission and load it onto the drone itself.
Note: when you install the mission onto the drone, wait for around 30 seconds for the drone to reboot successfully.

```
airsdk build
airsdk install --default
# wait for around 30 seconds
python takeoff.py
```

## olympe-airsdk
This requires AirSDK version 8.4 and higher. 
I want to test out the capabilities of running Olympe within AirSDK. The idea is that I can use Olympe within
AirSDK. So, instead of having to write C/C++ code in AirSDK, I can use Olympe instead.

Olympe was previously meant for controlling drones from the ground station.
