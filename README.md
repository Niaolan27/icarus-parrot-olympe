# icarus-parrot-olympe
## sphinx setup

Run these commands to launch the Sphinx simulator and an Anafi Ai drone.
```
sudo systemctl start firmwared.service

sphinx "/opt/parrot-sphinx/usr/share/sphinx/drones/anafi_ai.drone"::firmware="https://firmware.parrot.com/Versions/anafi2/pc/%23latest/images/anafi2-pc.ext2.zip"::wifi_iface=""

# This launches an emtpy world.
parrot-ue4-empty
```

You can also download more worlds. Check the available worlds by running these commands
```
sudo apt update
apt-cache search parrot-ue
```

The following code is to gain shell access to the Anafi. This allows you to view the logs from ulog.
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

Turns out I cannot really test out the capabilities because Anafi Ai does not support SDK version 8.4.0. 
You need to run the Anafi UKR.

## road_runner
This is a comprehensive mission that runs both services and guidance modes. The drone runs a computer vision
algorithm on the video frames, identifying the center road line. From there, it calculates the necessary movement
to follow the road.

The guidance mode makes the drone follow the road based on the calculated movements from the service.

## yolo-test
Ignore this for now. I am trying to see if I can run a simple YOLO algorithm onboard the drone itself.

