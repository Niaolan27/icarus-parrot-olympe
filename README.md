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

## yolo-cpp-test-so
This is a mission to demonstrate the ability to run custom code onboard the drone. In this example, I managed to run
YOLO26n natively on the drone. 

To do so, I included NCNN as a dependency of the mission. I added the cross-compiled .so file and header files to 
the mission. I also updated the `atom.mk` file. NCNN is a lightweight neural network framework written in C++, optimized for edge devices. This 
allows me to run the pretrained YOLO26n model on the drone. I chose to go with NCNN instead of PyTorch/Tensorflow simply
because those frameworks are way too large to be included as a dependency. You are constrained by the storage available
on the drone.

As for the model weights, you can add them under `assets/`. 

You also need to update the `mission.yaml` file to reflect the additional dependencies that you add.

To see that the model runs, you want to set up the environment and drone, then install the mission into the drone.
Once the mission is installed, the YOLO service automatically starts running. To see the results of the inference,
you need to run `adb shell` and use `ulogcat | grep YOLO` to see log messages related to the YOLO service. 

