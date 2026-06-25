# icarus-parrot-olympe
This repository includes several projects focused on testing the capabilities of the Parrot AirSDK. It aims to explore
how much computation you can perform onboard the Parrot drones themselves.

The Parrot AirSDK is mainly supported on the Parrot Anafi Ai and Parrot Anafi UKR. However, all of these projects are
tested and run on the Parrot Anafi, both in real life and in the Sphinx simulator.

## Parrot AirSDK Setup

First you need to add Parrot packages as a trusted source.
```
curl https://debian.parrot.com/gpg | sudo gpg --dearmor -o /usr/share/keyrings/debian.parrot.com.gpg
echo "deb [signed-by=/usr/share/keyrings/debian.parrot.com.gpg] https://debian.parrot.com/ $(lsb_release -cs) main generic" | sudo tee /etc/apt/sources.list.d/debian.parrot.com.list > /dev/null
sudo apt update
```

Next, install airsdk-cli.
```
sudo apt install parrot-airsdk-cli
```

## Sphinx Setup

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

## Shell Access

The following code is to gain shell access to the Anafi. This allows you to view the logs from ulog. This is very useful for
debugging whether your service is running correctly on the drone. 

You need to enable shell access under developer settings. You can find the instructions in the link below.

Anafi UKR: https://developer.parrot.com/docs/airsdk/general/developer_settings_for_anafi_ukr.html#shell-access
Anafi Ai: https://developer.parrot.com/docs/airsdk/user_guide/developer_settings_for_anafi_ai.html#developer-settings-for-anafi-ai

```
adb connect anafi-ai.local:9050
adb shell
```

## Installing the mission on the physical drone
### Security
You need to set up the security key on the drone. You need to set up a private/public key. More instructions can be found 
here https://developer.parrot.com/docs/airsdk/general/security.html.

If you don't set up the security key, the mission would not be installed onto the drone. 

# Project Examples

Here are several examples of projects testing out different capabilities.

## jason-move
I am testing out using the AirSDK directly to create a mission. I build the mission and load it onto the drone itself.
The drone moves in simple preset trajectory.
Note: when you install the mission onto the drone, the drone reboots by default. Ensure that you allow the drone enough
time before you launch the drone. Otherwise the mission may not boot correctly.

```
airsdk build
airsdk install --default
# wait for around 30 seconds
python takeoff.py
```

## olympe-airsdk
This requires AirSDK version 8.4 and higher. 
I want to test out the capabilities of running Olympe within AirSDK. The idea is that I can use Olympe within
AirSDK. So, instead of having to write C/C++ code in AirSDK, I can use Olympe instead. There are some examples
provided by Parro themselves.

Olympe is a separate SDK that is meant for controlling drones from the ground station. You can send Olympe
commands to the drone to control it.

Note: I could not test this capability because it is a new feature on the AirSDK 8.4.0. This new version is only
supported on the Anafi UKR and not the Anafi Ai. 

## road_runner
This is an example project from Parrot themselves. It is a comprehensive mission that runs both services and guidance modes. 
The drone runs a computer vision algorithm on the video frames, identifying the center road line. From there, it calculates 
the necessary movement to follow the road.

This is a good example project to figure out how services, guidance modes and the flight supervisor interact with one another.

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

## yolo-position-estimation
In this mission, I added a service which runs the position estimation algorithm alongside the CV service. At fixed intervals,
the service will run the position estimation algorithm on the bounding boxes detected at that time. 

The algorithm assumes that the target is on the ground level (Z = 0). For a given pixel, it is able to project a 
camera ray from the camera (using a pinhole camera model), and making the ray intersect with the ground. From there,
since you know the altitude of the drone and the pitch of the camera, you can solve for the X and Y displacement of
the target object relative to the drone. 

