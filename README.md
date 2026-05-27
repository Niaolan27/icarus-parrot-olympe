# icarus-parrot-olympe

# jason-move
I am testing out using the AirSDK directly to create a mission. I build the mission and load it onto the drone itself.
Note: when you install the mission onto the drone, wait for around 30 seconds for the drone to reboot successfully.

```
airsdk build
airsdk install --default
# wait for around 30 seconds
python takeoff.py
```
