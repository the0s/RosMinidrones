# RosMinidrones
Control a number of Bluetooth minidrones using ROS


## Demo
1. run this to connect to drones. Make sure you put the correct bluetooth address in the launch file. The Delay is added because of issues when multiple devices try to connect to bluetooth.

```
roslaunch ros_minidrones swarmROs.launch
```

2. When connection is estasblished with all drones 
a. run `teleMamboROs.py` to teleop the drones with the keyboard. 
b. run `testRosMamboInput.py` to preview a demo with all drones

In both files set the variable 'mambos' to the correct address of each drone


3. Hopefully if you run rviz you can see an approximation of the (noisy) state of the drones.
