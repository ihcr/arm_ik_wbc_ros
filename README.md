arm_ik_wbc_ros
------

### What it is
A ROS interface for the arm_ik_wbc repo. This provides the launch files and messages required to run the arm in simulation and on hardware.

### Dependency Installation

1. Check dependancies of the packages listed bellow.

### Package Usage
Replace `<work_folder>` with a specific workspace name, such as rob_ws.
```
mkdir -p <work_folder>/src
cd <work_folder>/src
git clone https://github.com/ihcr/commutils.git
git clone https://github.com/ihcr/limbsim.git
git clone https://github.com/ihcr/yamlutils.git
git clone https://github.com/ihcr/interbotix_ihrc.git
git clone https://github.com/ihcr/arm_ik_wbc.git
git clone https://github.com/ihcr/arm_ik_wbc_ros.git
colcon build
```
Once the code has been compiled, you can source .bash file in `install/setup.bash`
```
. install/setup.bash
```

### Launching the demo WBC node:
```
cd <work_folder>/
roslaunch ./src/arm_ik_wbc_ros/launch/demo_launch.launch robot_model:=vx300s
```

### Launching the hardware node:
```
cd <work_folder>/
roslaunch ./src/arm_ik_wbc_ros/launch/interbotix_wbc.launch robot_model:=vx300s
```

### Topics
Once launched, the input to the wbc can be provided to this topic, although it should be noted that no input topic is active during the demo:
```
/<config_name>_WBC/input
```
If the hardware node is launched, the output is sent straight to the interbotix SDK node:
```
/<config_name>/commands/joint_group
```
If the simulation node is launched, the output is send to:
```
/<config_name>_WBC/data
```

### License and Copyrights

Copyright (c) 2021, University of Leeds.
BSD 3-Clause License
