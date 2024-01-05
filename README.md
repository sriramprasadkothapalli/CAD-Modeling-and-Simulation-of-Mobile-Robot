# CAD Modeling and Simulation of Mobile-Robot

## Mobile Robot image:
The mobile robot is modeled in Solidworks
![p1 sw](https://github.com/sriramprasadkothapalli/CAD-Modeling-and-Simulation-of-Mobile-Robot/assets/143056659/d3dec223-e276-484a-81ab-224d7cade3ea)
### Steps to run the project:

- Create a ROS2 Workspace
- git clone the package in the ros2 workspace

```
cd ros2_ws/src
git clone "link to repository"
```
- Source your bashrc and ROS2
- Build you package and source your workspace

```
cd ros2_ws/
source /opt/ros/"ROS2_DISTRO"/setup.bash
colcon build
source "WS_DIRECTORY"/install/setup.bash
```
- To spawn and robot in compettive world and run Teleop
In one terminal, run the following command to open the competetion world in Gazebo
```
ros2 launch toy_car competition.launch.py
```
In second terminal, run the below command to run the teleoperation
```
ros2 run toy_car teleop.py
```

- Mobile robot in the Gazebo and Rviz envirmonment
- 

