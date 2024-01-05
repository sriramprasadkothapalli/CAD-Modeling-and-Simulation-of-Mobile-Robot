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

![p1 gzrviz](https://github.com/sriramprasadkothapalli/CAD-Modeling-and-Simulation-of-Mobile-Robot/assets/143056659/b2f80ed4-237b-4795-b6e2-cc142b9c72c8)

- To make Robot move to [10,10] location, a proportional controller is used, to make the robot move, run the below commands
```
source WS_DIRECTORY/install/setup.bash
ros2 launch toy_car gazebo.launch.py
ros2 run toy_car p_controller.py
```
## Results 
- [Teleop](https://drive.google.com/file/d/141weB6x8F-j0YxN88nPRulcDrmmHv0vg/view?usp=sharing)
- [Proportional Controller](https://drive.google.com/file/d/1JutA0L2NpVbp9G4MeJvpsdbmqqcM7HtV/view?usp=sharing)

