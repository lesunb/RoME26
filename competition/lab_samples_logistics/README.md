# Lab Samples Logistics Mission Example

## Dependencies

ROS2 Humble
Gazebo Ignition Fortress
ROS2 Humble turtlebot4 packages

## Installation

1. Install ROS2 Humble for Ubuntu 22.04 [ROS2 Humble Documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
2. Follow the turtlebot4 documentation for simulations: [Turtlebot4 Documentation](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html)
3. Install the packages:

```bash
cd ros_pkg
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Running the Simulation

1. Run gazebo:

```bash
ros2 launch gazebo_simulation tb4_sim_bringup.launch.py
```

2. Set robot initial pose for navigation (can also be done through RVIZ):

```bash
ros2 run gazebo_simulation set_initial_pose --ros-args -p namespace:=robot1 x:=0.0 y:=0.0
```

3. Execute behavior tree

```bash
ros2 launch behavior_trees execute_bt.launch.py bt:=/path/to/behavior_tree
```

Example:

```bash
ros2 launch behavior_trees execute_bt.launch.py bt:=/sim_ws/src/behavior_trees/behavior_trees/samples_delivery.xml
```
