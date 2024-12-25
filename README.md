# AHSMC_EKF-for-Quadrotors-UAVs-on-Gazebo-ROS-2
This repository provides a methodology for controlling Quadrotor Unmanned Aerial Vehicles (UAVs) on Gazebo/Ros 2 Humble

## Software Requirements & Setup

The custom controller is designed in:

- Ubuntu 22.04
- ROS2 Humble
- Gazebo 11
- Xarco-ROS-Humble (sudo apt install ros-humble-xacro)

Follow these commands to install the package:

```shell
# Step 1: Create and build a colcon workspace:
$ mkdir -p ~/dev_ws/src
$ cd ~/dev_ws/
$ colcon build
$ echo "source ~/dev_ws/devel/setup.bash" >> ~/.bashrc

# Step 2: Clone this repo into your workspace
$ cd ~/dev_ws/src
$ git clone --recursive https://github.com/aralab-unr/Custom-controller-for-the-cube-drone.git

# Step 3: Build the catkin workspace for this package
$ cd ~/dev_ws
$ catkin_make
```
