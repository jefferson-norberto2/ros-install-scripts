#!/bin/bash

echo "Executado arquivo bash"

sudo apt update

sudo apt install lsb-release -y # if you haven't already installed lsb-release

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl gnupg -y # if you haven't already installed curl or gnupg

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt-get update

sudo apt-get install git wget ipython3 python3-rosinstall ros-noetic-desktop-full python3-catkin-tools python3-rosdep python-is-python3 ros-noetic-actionlib-tools ros-noetic-moveit-commander -y

source /opt/ros/noetic/setup.bash

mkdir ~/tiago_public_ws

cd ~/tiago_public_ws

wget https://raw.githubusercontent.com/pal-robotics/tiago_tutorials/noetic-devel/tiago_public-noetic.rosinstall

rosinstall src /opt/ros/noetic tiago_public-noetic.rosinstall

rosdep update

rosdep install -y --from-paths src --ignore-src --rosdistro noetic --skip-keys "urdf_test omni_drive_controller orocos_kdl pal_filters libgazebo9-dev pal_usb_utils speed_limit_node camera_calibration_files pal_moveit_plugins pal_startup_msgs pal_local_joint_control pal_pcl_points_throttle_and_filter current_limit_controller hokuyo_node dynamixel_cpp pal_moveit_capabilities pal_pcl dynamic_footprint gravity_compensation_controller pal-orbbec-openni2 pal_loc_measure pal_map_manager ydlidar_ros_driver"

source /opt/ros/noetic/setup.bash

catkin build -DCATKIN_ENABLE_TESTING=0 -j $(expr `nproc` / 2)

source ~/tiago_public_ws/devel/setup.bash

roslaunch tiago_gazebo tiago_gazebo.launch public_sim:=true end_effector:=pal-gripper
