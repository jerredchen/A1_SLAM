# A1 SLAM: Quadruped SLAM using the A1's onboard sensors

![a1_slam_trajectory](media/a1_slam_trajectory.gif)

**Note:** This repository is still under active development.

A1 SLAM is a rospackage that brings real-time SLAM capabilities utilizing factor graph optimization to Unitree's A1 quadruped. This package was designed around the onboard sensors on the A1 and aims for easy, convenient installation. In addition, this package is compatible with sensors that are not default to the A1.

This package has been tested using ROS Noetic on Ubuntu 20.04.

## Installation

If you haven't install ROS already, please follow the steps for [ROS installation](http://wiki.ros.org/ROS/Installation).

```
# Install GTSAM as dependency if necessary
pip install --user gtsam
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/jerredchen/A1_SLAM.git --recursive
cd ..
catkin_make
```

## Documentation and Usage

For more information on using A1 SLAM, please see the [documentation](http://a1-slam.rtfd.io).
