# ocs2_cmake

## Introduction

This project is based on [ocs2](https://github.com/leggedrobotics/ocs2). osc2 is an excellent optimal control package, including setting up optimal control problems, converting them into nonlinear optimization problems, and the necessary MPC/MRT interface for robotics, especially legged robots, which are classified as switched system. However, the original project uses `catkin` as the build system, uses ROS as communication middle-ware and parameter server, uses ROS package management system, which makes it closely coupled with ROS, while ROS, specifically, ROS1, is depend on Ubuntu up to 20.04 (Focal Fossa), i.e. ROS noetic. However, let alone ROS noetic and Focal Fossa are both near their end-of-life, the strong coupling is not friendly for deploy algorithms on other platform (i.e. cpu artechture, say x86, arm, risc-v) or operating system (boarder unix-like OS). Thanks to the opensource from the original ocs2, this work is made possible to change the build system to cmake, and leave the communication middle-ware interface flexible for users to select or implement.

## Install

Unfortunately, for now, you'll first need to install `eigen v3.3`, `boost > v1.71`, and clone and install the following:

```sh
# install dependencies
sudo apt install liburdfdom-dev liboctomap-dev libassimp-dev
# Clone hpp-fcl
git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git
# Then build and install
# Clone pinocchio
git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git
# Then build and install
```

Then do the regular build for this repo:

```sh
cd /path/to/this/repo
mkdir build && cd build
cmake .. # note cmake will fetch content from Internet!
make -j
```

We will try to remove the eigen, boost, hpp-fcl, and pinocchio by integrating them into the CMakeLists.txt as fetched contents in future versions. We will also have a version with all dependencies downloaded, in which case no internet is needed.

## Test

To demonstrate the mpc capability, we demonstrate a quadruped robot with centroid dynamics model by solving a trajectory optimization problem.
Please go to `ocs2_robotic_examples/ocs2_legged_robot_test/LeggedRobotSqpMpcNode.cpp` for details.

## Copyright Claim

This project is based on the original work by Farbod Farshidian (2017).
Modifications made by Yuntian Zhao in 2025.

Please see the LICENSE for details.