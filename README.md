# AWS Utils Library for ROS2

## Overview
This is the common library for all of AWS RoboMaker ROS2 packages.
It builds on top of the generic interfaces defined by [utils-common](https://github.com/aws-robotics/utils-common) to provide ROS2-specific helper functions and classes implementations for things such as logging and parameter loading.

### License
The source code is released under an [Apache 2.0].

**Author**: AWS RoboMaker<br/>
**Affiliation**: [Amazon Web Services (AWS)]<br/>
**Maintainer**: AWS RoboMaker, ros-contributions@amazon.com

### Supported ROS2 Distributions
- Dashing

### Build status

* Travis CI: [![Build Status](https://travis-ci.org/aws-robotics/utils-ros2.svg?branch=master)](https://travis-ci.org/aws-robotics/utils-ros2)
 * ROS build farm:
   * ROS Dashing @ u18.04 Bionic [![Build Status](http://build.ros2.org/job/Dbin_uB64__aws_ros2_common__ubuntu_bionic_amd64__binary/badge/icon)](http://build.ros2.org/job/Dbin_uB64__aws_ros2_common__ubuntu_bionic_amd64__binary)

[Amazon Web Services (AWS)]: https://aws.amazon.com/
[Apache 2.0]: https://aws.amazon.com/apache-2-0/


## Installation

### Binaries
Pending bloom-release. In the future, you'd be able to install the latest released version of this package using the following command on Ubuntu:

        sudo apt-get update
        sudo apt-get install -y ros-$ROS_DISTRO-aws-ros2-common

### Building from Source

To build from source you'll need to create a new workspace, clone and checkout the latest release branch of this repository, install all the dependencies, and compile. If you need the latest development features you can clone from the `master` branch instead of the latest release branch. While we guarantee the release branches are stable, __the `master` should be considered to have an unstable build__ due to ongoing development. 

- Create a ROS workspace and a source directory: `mkdir -p ~/ros-workspace/src`

- Clone the package into the source directory

        cd ~/ros-workspace/src
        git clone https://github.com/aws-robotics/utils-ros2.git -b release-latest

- Install dependencies

        cd ~/ros-workspace 
        sudo apt-get update && rosdep update
        rosdep install --from-paths src --ignore-src -r -y
        
_Note: If building the master branch instead of a release branch you may need to also checkout and build the master branches of the packages this package depends on._

- Build the packages

        cd ~/ros-workspace && colcon build

- Configure ROS library path

        source ~/ros-workspace/install/local_setup.bash
