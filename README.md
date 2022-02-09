# AWS Utils Library for ROS2

## Overview
This is the common library for all of AWS RoboMaker ROS2 packages.
It builds on top of the generic interfaces defined by [utils-common](https://github.com/aws-robotics/utils-common) to provide ROS2-specific helper functions and classes implementations for things such as logging and parameter loading.

### License
The source code is released under an [Apache 2.0].

**Author**: AWS RoboMaker<br/>
**Affiliation**: [Amazon Web Services (AWS)]<br/>

RoboMaker cloud extensions rely on third-party software licensed under open-source licenses and are provided for demonstration purposes only. Incorporation or use of RoboMaker cloud extensions in connection with your production workloads or commercial product(s) or devices may affect your legal rights or obligations under the applicable open-source licenses. License information for this repository can be found [here](https://github.com/aws-robotics/utils-ros2/blob/master/LICENSE). AWS does not provide support for this cloud extension. You are solely responsible for how you configure, deploy, and maintain this cloud extension in your workloads or commercial product(s) or devices.

### Supported ROS2 Distributions
- Dashing

## Installation

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
