# Self-Navigating Bot

## Prerequisites
Install the [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) framework for Ubuntu 20.04.

## Setup
Create a workspace
```
mkdir -p ~/project/src
cd ~/project/src
```
Clone the git repository into the src folder
```
# Git must be installed
git clone https://github.com/varun-kumar-tp/navigator_bringup.git
# Installs the dependencies required for successful catkin_make
navigator_bringup/dependencies.sh
# Clones the required packages from github into the src folder 
navigator_bringup/packages.sh
cd ..
# Builds the code
catkin_make
# If no errors run
source devel/setup.bash
```