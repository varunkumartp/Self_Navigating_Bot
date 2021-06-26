# Self-Navigating Bot

## Prerequisites
Install the [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) framework for Ubuntu 20.04.

## Setup
Create a workspace
``` bash
mkdir -p ~/project/src
cd ~/project/src
```
Clone the git repository into the src folder
```bash
# Git must be installed
git clone https://github.com/varun-kumar-tp/navigator_bringup.git
# Installs the dependencies required for successful catkin_make
navigator_bringup/shell_scripts/dependencies.sh
# Clones the required packages from github into the src folder 
navigator_bringup/shell_scripts/packages.sh
cd ..
# Builds the code
catkin_make
# If no errors run
source devel/setup.bash
```