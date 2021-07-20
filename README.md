# Self-Navigating Bot
Open source Self-Navigating Bot implemented in ROS using Kinect sensor.

## Prerequisites
Install the [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) framework for Ubuntu 20.04.

## Setup
### Create a workspace
``` bash
mkdir -p ~/project/src
cd ~/project/src
```
### Clone the git repository into the src folder
```bash
# Git must be installed
git clone https://github.com/varun-kumar-tp/Self_Navigating_Bot.git

# Installs the dependencies required for successful catkin_make
Self_Navigating_Bot/dependencies.sh

# Clones the required packages from github into the src folder 
Self_Navigating_Bot/packages.sh
cd ..

# Build the code
catkin_make

# If no errors run
source devel/setup.bash
```

## Implementation

### Simultaneous Localization And Mapping - SLAM
```bash
# open four terminals
cd ~/project

# source all the terminals
source devel/setup.bash

# Terminal 1
roslaunch navigator_bringup robot_standalone.launch

# Terminal 2
roslaunch navigator_slam mapping.launch

# Terminal 3
roslaunch navigator_bringup view_navigation.launch

# Terminal 4 - Run the following command after building the map
rosrun map_server map_saver -f ~/project/src/Self_Navigating_Bot/navigator_navigation/maps/map_name    
```

### Autonomous Navigation
```bash
# open two terminals
cd ~/project

# source the terminals
source devel/setup.bash

# Terminal 1
roslaunch navigator_bringup robot_standalone.launch

# Terminal 2
roslaunch navigator_navigation auto.launch
```

### Simultaneous Mapping and Autonomous Navigation
```bash
# open two terminals
cd ~/project

# source all the terminals
source devel/setup.bash

# Terminal 1
roslaunch navigator_bringup robot_standalone.launch

# Terminal 2
roslaunch navigator_navigation sman.launch
```