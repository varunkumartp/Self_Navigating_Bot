# Self-Navigating Bot using Ultrasonic Sensors
Open source Self-Navigating Bot implemented in ROS using Kinect sensor and ultrasonic sensors. The objects that will be in the range of the ultrasonic sensors will also be considered in the local costmap and will be avoided.

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
# open three terminals
cd ~/project

# source all the terminals
source devel/setup.bash

# Terminal 1
roslaunch navigator_bringup robot_standalone.launch

# Terminal 2
roslaunch navigator_sonar auto.launch

# Terminal 3
roslaunch navigator_bringup view_navigation.launch
```

### Simultaneous Mapping and Autonomous Navigation
```bash
# open three terminals
cd ~/project

# source all the terminals
source devel/setup.bash

# Terminal 1
roslaunch navigator_bringup robot_standalone.launch

# Terminal 2
roslaunch navigator_sonar sman.launch

# Terminal 3
roslaunch navigator_bringup view_navigation.launch
```


