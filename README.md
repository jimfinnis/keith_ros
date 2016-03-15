# keith_ros
ROS package for controlling Big Keith. Contains files both for Keith and the controlling PC.

Clone into "keith" in the catkin workspaces on both systems.

##Nodes
- **raspicam\_node** is a modified clone of the raspicam/raspicam\_node camera system. Mods are: automatic startup, some extra reporting. **Please specify a framerate of around 5** to reduce power consumption on the pi,

##Launch
- **camera.launch** just fires up the robot's camera and a local image viewer when run on the PC. 
