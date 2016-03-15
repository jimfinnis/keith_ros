# keith_ros
ROS package for controlling Big Keith. Contains files both for Keith and the controlling PC.

Clone into "keith" in the catkin workspaces on both systems.

##Nodes
**keith_node** is the controlling node for the robot, allowing
motor speeds to be set and sensors to be read.

###Topics subscribed to
- **/leftmotors** (std\_msgs/Float32): required speed for left motors
- **/rightmotors** (std\_msgs/Float32): required speed for right motors

###Topics published
- **/sonar** (std\_msgs/Float32MultiArray): array of sonar readings

###Motor control parameters

- \~pgain
- \~igain
- \~dgain
- \~icap
- \~idecay
- \~deadzone

##Launch
- **camera.launch** just fires up the robot's camera and a local image viewer when run on the PC. 
