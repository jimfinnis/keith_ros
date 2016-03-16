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
- **/sonar** (std\_msgs/Float32MultiArray): array of smoothed, remapped readings
- **/sonar/raw** (std\_msgs/Float32MultiArray): array of raw sonar readings

###Motor control parameters

- \~pgain
- \~igain
- \~dgain
- \~icap
- \~idecay
- \~deadzone

###Sonar remapping
The sonar readings are smoothed and remapped into occlusion values, which are from 0 (clear) to 1 (occluded).
The smoothing is done such that a falling sonar reading will be much less smoothed. Thus, occluding objects will register quickly and die away slowly.

The remapping is currently 5cm=occluded(1), 40cm=clear(0) with a linear range between.

##Launch (to be run on the PC - don't start roscore on the robot!)
- **all.launch** includes camera.launch and ctrl.launch
- **camera.launch** just fires up the robot's camera and a local image viewer when run on the PC. 
- **ctrl.launch** starts keith\_node on the robot
