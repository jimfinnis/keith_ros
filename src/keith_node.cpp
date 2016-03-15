/**
 * @file keith_node.cpp
 * @brief  Brief description of file.
 *
 */
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"

// this is sheer lunacy, but it makes sense.
// The rover code is in "keith/control" in the user's home dir.

#include "../../../../keith/control/rover.h"

Rover r;
class MyStatusListener: public StatusListener {
    virtual void onMessage(const char *str){
        printf("MESSAGE: %s\n",str);
    }
};
static MyStatusListener stat;

static float leftSpeed=0,rightSpeed=0;
void leftCallback(const std_msgs::Float32::ConstPtr& msg){
    leftSpeed = msg->data;
}
void rightCallback(const std_msgs::Float32::ConstPtr& msg){
    rightSpeed = msg->data;
}


int main(int argc,char *argv[]){
    ros::init(argc,argv,"keith_node");
    ros::NodeHandle n;
    
    ros::Subscriber subl = n.subscribe<std_msgs::Float32>
          ("leftmotors",100,leftCallback);
    ros::Subscriber subr = n.subscribe<std_msgs::Float32>
          ("rightmotors",100,rightCallback);
    
    ros::Publisher sonarpub = n.advertise<std_msgs::Float32MultiArray>
          ("sonar",100);
    
    
    r.attachCommsListener(&stat);
    if(!r.init("/dev/ttyACM0")){
        ROS_ERROR("Cannot connect");
        return 1;
    }
    r.resetSlaveExceptions();
    
    ros::Rate loop_rate(5);
    
    while(ros::ok()){
        r.update();
        MasterData *m= r.getMasterData();
        
        std_msgs::Float32MultiArray sonarOut;
        sonarOut.data.clear();
        for(int i=0;i<3;i++)
            sonarOut.data.push_back(m->sonarDists[i]);
        sonarpub.publish(sonarOut);
        
        SlaveData *slave = r.getSlaveData(0);
        slave->setSpeed(0,leftSpeed);
        slave->setSpeed(1,rightSpeed);
        slave = r.getSlaveData(1);
        slave->setSpeed(0,leftSpeed);
        slave->setSpeed(1,rightSpeed);
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    ros::spin();
    return 0;
}
