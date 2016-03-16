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

void setParams(){
    float temp;
    
    MotorParams p;
    p.pgain = ros::param::get("~pgain",temp) ? temp : 0.02;
    p.igain = ros::param::get("~igain",temp) ? temp : 0.01;
    p.dgain = ros::param::get("~dgain",temp) ? temp : 0;
    p.icap = ros::param::get("~icap",temp) ? temp : 100;
    p.idecay = ros::param::get("~idecay",temp) ? temp : 0.9;
    p.deadzone = ros::param::get("~deadzone",temp) ? temp : 0;
    
    SlaveData *sd = r.getSlaveData(0);
    sd->sendMotorParams(0,&p);
    sd->sendMotorParams(1,&p);
    sd = r.getSlaveData(1);
    sd->sendMotorParams(0,&p);
    sd->sendMotorParams(1,&p);
}

#define SMOOTHRISING 0.2f
#define SMOOTHFALLING 0.02f

class Smoother {
    float val;
public:
    Smoother(){
        val=0;
    }
    float run(float f){
        if(f>val){
            val = SMOOTHRISING * f + (1.0f-SMOOTHRISING)*val;
        } else {
            val = SMOOTHFALLING * f + (1.0f-SMOOTHFALLING)*val;
        }
    }
};
    


int main(int argc,char *argv[]){
    ros::init(argc,argv,"keith_node");
    ros::NodeHandle n;
    
    ros::Subscriber subl = n.subscribe<std_msgs::Float32>
          ("leftmotors",100,leftCallback);
    ros::Subscriber subr = n.subscribe<std_msgs::Float32>
          ("rightmotors",100,rightCallback);
    
    ros::Publisher sonarpub = n.advertise<std_msgs::Float32MultiArray>
          ("sonar",100);
    ros::Publisher sonarrawpub = n.advertise<std_msgs::Float32MultiArray>
          ("sonar/raw",100);
    
    
    r.attachCommsListener(&stat);
    if(!r.init("/dev/ttyACM0")){
        ROS_ERROR("Cannot connect");
        return 1;
    }
    r.resetSlaveExceptions();
    
    setParams();
    
    ros::Rate loop_rate(5);
    Smoother sonarsmoothers[3];
    
    while(ros::ok()){
        r.update();
        MasterData *m= r.getMasterData();
        
        // output raw sonar
        std_msgs::Float32MultiArray sonarOut;
        sonarOut.data.clear();
        for(int i=0;i<3;i++)
            sonarOut.data.push_back(m->sonarDists[i]);
        sonarrawpub.publish(sonarOut);
        
        // now smooth them
        sonarOut.data.clear();
        for(int i=0;i<3;i++)
            sonarOut.data.push_back(sonarsmoothers[i].run(m->sonarDists[i]));
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
