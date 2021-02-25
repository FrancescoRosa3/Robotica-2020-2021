#include "ros/ros.h"
#include "std_msgs/String.h"
#include <controller_pkg/Sensors.h>

void topicCallback(const controller_pkg::Sensors msg){
    ROS_INFO_STREAM("I heard " << msg);
}

int main(int argc, char **argv){
    
    ros::init(argc, argv, "controller");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<controller_pkg::Sensors>("sensors", 10, topicCallback);

    ros::Rate loop_rate(1);

    ros::spin();

    return 0;
}