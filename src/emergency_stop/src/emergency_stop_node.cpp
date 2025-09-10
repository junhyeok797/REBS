#include <ros/ros.h>

#include "emergency_stop.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "emergency_stop_node");

    EmergencyStop node;
    node.spin();

    return 0;
}