#include <iostream>

#include <ros/ros.h>

#include "lidar_object_detector.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_cluster_detector_node");

    LidarClusterDetector detector;
    ros::spin();
    
    return 0;
}