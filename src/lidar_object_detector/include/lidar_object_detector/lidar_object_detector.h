#ifndef LIDAR_OBJECT_DETECTOR_H
#define LIDAR_OBJECT_DETECTOR_H

#include <iostream>
#include <cmath>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

struct Point2D
{
    float x, y;
};

class LidarClusterDetector
{
public:
    LidarClusterDetector();

private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher cluster_pub_;
    ros::Publisher pedestrian_pub_;
    ros::Publisher min_distance_pub_;

    const float detection_radius_ = 1.5;        // 감지 범위
    const float cluster_max_width_ = 0.02;      // 클러스터링 크기
    const float point_pair_threshold_ = 0.15;   // 포인트 간 거리(클러스터링 기준)

    float euclideanDistance(const Point2D& p1, const Point2D& p2);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
};

#endif  // LIDAR_OBJECT_DETECTOR_H