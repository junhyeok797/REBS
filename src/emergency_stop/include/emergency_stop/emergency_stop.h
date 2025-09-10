#ifndef EMERGENCY_STOP_H
#define EMERGENCY_STOP_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

class EmergencyStop
{
public:
    EmergencyStop();

    void spin();

private:
    ros::NodeHandle nh_;
    ros::Subscriber pedestrian_sub_;
    ros::Subscriber yolo_sub_;
    ros::Subscriber obstacle_distance_sub_;
    ros::Publisher emergency_pub_;

    bool pedestrian_detected_;
    bool yolo_detected_;
    float closest_distance_;

    bool emergency_active_;
    ros::Time emergency_trigger_time_;
    bool pedestrian_active_;
    ros::Time pedestrian_trigger_time_;

    const float distance_threshold_ = 0.01;  // 조절

    void pedestrianCallback(const std_msgs::Bool::ConstPtr& msg);
    void yoloCallback(const std_msgs::Bool::ConstPtr& msg);
    void distanceCallback(const std_msgs::Float32::ConstPtr& msg);
    bool shouldTriggerEmergencyStop();
};

#endif  // EMERGENCY_STOP_H