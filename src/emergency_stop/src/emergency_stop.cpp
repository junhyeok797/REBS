#include "emergency_stop.h"

EmergencyStop::EmergencyStop()
{
    pedestrian_sub_ = nh_.subscribe("/pedestrian_detected", 1, &EmergencyStop::pedestrianCallback, this);
    yolo_sub_ = nh_.subscribe("/yolo_topic", 1, &EmergencyStop::yoloCallback, this);
    obstacle_distance_sub_ = nh_.subscribe("/closest_obstacle_distance", 1, &EmergencyStop::distanceCallback, this);
    emergency_pub_ = nh_.advertise<std_msgs::Bool>("/emergency_stop", 1);

    pedestrian_detected_ = false;
    yolo_detected_ = false;
    closest_distance_ = std::numeric_limits<float>::infinity();
}

void EmergencyStop::spin()
{
    ros::Rate rate(10); // 10Hz

    while (ros::ok())
    {
        ros::Time now = ros::Time::now();
        std_msgs::Bool msg;

        bool emergency_condition = shouldTriggerEmergencyStop();
        
        if (emergency_condition)
        {
            emergency_active_ = true;
            emergency_trigger_time_ = now;
            msg.data = true;
        }
        else if (emergency_active_)
        {
            ros::Duration elapsed = now - emergency_trigger_time_;
            if (elapsed.toSec() < 5.0)
                msg.data = true;
            else
            {
                emergency_active_ = false;
                msg.data = false;
            }
        }
        else
        {
            msg.data = false;
        }

        emergency_pub_.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }
}

void EmergencyStop::pedestrianCallback(const std_msgs::Bool::ConstPtr& msg)
{
    ros::Time now = ros::Time::now();

    bool detected_condition = msg->data;
    if (detected_condition)
    {
        pedestrian_active_ = true;
        pedestrian_trigger_time_ = now;
        pedestrian_detected_ = true;
    }
    else if (pedestrian_active_)
    {
        ros::Duration elapsed = now - pedestrian_trigger_time_;
        if (elapsed.toSec() < 5.0)
            pedestrian_detected_ = true;
        else
        {
            pedestrian_active_ = false;
            pedestrian_detected_ = false;
        }
    }
    else
    {
        pedestrian_detected_ = false;
    }

    if (pedestrian_detected_ == true)
        ROS_WARN("LiDAR Pedestrian Detect");
}

void EmergencyStop::yoloCallback(const std_msgs::Bool::ConstPtr& msg)
{
    yolo_detected_ = msg->data;

    if (yolo_detected_ == true)
        ROS_WARN("YOLO Pedestrian Detect");
}

void EmergencyStop::distanceCallback(const std_msgs::Float32::ConstPtr& msg)
{
    closest_distance_ = msg->data;

    ROS_INFO("closest_distance: %f", closest_distance_);
}

bool EmergencyStop::shouldTriggerEmergencyStop()
{
    return pedestrian_detected_ && yolo_detected_ && (closest_distance_ <  distance_threshold_);
    // return pedestrian_detected_ && (closest_distance_ <  distance_threshold_);
}