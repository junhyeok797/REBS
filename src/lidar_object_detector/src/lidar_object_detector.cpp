#include "lidar_object_detector.h"

LidarClusterDetector::LidarClusterDetector()
{
    ROS_INFO("Start Pedestrian Detection");
    scan_sub_ = nh_.subscribe("/scan", 10, &LidarClusterDetector::scanCallback, this);
    cluster_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/detected_object", 10);
    pedestrian_pub_ = nh_.advertise<std_msgs::Bool>("/pedestrian_detected", 10);
    min_distance_pub_ = nh_.advertise<std_msgs::Float32>("/closest_obstacle_distance", 10);
}

float LidarClusterDetector::euclideanDistance(const Point2D& p1, const Point2D& p2)
{
    return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

void LidarClusterDetector::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    std::vector<Point2D> points;

    const float front_roi_min = -M_PI / 4; // -45도
    const float front_roi_max = M_PI / 4;  // +45도
    const float rear_roi_min = 3.0f * M_PI / 4.0f; // +135도
    const float rear_roi_max = 5.0f * M_PI / 4.0f; // +225도
    const float roi_min = -5.0f * M_PI / 4.0f; // -225도 ≈ -3.927 rad
    const float roi_max = -3.0f * M_PI / 4.0f; // -135도 ≈ -2.356 rad

    // 1. ROI
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
    {
        float r = scan_msg->ranges[i];
        if (std::isnan(r) || std::isinf(r) || r > detection_radius_)
            continue;
        
        float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        // if (angle < front_roi_min || angle > front_roi_max)
        //     continue;
        if (angle < rear_roi_min || angle > rear_roi_max)
            continue;
        // if (angle < roi_min || angle > roi_max)
        //     continue;

        Point2D p;
        p.x = r * std::cos(angle);
        p.y = r * std::sin(angle);
        points.push_back(p);
    }

    std::vector<std::vector<Point2D>> clusters;
    std::vector<Point2D> current_cluster;

    // 2. 클러스터링
    for (size_t i = 0; i < points.size(); ++i)
    {
        if (current_cluster.empty())
            current_cluster.push_back(points[i]);
        else
        {
            float dist = euclideanDistance(points[i], current_cluster.back());
            if (dist < point_pair_threshold_)
            {
                current_cluster.push_back(points[i]);
                // ROS_INFO("point_pari_dist: %.4f", dist);    // 디버깅 용도
            }
            else
            {
                clusters.push_back(current_cluster);
                current_cluster.clear();
                current_cluster.push_back(points[i]);
            }
        }
    }
    if (!current_cluster.empty())
        clusters.push_back(current_cluster);

    // 3. 클러스터 필터링 및 출력 메시지 구성
    sensor_msgs::PointCloud output;
    output.header.stamp = scan_msg->header.stamp;
    output.header.frame_id = scan_msg->header.frame_id;

    bool pedestrian_detected = false;
    float min_cluster_distance = std::numeric_limits<float>::infinity();  // 클러스터 중심까지 최소 거리

    for (const auto& cluster : clusters)
    {
        if (cluster.size() < 2) continue;

        float cluster_length = euclideanDistance(cluster.front(), cluster.back());
        if (cluster_length <= cluster_max_width_)
        {
            // 중심점 계산
            float sum_x = 0, sum_y = 0;
            for (const auto& p : cluster)
            {
                sum_x += p.x;
                sum_y += p.y;
            }
            geometry_msgs::Point32 center;
            center.x = sum_x / cluster.size();
            center.y = sum_y / cluster.size();
            center.z = 0.0;
            output.points.push_back(center);

            float dist_to_lidar = std::hypot(center.x, center.y);
            if (dist_to_lidar < min_cluster_distance)
                min_cluster_distance = dist_to_lidar;

            pedestrian_detected = true; // 물체 감지됨
            ROS_INFO("Cluster size: %lu, length: %.4f", cluster.size(), cluster_length);    // 디버깅 용도
        }
    }

    // 4. 퍼블리시
    cluster_pub_.publish(output);

    std_msgs::Bool pedestrian_msg;
    pedestrian_msg.data = pedestrian_detected;
    pedestrian_pub_.publish(pedestrian_msg);

    std_msgs::Float32 min_distance_msg;
    min_distance_msg.data = std::isinf(min_cluster_distance) ? -1.0 : min_cluster_distance;
    min_distance_pub_.publish(min_distance_msg);
}