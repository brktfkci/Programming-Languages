#pragma once
#include "constants.hpp"
#include "histogram.hpp"
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <memory>

class PointCloudProcessor {
public:
    PointCloudProcessor(std::shared_ptr<Histogram> histogram, 
        double fov_h, double fov_v, 
        double safe_distance, double yaw,
        rclcpp::Logger logger,
        rclcpp::Node::SharedPtr node,
        rclcpp::Publisher<px4_msgs::msg::ObstacleDistance>::SharedPtr obstacle_pub,
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr histogram_image_pub,
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub,
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsampled_cloud_pub,
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_cloud_pub);

    void processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void publishTimerCallback();
    const std::vector<uint16_t>& getDistances() const;
    void updateYaw(double new_yaw);

private:
    bool isInsideFOV(float x, float y, float z);
    float normalizeYaw(float yaw);
    void publishObstacleDistance();
    void publishHistogramImage();
    void publishLaserScan();
    void publishDownsampledCloud(const sensor_msgs::msg::PointCloud2& cloud);

    std::shared_ptr<Histogram> histogram_;
    std::vector<uint16_t> distances_;
    double fov_h_, fov_v_;
    double safe_distance_;
    double yaw_;
    rclcpp::Logger logger_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<px4_msgs::msg::ObstacleDistance>::SharedPtr obstacle_distance_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr histogram_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsampled_cloud_pub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_cloud_pub_;  // Новый член
    void publishRawCloud(const sensor_msgs::msg::PointCloud2& cloud);  // Новый метод
};