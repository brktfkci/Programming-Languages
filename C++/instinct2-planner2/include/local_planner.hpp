#ifndef LOCAL_PLANNER_HPP
#define LOCAL_PLANNER_HPP

#include "pointcloud_processor.hpp"
#include "waypoint_generator.hpp"
#include "star_planner.hpp"
#include "constants.hpp"
#include "histogram.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <px4_msgs/msg/obstacle_distance.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <Eigen/Geometry>

class LocalPlannerNode : public rclcpp::Node {
public:
    LocalPlannerNode();
    void initialize();


private:
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void positionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg);
    void computeVFH();
    int findStartPoint(const std::vector<Eigen::Vector3f>& waypoints, const Eigen::Vector3f& position);
    float computeYaw(float x1, float y1, float x2, float y2);
    float computeDistanceToPoint(const Eigen::Vector3f& position, const Eigen::Vector3f& point, bool only_2d);
    float normalizeYaw(float yaw);
    void setPositionMode();
    void sendVelocityCommand(double vx, double vy, double vz, float target_yaw);
    void sendLandCommand();

    // Члены класса
    std::unique_ptr<PointCloudProcessor> pointcloud_processor_;
    std::shared_ptr<Histogram> histogram_;
    std::unique_ptr<WaypointGenerator> waypoint_generator_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
    rclcpp::Publisher<px4_msgs::msg::ObstacleDistance>::SharedPtr obstacle_distance_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr histogram_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsampled_cloud_pub_;

    double goal_x_, goal_y_, goal_z_;
    double fov_h_, fov_v_;
    double robot_radius_;
    double safe_distance_;
    double speed_;
    double current_x_, current_y_, current_z_;
    double yaw_;
    bool position_received_ = false;
    Eigen::Vector3f prev_waypoint_ = Eigen::Vector3f::Zero();

    static constexpr double DEFAULT_HEIGHT = 2.0;
    static constexpr double DEFAULT_FOV_HORIZONTAL = 90.0;
    static constexpr double DEFAULT_FOV_VERTICAL = 60.0;
    static constexpr double DEFAULT_ROBOT_RADIUS = 0.3;
    static constexpr double DEFAULT_SAFE_DISTANCE = 5.0;
    static constexpr double DEFAULT_SPEED = 1.0;
    static constexpr double GOAL_TOLERANCE = 0.5;
    static constexpr double MAX_YAW_RATE = 30.0;
    static constexpr int HISTOGRAM_SIZE = 72;  // Предполагается, можно уточнить
    static constexpr double HISTOGRAM_RESOLUTION = 360.0 / HISTOGRAM_SIZE;
};

#endif  // LOCAL_PLANNER_HPP