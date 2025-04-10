#include "local_planner.hpp"

LocalPlannerNode::LocalPlannerNode() : Node("local_planner_node") {
    // Объявление параметров
    declare_parameter("goal_x", 0.0);
    declare_parameter("goal_y", 0.0);
    declare_parameter("goal_z", DEFAULT_HEIGHT);
    declare_parameter("fov_horizontal", DEFAULT_FOV_HORIZONTAL);
    declare_parameter("fov_vertical", DEFAULT_FOV_VERTICAL);
    declare_parameter("robot_radius", DEFAULT_ROBOT_RADIUS);
    declare_parameter("safe_distance", DEFAULT_SAFE_DISTANCE);
    declare_parameter("speed", DEFAULT_SPEED);

    // Получение параметров
    goal_x_ = get_parameter("goal_x").as_double();
    goal_y_ = get_parameter("goal_y").as_double();
    goal_z_ = get_parameter("goal_z").as_double();
    fov_h_ = get_parameter("fov_horizontal").as_double();
    fov_v_ = get_parameter("fov_vertical").as_double();
    robot_radius_ = get_parameter("robot_radius").as_double();
    safe_distance_ = get_parameter("safe_distance").as_double();
    speed_ = get_parameter("speed").as_double();

    // Настройка QoS
    rclcpp::QoS qos(10);
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
    qos.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);

    // Инициализация подписчиков и паблишеров
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/points2", 10, std::bind(&LocalPlannerNode::pointcloudCallback, this, std::placeholders::_1));

    position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position", qos, std::bind(&LocalPlannerNode::positionCallback, this, std::placeholders::_1));

    command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
    velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/fmu/in/setpoint_velocity/cmd_vel", 10);
    obstacle_distance_pub_ = create_publisher<px4_msgs::msg::ObstacleDistance>("/fmu/in/obstacle_distance", 10);
    histogram_image_pub_ = create_publisher<sensor_msgs::msg::Image>("/planner/histogram_image", 10);
    laser_scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/planner/laser_scan", 10);
    raw_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("raw_pointcloud", 10);  // Исправлено имя
    downsampled_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("downsampled_pointcloud", 10);  // Исправлено имя

    // Инициализация объектов
    histogram_ = std::make_shared<Histogram>();
    waypoint_generator_ = std::make_unique<WaypointGenerator>(speed_, safe_distance_);

    setPositionMode();

    RCLCPP_INFO(this->get_logger(), "Local Planner Node started. Goal: [%.2f, %.2f, %.2f], FOV: [%.1f, %.1f], Safe Distance: %.1f m",
                goal_x_, goal_y_, goal_z_, fov_h_, fov_v_, safe_distance_);
}

void LocalPlannerNode::initialize() {
    pointcloud_processor_ = std::make_unique<PointCloudProcessor>(
        histogram_, fov_h_, fov_v_, safe_distance_, yaw_, this->get_logger(), shared_from_this(),
        obstacle_distance_pub_, histogram_image_pub_, laser_scan_pub_, downsampled_cloud_pub_, raw_cloud_pub_);
}

void LocalPlannerNode::pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!position_received_) {
        RCLCPP_WARN(this->get_logger(), "Waiting for position data...");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Received point cloud with %u points",
                msg->width * msg->height);
    pointcloud_processor_->processPointCloud(msg);
    computeVFH();
}

void LocalPlannerNode::positionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
    current_x_ = msg->x;
    current_y_ = msg->y;
    current_z_ = msg->z;
    yaw_ = msg->heading * 180.0 / M_PI;
    yaw_ = normalizeYaw(yaw_);
    position_received_ = true;

    pointcloud_processor_->updateYaw(yaw_);

    double dist_to_goal = std::sqrt(std::pow(goal_x_ - current_x_, 2) + std::pow(goal_y_ - current_y_, 2));
    if (dist_to_goal < GOAL_TOLERANCE) {
        RCLCPP_INFO(this->get_logger(), "Goal reached! Landing...");
        sendLandCommand();
    }
}

void LocalPlannerNode::computeVFH() {
    Eigen::Vector3f current_pos(current_x_, current_y_, current_z_);
    Eigen::Vector3f goal_pos(goal_x_, goal_y_, goal_z_);

    double goal_angle = std::atan2(goal_y_ - current_y_, goal_x_ - current_x_) * 180.0 / M_PI;
    goal_angle = normalizeYaw(goal_angle);
    int goal_bin = static_cast<int>(goal_angle / HISTOGRAM_RESOLUTION);
    int start_bin = static_cast<int>(yaw_ / HISTOGRAM_RESOLUTION);

    const auto& binary_histogram = histogram_->getBinaryHistogram();
    StarPlanner star_planner(binary_histogram, goal_bin);
    std::vector<int> raw_path = star_planner.findPath(start_bin);
    std::vector<int> path = star_planner.smoothPath(raw_path);

    if (path.empty()) {
        RCLCPP_WARN(this->get_logger(), "No path found, falling back to basic VFH");
        double min_cost = std::numeric_limits<double>::max();
        int best_bin = goal_bin;

        for (int i = 0; i < HISTOGRAM_SIZE; ++i) {
            if (binary_histogram[i] == 1) continue;

            double angle_diff = std::abs(i * HISTOGRAM_RESOLUTION - goal_angle);
            if (angle_diff > 180.0) angle_diff = 360.0 - angle_diff;

            double cost = angle_diff;
            if (cost < min_cost) {
                min_cost = cost;
                best_bin = i;
            }
        }
        path = {best_bin};
    }

    Eigen::Vector3f next_waypoint = waypoint_generator_->generateWaypoint(current_pos, goal_pos, path, histogram_->getPolarHistogram());
    if (!prev_waypoint_.isZero()) {
        next_waypoint = waypoint_generator_->smoothWaypoint(current_pos, prev_waypoint_, next_waypoint);
    }
    prev_waypoint_ = next_waypoint;

    float target_x = next_waypoint[0];
    float target_y = next_waypoint[1];
    float target_z = next_waypoint[2];

    double target_angle = std::atan2(target_y - current_y_, target_x - current_x_) * 180.0 / M_PI;
    target_angle = normalizeYaw(target_angle);
    int target_bin = static_cast<int>(target_angle / HISTOGRAM_RESOLUTION);

    double vx = speed_ * (target_x - current_x_) / computeDistanceToPoint(current_pos, next_waypoint, true);
    double vy = speed_ * (target_y - current_y_) / computeDistanceToPoint(current_pos, next_waypoint, true);
    double vz = (target_z - current_z_) * 0.5;

    double vel_magnitude = std::sqrt(vx * vx + vy * vy);
    if (vel_magnitude > speed_) {
        vx *= speed_ / vel_magnitude;
        vy *= speed_ / vel_magnitude;
    }

    double target_yaw = target_bin * HISTOGRAM_RESOLUTION;
    RCLCPP_INFO(this->get_logger(), "Target waypoint: [%.2f, %.2f, %.2f], Target velocity: [vx=%.2f, vy=%.2f, vz=%.2f], target_yaw=%.2f°",
                target_x, target_y, target_z, vx, vy, vz, target_yaw);
    sendVelocityCommand(vx, vy, vz, target_yaw);
}

int LocalPlannerNode::findStartPoint(const std::vector<Eigen::Vector3f>& waypoints, const Eigen::Vector3f& position) {
    float min_dist = std::numeric_limits<float>::max();
    int closest_idx = 0;

    for (size_t i = 0; i < waypoints.size(); ++i) {
        float dist = computeDistanceToPoint(position, waypoints[i], true);
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }
    return closest_idx;
}

float LocalPlannerNode::computeYaw(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return std::atan2(dy, dx) * 180.0 / M_PI;
}

float LocalPlannerNode::computeDistanceToPoint(const Eigen::Vector3f& position, const Eigen::Vector3f& point, bool only_2d) {
    float dx = point[0] - position[0];
    float dy = point[1] - position[1];
    if (only_2d) {
        return std::sqrt(dx * dx + dy * dy);
    }
    float dz = point[2] - position[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

float LocalPlannerNode::normalizeYaw(float yaw) {
    while (yaw >= 360.0) yaw -= 360.0;
    while (yaw < 0.0) yaw += 360.0;
    return yaw;
}

void LocalPlannerNode::setPositionMode() {
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    cmd.param1 = 1;
    cmd.param2 = 3;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 255;
    cmd.source_component = 0;
    cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    command_pub_->publish(cmd);
}

void LocalPlannerNode::sendVelocityCommand(double vx, double vy, double vz, float target_yaw) {
    geometry_msgs::msg::TwistStamped twist{};
    twist.header.stamp = this->get_clock()->now();
    twist.header.frame_id = "base_link";
    twist.twist.linear.x = vx;
    twist.twist.linear.y = vy;
    twist.twist.linear.z = vz;

    double yaw_diff = target_yaw - yaw_;
    if (yaw_diff > 180.0) yaw_diff -= 360.0;
    if (yaw_diff < -180.0) yaw_diff += 360.0;

    double angular_z = yaw_diff * 0.5;
    if (std::abs(angular_z) > MAX_YAW_RATE) {
        angular_z = (angular_z > 0) ? MAX_YAW_RATE : -MAX_YAW_RATE;
    }
    twist.twist.angular.z = angular_z * M_PI / 180.0;

    velocity_pub_->publish(twist);
}

void LocalPlannerNode::sendLandCommand() {
    px4_msgs::msg::VehicleCommand cmd{};
    cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 255;
    cmd.source_component = 0;
    cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    command_pub_->publish(cmd);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalPlannerNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}