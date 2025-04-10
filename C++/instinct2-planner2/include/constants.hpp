#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/obstacle_distance.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>
#include <eigen3/Eigen/Dense>
#include <queue>
#include <unordered_map>

// Параметры по умолчанию
const double HISTOGRAM_RESOLUTION = 5.0;
const int HISTOGRAM_SIZE = 360 / HISTOGRAM_RESOLUTION;
const double DEFAULT_FOV_HORIZONTAL = 70.0;
const double DEFAULT_FOV_VERTICAL = 55.0;
const double DEFAULT_ROBOT_RADIUS = 0.5;
const double DEFAULT_SAFE_DISTANCE = 5.0;
const double DEFAULT_HEIGHT = 2.0;
const double DEFAULT_SPEED = 1.0;
const double GOAL_TOLERANCE = 0.5;
const uint16_t MAX_OBSTACLE_DISTANCE = 65535;
const double MAX_YAW_RATE = 30.0;

// Параметры камеры
const double FOCAL_LENGTH_MM = 2.6;
const double PIXEL_SIZE_MM = 0.00112;
const double BASELINE_MM = 60.0;
const double DEPTH_CONSTANT = (FOCAL_LENGTH_MM * BASELINE_MM) / PIXEL_SIZE_MM;