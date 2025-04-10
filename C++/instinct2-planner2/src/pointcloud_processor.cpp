#include "pointcloud_processor.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

PointCloudProcessor::PointCloudProcessor(std::shared_ptr<Histogram> histogram,
                                        double fov_h, double fov_v,
                                        double safe_distance, double yaw,
                                        rclcpp::Logger logger,
                                        rclcpp::Node::SharedPtr node,
                                        rclcpp::Publisher<px4_msgs::msg::ObstacleDistance>::SharedPtr obstacle_pub,
                                        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr histogram_image_pub,
                                        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub,
                                        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsampled_cloud_pub,
                                        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_cloud_pub)  // Новый издатель
    : histogram_(histogram), fov_h_(fov_h), fov_v_(fov_v), safe_distance_(safe_distance), yaw_(yaw),
      logger_(logger), node_(node), obstacle_distance_pub_(obstacle_pub), histogram_image_pub_(histogram_image_pub),
      laser_scan_pub_(laser_scan_pub), downsampled_cloud_pub_(downsampled_cloud_pub), raw_cloud_pub_(raw_cloud_pub) {  // Инициализация
    distances_.resize(HISTOGRAM_SIZE, MAX_OBSTACLE_DISTANCE);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    publish_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(50),  // 20 Гц
        std::bind(&PointCloudProcessor::publishTimerCallback, this));
}

void PointCloudProcessor::processPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto start_time = std::chrono::steady_clock::now();

    // Конвертация в PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *pcl_cloud);

    // Предварительный даунсэмплинг
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(pcl_cloud);
    voxel_grid.setLeafSize(0.3f, 0.3f, 0.3f);  // 10 см
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_grid.filter(*downsampled_cloud);

    // Трансформация
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    pcl::toROSMsg(*downsampled_cloud, transformed_cloud);
    transformed_cloud.header = msg->header;
    try {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
            "base_link", msg->header.frame_id, tf2::TimePointZero);
        tf2::doTransform(transformed_cloud, transformed_cloud, transform);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(logger_, "Transform failed: %s", ex.what());
    }

    publishRawCloud(transformed_cloud);

    // Инициализация гистограммы и расстояний
    distances_.assign(HISTOGRAM_SIZE, MAX_OBSTACLE_DISTANCE);
    histogram_->reset();

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filtered_pcl_cloud->header = pcl_conversions::toPCL(transformed_cloud.header);
    filtered_pcl_cloud->is_dense = true;
    filtered_pcl_cloud->reserve(transformed_cloud.width * transformed_cloud.height / 2);

    int valid_points = 0, nan_points = 0, fov_filtered = 0;
    float min_distance = std::numeric_limits<float>::max(), max_distance = 0.0;
    double avg_distance = 0.0;
    int avg_count = 0;

    sensor_msgs::PointCloud2Iterator<float> iter_x(transformed_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(transformed_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(transformed_cloud, "z");

    for (size_t i = 0; i < transformed_cloud.width * transformed_cloud.height; ++i, ++iter_x, ++iter_y, ++iter_z) {
        float x = *iter_x, y = *iter_y, z = *iter_z;
        if (std::isnan(x) || std::isnan(y) || std::isnan(z) || x <= 0.0) {
            nan_points++;
            continue;
        }

        float distance = std::sqrt(x * x + y * y + z * z);
        if (distance > safe_distance_) continue;
        if (!isInsideFOV(x, y, z)) {
            fov_filtered++;
            continue;
        }

        filtered_pcl_cloud->points.emplace_back(x, y, z);

        if (distance < min_distance) min_distance = distance;
        if (distance > max_distance) max_distance = distance;
        avg_distance += distance;
        avg_count++;

        double angle = std::atan2(y, x) * 180.0 / M_PI;
        angle = normalizeYaw(angle);
        int bin = static_cast<int>(angle / HISTOGRAM_RESOLUTION);
        if (bin >= HISTOGRAM_SIZE) bin = HISTOGRAM_SIZE - 1;

        distances_[bin] = std::min(distances_[bin], static_cast<uint16_t>(std::round(distance * 100.0)));
        histogram_->addPoint(x, y, z, safe_distance_);

        valid_points++;
    }

    histogram_->computeBinaryHistogram(0.1);
    histogram_->smoothHistogram();

    sensor_msgs::msg::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*filtered_pcl_cloud, filtered_cloud_msg);
    filtered_cloud_msg.header = transformed_cloud.header;
    publishDownsampledCloud(filtered_cloud_msg);

    auto end_time = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_INFO(logger_, "Processed %d points (valid=%d, NaN=%d, FOV_filtered=%d) in %ld ms",
                transformed_cloud.width * transformed_cloud.height, valid_points, nan_points, fov_filtered, duration.count());
}

void PointCloudProcessor::publishTimerCallback() {
    publishObstacleDistance();
    publishHistogramImage();
    publishLaserScan();
}

void PointCloudProcessor::publishDownsampledCloud(const sensor_msgs::msg::PointCloud2& cloud) {
    downsampled_cloud_pub_->publish(cloud);
    RCLCPP_INFO(logger_, "Published downsampled cloud with %u points", static_cast<unsigned int>(cloud.width * cloud.height));
}

void PointCloudProcessor::publishRawCloud(const sensor_msgs::msg::PointCloud2& cloud) {  // Новый метод
    raw_cloud_pub_->publish(cloud);
    RCLCPP_INFO(logger_, "Published raw cloud with %u points", static_cast<unsigned int>(cloud.width * cloud.height));
}

const std::vector<uint16_t>& PointCloudProcessor::getDistances() const {
    return distances_;
}

void PointCloudProcessor::updateYaw(double new_yaw) {
    yaw_ = normalizeYaw(new_yaw);
}

bool PointCloudProcessor::isInsideFOV(float x, float y, float z) {
    float horizontal_angle = std::atan2(y, x) * 180.0 / M_PI;
    float vertical_angle = std::atan2(z, std::sqrt(x * x + y * y)) * 180.0 / M_PI;
    return (std::abs(horizontal_angle) <= fov_h_ / 2.0 && std::abs(vertical_angle) <= fov_v_ / 2.0);
}

float PointCloudProcessor::normalizeYaw(float yaw) {
    while (yaw >= 360.0) yaw -= 360.0;
    while (yaw < 0.0) yaw += 360.0;
    return yaw;
}

void PointCloudProcessor::publishObstacleDistance() {
    px4_msgs::msg::ObstacleDistance obstacle_msg{};
    obstacle_msg.timestamp = rclcpp::Clock().now().nanoseconds() / 1000;
    obstacle_msg.sensor_type = 0;
    obstacle_msg.increment = HISTOGRAM_RESOLUTION;  // 5°
    obstacle_msg.angle_offset = 0.0;
    obstacle_msg.frame = 12;  // MAV_FRAME_BODY_FRD

    std::vector<uint16_t> distances_resized(72, MAX_OBSTACLE_DISTANCE);
    int sectors_per_bin = HISTOGRAM_SIZE / 72;
    for (int i = 0; i < 72; ++i) {
        int start_bin = i * sectors_per_bin;
        uint16_t min_distance = MAX_OBSTACLE_DISTANCE;
        for (int j = 0; j < sectors_per_bin && (start_bin + j) < HISTOGRAM_SIZE; ++j) {
            min_distance = std::min(min_distance, distances_[start_bin + j]);
        }
        distances_resized[i] = min_distance;
    }

    std::copy(distances_resized.begin(), distances_resized.end(), obstacle_msg.distances.begin());
    obstacle_msg.min_distance = *std::min_element(distances_resized.begin(), distances_resized.end());
    obstacle_msg.max_distance = safe_distance_ * 100.0;

    obstacle_distance_pub_->publish(obstacle_msg);
}

void PointCloudProcessor::publishHistogramImage() {
    sensor_msgs::msg::Image image_msg{};
    image_msg.header.stamp = rclcpp::Clock().now();
    image_msg.header.frame_id = "base_link";
    image_msg.height = 100;
    image_msg.width = HISTOGRAM_SIZE;
    image_msg.encoding = "mono8";
    image_msg.step = image_msg.width;
    image_msg.data.resize(image_msg.height * image_msg.step, 0);

    const auto& polar_histogram = histogram_->getPolarHistogram();
    double max_weight = *std::max_element(polar_histogram.begin(), polar_histogram.end());
    if (max_weight == 0.0) max_weight = 1.0;

    for (int x = 0; x < HISTOGRAM_SIZE; ++x) {
        int height = static_cast<int>((polar_histogram[x] / max_weight) * (image_msg.height - 1));
        for (int y = 0; y <= height; ++y) {
            image_msg.data[(image_msg.height - 1 - y) * image_msg.step + x] = 255;
        }
    }

    histogram_image_pub_->publish(image_msg);
}

void PointCloudProcessor::publishLaserScan() {
    sensor_msgs::msg::LaserScan scan_msg{};
    scan_msg.header.stamp = rclcpp::Clock().now();
    scan_msg.header.frame_id = "base_link";
    scan_msg.angle_min = -fov_h_ * M_PI / 360.0;
    scan_msg.angle_max = fov_h_ * M_PI / 360.0;
    scan_msg.angle_increment = HISTOGRAM_RESOLUTION * M_PI / 180.0;
    scan_msg.time_increment = 0.0;
    scan_msg.range_min = 0.0;
    scan_msg.range_max = safe_distance_;
    scan_msg.ranges.resize(HISTOGRAM_SIZE);
    scan_msg.intensities.resize(HISTOGRAM_SIZE);

    for (int i = 0; i < HISTOGRAM_SIZE; ++i) {
        double angle = i * HISTOGRAM_RESOLUTION;
        if (angle < -fov_h_ / 2.0 || angle > fov_h_ / 2.0) {
            scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
            scan_msg.intensities[i] = 0.0;
        } else {
            scan_msg.ranges[i] = distances_[i] / 100.0;
            scan_msg.intensities[i] = histogram_->getPolarHistogram()[i];
        }
    }

    laser_scan_pub_->publish(scan_msg);
}