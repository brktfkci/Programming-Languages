#include "waypoint_generator.hpp"

WaypointGenerator::WaypointGenerator(float speed, float safe_distance)
    : speed_(speed), safe_distance_(safe_distance) {}

Eigen::Vector3f WaypointGenerator::generateWaypoint(const Eigen::Vector3f& current_pos,
                                                  const Eigen::Vector3f& goal_pos,
                                                  const std::vector<int>& path,
                                                  const std::vector<float>& histogram) {
    if (path.empty()) return goal_pos;

    int next_bin = path[0];
    float target_angle = next_bin * HISTOGRAM_RESOLUTION * M_PI / 180.0;
    float distance = std::min(static_cast<float>(safe_distance_), static_cast<float>(speed_ * 0.5));

    Eigen::Vector3f waypoint;
    waypoint[0] = current_pos[0] + distance * std::cos(target_angle);
    waypoint[1] = current_pos[1] + distance * std::sin(target_angle);
    waypoint[2] = goal_pos[2];

    return waypoint;
}

Eigen::Vector3f WaypointGenerator::smoothWaypoint(const Eigen::Vector3f& current_pos,
                                                const Eigen::Vector3f& prev_waypoint,
                                                const Eigen::Vector3f& new_waypoint) {
    Eigen::Vector3f smoothed = 0.3 * current_pos + 0.3 * prev_waypoint + 0.4 * new_waypoint;
    return smoothed;
}