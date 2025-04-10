#pragma once
#include "constants.hpp"

class WaypointGenerator {
public:
    WaypointGenerator(float speed, float safe_distance);
    Eigen::Vector3f generateWaypoint(const Eigen::Vector3f& current_pos, 
                                   const Eigen::Vector3f& goal_pos,
                                   const std::vector<int>& path, 
                                   const std::vector<float>& histogram);
    Eigen::Vector3f smoothWaypoint(const Eigen::Vector3f& current_pos, 
                                 const Eigen::Vector3f& prev_waypoint,
                                 const Eigen::Vector3f& new_waypoint);

private:
    float speed_;
    float safe_distance_;
};