#include "star_planner.hpp"

TreeNode::TreeNode(int bin, float cost, TreeNode* parent)
    : bin_(bin), cost_(cost), parent_(parent) {}

int TreeNode::bin() const { return bin_; }
float TreeNode::cost() const { return cost_; }
TreeNode* TreeNode::parent() const { return parent_; }

StarPlanner::StarPlanner(const std::vector<int>& binary_histogram, int goal_bin)
    : binary_histogram_(binary_histogram), goal_bin_(goal_bin) {}

std::vector<int> StarPlanner::findPath(int start_bin) {
    std::priority_queue<std::pair<float, TreeNode*>, std::vector<std::pair<float, TreeNode*>>, std::greater<>> open_set;
    std::unordered_map<int, TreeNode*> all_nodes;
    std::unordered_map<int, float> g_scores;

    TreeNode* start_node = new TreeNode(start_bin, 0.0);
    open_set.emplace(0.0, start_node);
    all_nodes[start_bin] = start_node;
    g_scores[start_bin] = 0.0;

    while (!open_set.empty()) {
        TreeNode* current = open_set.top().second;
        open_set.pop();

        if (current->bin() == goal_bin_) {
            std::vector<int> path = reconstructPath(current);
            cleanup(all_nodes);
            return path;
        }

        for (int neighbor : getNeighbors(current->bin())) {
            if (binary_histogram_[neighbor] == 1) continue;

            float tentative_g_score = g_scores[current->bin()] + 1.0;
            if (all_nodes.find(neighbor) == all_nodes.end() || tentative_g_score < g_scores[neighbor]) {
                float f_score = tentative_g_score + heuristic(neighbor);
                TreeNode* neighbor_node = new TreeNode(neighbor, tentative_g_score, current);
                open_set.emplace(f_score, neighbor_node);
                all_nodes[neighbor] = neighbor_node;
                g_scores[neighbor] = tentative_g_score;
            }
        }
    }

    cleanup(all_nodes);
    return std::vector<int>();
}

std::vector<int> StarPlanner::smoothPath(const std::vector<int>& path) {
    if (path.size() <= 2) return path;

    std::vector<int> smoothed = path;
    for (size_t i = 1; i < smoothed.size() - 1; ++i) {
        smoothed[i] = (smoothed[i - 1] + smoothed[i + 1]) / 2;
    }
    return smoothed;
}

std::vector<int> StarPlanner::getNeighbors(int bin) {
    std::vector<int> neighbors;
    for (int offset = -1; offset <= 1; ++offset) {
        int neighbor = bin + offset;
        if (neighbor < 0) neighbor += HISTOGRAM_SIZE;
        if (neighbor >= HISTOGRAM_SIZE) neighbor -= HISTOGRAM_SIZE;
        neighbors.push_back(neighbor);
    }
    return neighbors;
}

float StarPlanner::heuristic(int bin) {
    float angle_diff = std::abs(bin - goal_bin_);
    if (angle_diff > HISTOGRAM_SIZE / 2) angle_diff = HISTOGRAM_SIZE - angle_diff;
    return angle_diff * HISTOGRAM_RESOLUTION;
}

std::vector<int> StarPlanner::reconstructPath(TreeNode* node) {
    std::vector<int> path;
    while (node != nullptr) {
        path.push_back(node->bin());
        node = node->parent();
    }
    std::reverse(path.begin(), path.end());
    return path;
}

void StarPlanner::cleanup(const std::unordered_map<int, TreeNode*>& nodes) {
    for (const auto& pair : nodes) {
        delete pair.second;
    }
}