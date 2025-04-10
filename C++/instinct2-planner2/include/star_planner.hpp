#pragma once
#include "constants.hpp"

class TreeNode {
public:
    TreeNode(int bin, float cost, TreeNode* parent = nullptr);
    int bin() const;
    float cost() const;
    TreeNode* parent() const;

private:
    int bin_;
    float cost_;
    TreeNode* parent_;
};

class StarPlanner {
public:
    StarPlanner(const std::vector<int>& binary_histogram, int goal_bin);
    std::vector<int> findPath(int start_bin);
    std::vector<int> smoothPath(const std::vector<int>& path);

private:
    const std::vector<int>& binary_histogram_;
    int goal_bin_;

    std::vector<int> getNeighbors(int bin);
    float heuristic(int bin);
    std::vector<int> reconstructPath(TreeNode* node);
    void cleanup(const std::unordered_map<int, TreeNode*>& nodes);
};