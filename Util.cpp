#pragma once

#include "Util.h"

namespace utils {

double distance(Position const &pos_1, Position const &pos_2)
{
    Position pos_diff = pos_1 - pos_2;

    return sqrt(pow(pos_diff.x, 2) + pow(pos_diff.y, 2));
}

std::vector<float> get_bound(Position point, double radius)
{
    std::vector<float> bounds(4);
    bounds[0] = point.x - radius;
    bounds[1] = point.y - radius;
    bounds[2] = point.x + radius;
    bounds[3] = point.y + radius;
    return bounds;
}

}

Tree::~Tree()
{
}

TreeNode *get_new_node(TreeNode *start, TreeNode *target, double step_size, double min_step_size)
{
    Position pos_diff = target->pos - start->pos;
    double dist = utils::distance(start->pos, target->pos);
    if (dist < min_step_size) {
        return nullptr;
    }
    double length = std::min(step_size, dist);
    Position vec_step = (start->pos + pos_diff * (length / dist));
    TreeNode *new_node = new TreeNode(vec_step);
    new_node->parent = start;
    start->child.push_back(new_node);
    return new_node;
}

TreeNode *random_position(Position const &target, float std, std::mt19937 &generator)
{
    Position tmp_pos = {-1, -1};
    while (tmp_pos.x >= 1500 || tmp_pos.x < 0) {
        tmp_pos.x = utils::normal(target.x, std, generator);
    }
    while (tmp_pos.y >= 1000 || tmp_pos.y < 0) {
        tmp_pos.y = utils::normal(target.y, std, generator);
    }
    TreeNode *new_node = new TreeNode(tmp_pos);
    return new_node;
}
