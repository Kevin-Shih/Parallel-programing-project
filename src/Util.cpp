#include "Util.h"

namespace rrt_utils {

double distance(Position const &pos_1, Position const &pos_2)
{
    Position pos_diff = pos_1 - pos_2;

    return sqrt(pow(pos_diff.x, 2) + pow(pos_diff.y, 2));
}

vector<float> get_bound(Position point, double radius)
{
    vector<float> bounds(4);
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
    double dist = rrt_utils::distance(start->pos, target->pos);
    if (dist < min_step_size) {
        return nullptr;
    }
    double length = min(step_size, dist);
    Position vec_step = (start->pos + pos_diff * (length / dist));
    TreeNode *new_node = new TreeNode(vec_step);
    new_node->parent = start;
    start->child.push_back(new_node);
    return new_node;
}

TreeNode *random_position(Position const &target, float std, mt19937 &generator)
{
    Position tmp_pos = {-1, -1};
    while (tmp_pos.x >= 1500 || tmp_pos.x < 0) {
        tmp_pos.x = rrt_utils::normal(target.x, std, generator);
    }
    while (tmp_pos.y >= 1000 || tmp_pos.y < 0) {
        tmp_pos.y = rrt_utils::normal(target.y, std, generator);
    }
    TreeNode *new_node = new TreeNode(tmp_pos);
    return new_node;
}

bool point_near_obstacle(const vector<vector<int>>& map, const Position& point, float radius) {
    auto vec = rrt_utils::get_bound(point, radius);

    float low_x = max(0, (int)rint(vec[0]));
    float low_y = max(0, (int)rint(vec[1]));
    float high_x = min((int)map[0].size() - 1, (int)rint(vec[2]));
    float high_y = min((int)map.size() - 1, (int)rint(vec[3]));

    for (int y = low_y; y <= high_y; ++y) {
        for (int x = low_x; x <= high_x; ++x) {
            if (map[y][x] == 0) {
                return true;
            }
        }
    }
    return false;
}

bool intersection(const vector<vector<int>>& map, const Position& start, const Position& end, float radius) {
    double dist = rrt_utils::distance(start, end);
    int num_points = static_cast<int>(dist);
    for (int i = 0; i <= num_points; ++i) {
        Position point = {
            start.x + static_cast<int>((end.x - start.x) * i / num_points),
            start.y + static_cast<int>((end.y - start.y) * i / num_points)
        };

        if (point_near_obstacle(map, point, radius)) {
            return true;
        }
    }
    return false;
}

TreeNode* nearest(TreeNode* root, const TreeNode* target, const vector<vector<int>>& map, float radius) {
    TreeNode* nearest_node = nullptr;
    double min_dist = numeric_limits<double>::infinity();

    queue<TreeNode*> queue;
    queue.push(root);

    while (!queue.empty()) {
        TreeNode* current = queue.front();
        queue.pop();

        if (!intersection(map, current->pos, target->pos, radius)) {
            double dist = rrt_utils::distance(current->pos, target->pos);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_node = current;
            }
        }

        for (TreeNode* child : current->child) {
            queue.push(child);
        }
    }

    return nearest_node;
}
