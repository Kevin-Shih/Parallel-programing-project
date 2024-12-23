#include "Util.h"

namespace rrt_utils {

    double distance(Position const& pos_1, Position const& pos_2) {
        Position pos_diff = pos_1 - pos_2;

        return sqrt(pow(pos_diff.x, 2) + pow(pos_diff.y, 2));
    }

    vector<float> get_bound(Position point, double radius) {
        vector<float> bounds(4);
        bounds[0] = point.x - radius;
        bounds[1] = point.y - radius;
        bounds[2] = point.x + radius;
        bounds[3] = point.y + radius;
        return bounds;
    }

    double find_percentile(vector<float> vec, int ptile) {
        float idx_ptile = ptile / 100.0 * vec.size();
        int low = floor(idx_ptile);
        int high = ceil(idx_ptile);
        return vec[low] + (vec[high] - vec[low]) * (idx_ptile - low);
    }

    double mean(vector<float> vec) {
        double sum = 0;
        for (double val : vec) {
            sum += val;
        }
        return sum / vec.size();
    }

    double std(vector<float> vec, double mean) {
        double sum = 0.0;
        double temp = 0.0;

        for (double val : vec) {
            temp = val - mean;
            sum += temp * temp;
        }

        return sqrt(sum / (vec.size() - 1));
    }
} // namespace rrt_utils

Tree::~Tree() {}
TreeNode* get_new_node(const vector<vector<uint8_t>>& map, TreeNode* start, TreeNode* target,
                       double step_size) {
    Position pos_diff = target->pos - start->pos;
    double dist = rrt_utils::distance(start->pos, target->pos);
    if (dist < step_size) return nullptr;
    Position vec_step = (start->pos + pos_diff * (step_size / dist));
    if (!intersection(map, start->pos, vec_step)) {
        TreeNode* new_node = new TreeNode(vec_step);
        new_node->parent = start;
        start->child.push_back(new_node);
        return new_node;
    }
    return nullptr;
}
int _w, _h;
TreeNode* random_position(Position const& target, float std, mt19937& generator) {
    Position tmp_pos = {-1, -1};
    while (tmp_pos.x >= _w || tmp_pos.x < 0) {
        tmp_pos.x = rrt_utils::normal(target.x, std, generator);
    }
    while (tmp_pos.y >= _h || tmp_pos.y < 0) {
        tmp_pos.y = rrt_utils::normal(target.y, std, generator);
    }
    TreeNode* new_node = new TreeNode(tmp_pos);
    return new_node;
}

bool intersection(const vector<vector<uint8_t>>& map, const Position& start, const Position& end) {
    int num_points = static_cast<int>(rrt_utils::distance(start, end));
    int flag = true; // whether all not obstacles

#pragma omp parallel for reduction(& : flag) schedule(dynamic, 64) num_threads(8)
    for (int i = 0; i <= num_points; ++i) {
        int x = start.x + static_cast<int>((end.x - start.x) * i / num_points);
        int y = start.y + static_cast<int>((end.y - start.y) * i / num_points);
        flag &= map[y][x];
    }
    return !flag;
}

TreeNode* nearest(vector<TreeNode*>& vec, const TreeNode* target) {
    // TreeNode* nearest_node = nullptr;
    double min_dist = std::numeric_limits<double>::max();
    int min_node = 0;
// queue<TreeNode*> queue;
// queue.push(root);
// while (!queue.empty()) {
//     TreeNode* current = queue.front();
//     queue.pop();
//     vec.push_back(current);
//     for (TreeNode* child : current->child) {
//         queue.push(child);
//     }
// }
#pragma omp parallel num_threads(8)
    {
        // near_node local_nearest = {std::numeric_limits<double>::max(), NULL};
        // double local_min_dist = std::numeric_limits<double>::max();
        // int local_min_node = -1;
#pragma omp for nowait schedule(dynamic, 64)
        for (size_t i = 0; i < vec.size(); i++) {
            double dist = rrt_utils::distance(vec[i]->pos, target->pos);
            if (dist < min_dist) {
                min_dist = dist;
                min_node = i;
            }
        }
        // if (local_min_dist < min_dist) {
        //     min_dist = local_min_dist;
        //     min_node = local_min_node;
        // }
    }
    return vec[min_node];
}

void inflate_map(Mat img, vector<vector<uint8_t>>& out_map, double radius) {
    _h = img.rows;
    _w = img.cols;
#pragma omp parallel for schedule(dynamic, 64) num_threads(8)
    for (int index = 0; index < img.rows * img.cols; index++) {
        int y = index / img.cols;
        int x = index % img.cols;
        if (img.at<uint8_t>(y, x) < 250) {
            int low_x = max(0, (int)ceil(x - radius));
            int low_y = max(0, (int)ceil(y - radius));
            int high_x = min(img.cols - 1, (int)ceil(x + radius));
            int high_y = min(img.rows - 1, (int)ceil(y + radius));
            for (int y = low_y; y <= high_y; ++y) {
                for (int x = low_x; x <= high_x; ++x) {
                    out_map[y][x] = 0;
                }
            }
        }
    }
}

void plot(Mat temp_mat, Tree* tree, const Position& startpos, const Position& targetpos,
          vector<Position> path, string path_name) {
    Point point_1, point_2;
    Scalar red(0, 0, 255);
    Scalar purple(173, 13, 106);
    Scalar black(0, 0, 0);
    int thickness = 2;

    queue<TreeNode*> queue;
    queue.push(tree->root);
    while (!queue.empty()) {
        TreeNode* current = queue.front();
        queue.pop();
        if (current->parent) {
            Position p_1 = current->parent->pos;
            Position p_2 = current->pos;
            point_1 = Point(static_cast<int>(p_1.x), static_cast<int>(p_1.y));
            point_2 = Point(static_cast<int>(p_2.x), static_cast<int>(p_2.y));
            drawMarker(temp_mat, point_2, purple, MARKER_DIAMOND, 4, thickness, LINE_8);
            line(temp_mat, point_1, point_2, black, 1, LINE_8);
        }
        for (TreeNode* child : current->child) {
            queue.push(child);
        }
    }

    point_1 = Point(startpos.x, startpos.y);
    circle(temp_mat, point_1, 6, Scalar(255, 0, 0), 10, LINE_AA);

    for (size_t i = 0; i < path.size() - 1; i++) {
        point_1 = Point(static_cast<int>(path[i].x), static_cast<int>(path[i].y));
        point_2 = Point(static_cast<int>(path[i + 1].x), static_cast<int>(path[i + 1].y));
        drawMarker(temp_mat, point_2, purple, MARKER_DIAMOND, 6, 3, LINE_8);
        line(temp_mat, point_1, point_2, red, thickness, LINE_8);
    }

    point_2 = Point(targetpos.x, targetpos.y);
    circle(temp_mat, point_2, 6, Scalar(0, 255, 0), 10, LINE_AA);
    imwrite("res/" + path_name + ".png", temp_mat);
}
