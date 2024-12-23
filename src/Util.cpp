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

TreeNode* random_position(Position const& target, float std, mt19937& generator) {
    Position tmp_pos = {-1, -1};
    while (tmp_pos.x >= 1500 || tmp_pos.x < 0) {
        tmp_pos.x = rrt_utils::normal(target.x, std, generator);
    }
    while (tmp_pos.y >= 1000 || tmp_pos.y < 0) {
        tmp_pos.y = rrt_utils::normal(target.y, std, generator);
    }
    TreeNode* new_node = new TreeNode(tmp_pos);
    return new_node;
}


// Thread function
void* check_segment(void* arg) {
    CheckSegArgs* args = static_cast<CheckSegArgs*>(arg);
    const auto& map = *args->map;
    const Position& start = *args->start;
    const Position& end = *args->end;
    int num_points = args->end_idx - args->start_idx + 1;

    for (int i = args->start_idx; i <= args->end_idx; ++i) {
        int x = start.x + static_cast<int>((end.x - start.x) * i / num_points);
        int y = start.y + static_cast<int>((end.y - start.y) * i / num_points);
        if (y > 0 && x > 0 && y < args->y_bound && x < args->x_bound && !map[y][x]) {
            args->flag->store(false, std::memory_order_relaxed);
            pthread_exit(nullptr); // Exit early if obstacle is found
        }
    }

    pthread_exit(nullptr);
}

bool intersection(const vector<vector<uint8_t>>& map, const Position& start, const Position& end) {
    int num_points = static_cast<int>(rrt_utils::distance(start, end));
    const int num_threads = 4;
    int points_per_thread = num_points / num_threads;
    int x_bound = map[0].size(), y_bound = map.size();

    std::atomic<bool> flag(true); // Shared flag to indicate no obstacles
    pthread_t threads[num_threads];
    CheckSegArgs args[num_threads];

    // Create threads
    for (int t = 0; t < num_threads; ++t) {
        args[t] = {
            &map,  &start, &end, t * points_per_thread, 
            (t == num_threads - 1) ? num_points : (t + 1) * points_per_thread - 1, 
            x_bound, y_bound, &flag
        };

        pthread_create(&threads[t], nullptr, check_segment, &args[t]);
    }

    // Join threads
    for (int t = 0; t < num_threads; ++t) {
        pthread_join(threads[t], nullptr);
    }

    return !flag.load(std::memory_order_relaxed); // Return true if any obstacle is found
}


void* nearest_thread(void* arg) {
    NearestArgs* args = static_cast<NearestArgs*>(arg);
    const auto& vec = *args->vec;
    const TreeNode* target = args->target;

    for (size_t i = args->start_idx; i <= args->end_idx; ++i) {
        double dist = rrt_utils::distance(vec[i]->pos, target->pos);
        if (dist < args->local_min_dist) {
            args->local_min_dist = dist;
            args->local_min_node = i;
        }
    }

    pthread_exit(nullptr);
}

TreeNode* nearest(vector<TreeNode*>& vec, const TreeNode* target) {
    if (vec.empty()) {
        std::cerr << "vec is empty, cannot find nearest" << std::endl;
        exit(1);
    }

    const int num_threads = std::min((int)vec.size(), 4);
    size_t chunk_size = vec.size() / num_threads;

    pthread_t threads[num_threads];
    NearestArgs args[num_threads];

    for (int t = 0; t < num_threads; ++t) {
        args[t] = {
            &vec, target, t * chunk_size,
            (t == num_threads - 1) ? vec.size() - 1 : (t + 1) * chunk_size - 1,
            std::numeric_limits<double>::max(),
            -1
        };

        if (pthread_create(&threads[t], nullptr, nearest_thread, &args[t]) != 0) {
            std::cerr << "error creating threads in nearest" << std::endl;
            exit(1);
        }
    }

    double global_min_dist = std::numeric_limits<double>::max();
    int global_min_node = -1;

    for (int t = 0; t < num_threads; ++t) {
        if (pthread_join(threads[t], nullptr) != 0) {
            std::cerr << "error joining threads in nearest" << std::endl;
            exit(1);
        }
        if (args[t].local_min_node != -1 && args[t].local_min_dist < global_min_dist) {
            global_min_dist = args[t].local_min_dist;
            global_min_node = args[t].local_min_node;
        }
    }

    return vec[global_min_node];
}

void* inflate_thread(void* arg) {
    InflateArgs* args = static_cast<InflateArgs*>(arg);
    const Mat& img = *args->img;
    auto& out_map = *args->out_map;
    double radius = args->radius;

    for (int index = args->start_idx; index <= args->end_idx; index++) {
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

    pthread_exit(nullptr);
}

void inflate_map(Mat img, vector<vector<uint8_t>>& out_map, double radius) {
    const int num_threads = 4; // Adjust thread count as needed
    int total_pixels = img.rows * img.cols;
    int chunk_size = total_pixels / num_threads;

    pthread_t threads[num_threads];
    InflateArgs args[num_threads];

    for (int t = 0; t < num_threads; ++t) {
        args[t] = {
            &img,
            &out_map,
            radius,
            t * chunk_size,
            (t == num_threads - 1) ? total_pixels - 1 : (t + 1) * chunk_size - 1
        };
        pthread_create(&threads[t], nullptr, inflate_thread, &args[t]);
    }

    for (int t = 0; t < num_threads; ++t) {
        pthread_join(threads[t], nullptr);
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
