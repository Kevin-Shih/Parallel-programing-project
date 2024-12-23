#include <getopt.h>
#include <omp.h>

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <queue>
#include <random>
#include <vector>
#include <atomic>
#include <algorithm>

using namespace std;
using namespace cv;


// #ifndef __RRT_UTIL__
// #define __RRT_UTIL__
struct Position {
        float x;
        float y;

        Position(float _x, float _y) : x(_x), y(_y) {}
        Position operator+(const Position &other) const {
            return Position(x + other.x, y + other.y);
        }
        Position operator-(const Position &other) const {
            return Position(x - other.x, y - other.y);
        }
        template <typename T>
        Position operator*(T mul) const {
            return Position(x * mul, y * mul);
        }
        bool operator==(const Position &other) const { return (x == other.x && y == other.y); }
};

namespace rrt_utils {

    double distance(Position const &pos_1, Position const &pos_2);

    template <typename T>
    double normal(T _mean, T _stddev, std::mt19937 &generator) {
        double mean = static_cast<double>(_mean);
        double stddev = static_cast<double>(_stddev);
        std::normal_distribution<double> distribution(mean, stddev);
        return distribution(generator);
    }

    vector<float> get_bound(Position point, double radius);

    double find_percentile(vector<float> vec, int ptile);
    double mean(vector<float> vec);
    double std(vector<float> vec, double mean);
} // namespace rrt_utils

struct TreeNode {
        Position pos;
        std::vector<TreeNode *> child;
        TreeNode *parent = nullptr;

        TreeNode(Position _pos) : pos(_pos) {}
        TreeNode(Position _pos, TreeNode *_parent) : pos(_pos), parent(_parent) {}
};

class Tree {
    public:
        Tree(TreeNode *_root) : root(_root) {}
        Tree(TreeNode *_root, TreeNode *_end) : root(_root), end(_end) {}
        ~Tree();
        TreeNode *root;
        TreeNode *end;
        bool success = false;
};

struct result {
        Tree *tree;
        vector<Position> path;
        float time;
};

struct CheckSegArgs {
    const vector<vector<uint8_t>>* map;
    const Position* start;
    const Position* end;
    int start_idx;
    int end_idx;
    int x_bound;
    int y_bound;
    std::atomic<bool>* flag;
};

struct NearestArgs {
    const vector<TreeNode*>* vec;
    const TreeNode* target;
    size_t start_idx;
    size_t end_idx;
    double local_min_dist;
    int local_min_node;
};

struct InflateArgs {
    const Mat* img;
    vector<vector<uint8_t>>* out_map;
    double radius;
    int start_idx;
    int end_idx;
};

TreeNode *get_new_node(const vector<vector<uint8_t>> &map, TreeNode *start, TreeNode *target,
                       double step_size);

TreeNode *random_position(Position const &target, float std, std::mt19937 &generator);

// check interseced with obstacles
bool intersection(const vector<vector<uint8_t>> &map, const Position &start, const Position &end);

// find nearest tree node
TreeNode *nearest(vector<TreeNode*> &vec, const TreeNode *target);

void inflate_map(Mat img, vector<vector<uint8_t>> &out_map, double radius);

void plot(Mat map, Tree *tree, const Position &startpos, const Position &endpos,
          vector<Position> path, string path_name = "");
// #endif