#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <array>
#include <queue>

#include <cmath>
#include <random>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

struct Position {
    float x;
    float y;

    Position(float _x, float _y)
        : x(_x), y(_y) {}
    Position operator+(const Position &other) const
    {
        return Position(x + other.x, y + other.y);
    }
    Position operator-(const Position &other) const
    {
        return Position(x - other.x, y - other.y);
    }
    template<typename T>
    Position operator*(T mul) const
    {
        return Position(x * mul, y * mul);
    }
    bool operator==(const Position &other) const
    {
        return (x == other.x && y == other.y);
    }
};

namespace rrt_utils {

double distance(Position const &pos_1, Position const &pos_2);

template<typename T>
std::vector<double> linspace(T _start, T _end, int _num)
{
    std::vector<double> linspaced;

    linspaced.reserve(_num);

    double start = static_cast<double>(_start);
    double end = static_cast<double>(_end);
    double num = static_cast<double>(_num);

    if (num <= 0) {
        return linspaced;
    }

    if (num == 1) {
        linspaced.push_back(start);
        return linspaced;
    }

    double delta = (end - start) / (num - 1);

    for(int i = 0; i < num - 1; ++i) {
        linspaced.push_back(start + delta * i);
    }
    linspaced.push_back(end);

    return linspaced;
}

template<typename T>
double normal(T _mean, T _stddev, std::mt19937 &generator)
{
    double mean = static_cast<double>(_mean);
    double stddev = static_cast<double>(_stddev);
    std::normal_distribution<double> distribution(mean, stddev);
    return distribution(generator);
}

vector<float> get_bound(Position point, double radius);

}

struct TreeNode {
    Position pos;
    std::vector<TreeNode*> child;
    TreeNode *parent = nullptr;

    TreeNode(Position _pos)
        : pos(_pos) {}
    TreeNode(Position _pos, TreeNode *_parent)
        : pos(_pos), parent(_parent) {}
};

class Tree {
public:
    Tree(TreeNode *_root)
        : root(_root) {}
    Tree(TreeNode *_root, TreeNode *_end)
        : root(_root), end(_end) {}
    ~Tree();
    TreeNode *root;
    TreeNode *end;
    bool success = false;
};

TreeNode *get_new_node(TreeNode *start, TreeNode *target, double step_size, double min_step_size = 3);
TreeNode *random_position(Position const &target, float std, std::mt19937 &generator);

// need 2
bool point_near_obstacle(const vector<vector<int>> &map, const Position& point, float radius);

// need 2
bool intersection(const vector<vector<int>> &map, const TreeNode *start, const TreeNode *end, float radius);

// need 2
TreeNode* nearest(TreeNode *root, const TreeNode *target, const vector<vector<int>> &map, float radius);

// matplotlib visualization
/*
def setup_fig(image, w= 12, h= 9):

def plot(img, tree:Tree, startpos, endpos, path= [], path_name= None):

startpos = []
def click_event(event, x, y, flags, params):

def get_start_point(map):

def get_target_point(targetname):
*/
