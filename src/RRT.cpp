#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>

#include "Util.h"

using namespace std;
using namespace chrono;
using namespace rrt_utils;

Tree *RRT(vector<vector<int>> &map, Position start, Position target,
          float radius, float step_size, int max_iter, int max_node, float std,
          std::mt19937 &generator) {
    TreeNode *root = new TreeNode(start);
    TreeNode *end = new TreeNode(target);
    Tree *tree = new Tree(root, end);

    int i, n_count = 0;
    for (i = 0; i < max_iter; i++) {
        TreeNode *near_node = nearest(root, end, map, radius);
        TreeNode *new_node = NULL;
        if (near_node) {
            end->parent = near_node;
            near_node->child.push_back(end);
            tree->success = true;
            new_node = end;
        } else {
            TreeNode *rand_node = random_position(end->pos, std, generator);
            if (point_near_obstacle(map, rand_node->pos,
                                    radius)) { // if not valid rand_node
                i--;
                continue;
            }
            near_node = nearest(root, rand_node, map, radius);
            if (!near_node) { // if no valid near_node
                continue;
            }
            new_node = get_new_node(near_node, rand_node, step_size);
            if (!new_node) { // if step too small
                i--;
                continue;
            }
        }
        n_count++;
        float dist = distance(new_node->pos, end->pos);
        printf("%4dth node:  pos = [%.1f, %.1f], dist = %4.1f cm    \r",
               n_count, new_node->pos.x, new_node->pos.y, dist);
        if (n_count >= max_node || tree->success) break;
    }
    if (tree->success)
        printf("\nFinish RRT construction in %d iters with %d nodes.\n\n",
               i + 1, n_count);
    else
        printf(
            "\nFailed! RRT construction terminated at iter %d with %d "
            "nodes.\n\n",
            i + 1, n_count);
    return tree;
}

vector<Position> path_search(vector<vector<int>> &map, Position startpos,
                             Position endpos, float radius = 15,
                             float step_size = 30, int max_iter = 10000,
                             int max_node = 500, float std = 500,
                             string path_name = "") {
    std::random_device rd;
    std::mt19937 rng(rd());
    Tree *tree = RRT(map, startpos, endpos, radius, step_size, max_iter,
                     max_node, std, rng);
    vector<Position> path;
    if (tree->success) {
        TreeNode *current = tree->end;
        while (current) {
            path.insert(path.begin(), current->pos);
            current = current->parent;
        }
    } else {
        path.insert(path.begin(), tree->end->pos);
    }
    // plot(map, tree, startpos, endpos, path, path_name);
    return path;
}

typedef duration<float> float_seconds;
int main(int argc, char **argv) {
    /* read img as bool map; */
    Mat img;
    img = cv::imread("res/map.png", IMREAD_GRAYSCALE);
    img.convertTo(img, CV_32F);
    vector<vector<int>> map(1000, vector<int>(1500));
    Mat temp_mat(1000, 1500, CV_8U);
    for (int i = 0; i < img.rows; ++i)
        for (int j = 0; j < img.cols; ++j) {
            map[i][j] = (img.at<float>(i, j) >= 255);
            temp_mat.at<uchar>(i, j) = (img.at<float>(i, j) >= 250) * 255;
        }
    imwrite("res/read_map.png", temp_mat);

    /* set start and destination point*/
    // startpos = get_start_point(img);
    Position startpos = Position(1235, 330);
    Position targetpos = Position(385, 675);
    // targetpos = get_target_point(target);

    printf("startpos: [%f, %f], targetpos: [%f, %f]\n", startpos.x, startpos.y,
           targetpos.x, targetpos.y);
    int max_iter = 15000;
    int max_node = 500;
    float std = 500;
    float radius = 15;
    float step_size = 30;

    auto start = high_resolution_clock::now();
    vector<Position> path = path_search(map, startpos, targetpos, radius,
                                        step_size, max_iter, max_node, std);
    auto durations = high_resolution_clock::now() - start;
    for (size_t i = 0; i < path.size() - 1; i++) {
        printf("[%.0f, %.0f] -> ", path[i].x, path[i].y);
    }
    if (path.size() > 0)
        printf("[%.0f, %.0f]\n", path[path.size() - 1].x, path[path.size() - 1].y);

    auto sec = duration_cast<float_seconds>(durations).count();
    printf("\nTime = %.3fs\n", sec);
    return 0;
}
