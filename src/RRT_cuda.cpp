#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <vector>

#include "kernel_util.h"

// using namespace std;
// using namespace c;
// using namespace kernel_utils;

Tree *RRT(int **map, Position start, Position target,
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
            TreeNode *near_node = NULL;
            if (!point_near_obstacle(map, rand_node->pos, radius)) { 
                // if is valid rand_node
                near_node = nearest(root, rand_node, map, radius);
                if (near_node) { 
                    // if has valid near_node
                    new_node = get_new_node(near_node, rand_node, step_size);
                }
            }
            // TreeNode **newnodes = (TreeNode **)malloc(size * sizeof(TreeNode*)), **d_newnodes;
            // dim3 threadsPerBlock(128);
            // dim3 numsBlock(size / threadsPerBlock.x);
            // cudaHostRegister(newnodes, size * sizeof(TreeNode*), cudaHostRegisterMapped);
            // cudaHostGetDevicePointer(&d_newnodes, newnodes, 0);
            // grow_tree<<<numsBlock, threadsPerBlock>>>(d_newnodes, map, tree, 
            //      radius, step_size, std, generator);
            // cudaHostUnregister(newnodes);
        }
        if(new_node != NULL){
            n_count++;
            float dist = kernel_utils::distance(new_node->pos, end->pos);
            printf("%4dth node:  pos = [%.1f, %.1f], dist = %4.1f cm    \r",
                n_count, new_node->pos.x, new_node->pos.y, dist);
            if (n_count >= max_node || tree->success) break;
        }
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

std::vector<Position> path_search(int **map, Position startpos,
                             Position endpos, float radius = 15,
                             float step_size = 30, int max_iter = 10000,
                             int max_node = 500, float std = 500,
                             std::string path_name = "") {
    std::random_device rd;
    std::mt19937 rng(rd());
    
    Tree *tree = RRT(map, startpos, endpos, radius, step_size, max_iter,
                     max_node, std, rng);
    std::vector<Position> path;
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

int main(int argc, char **argv) {
    /* read img as bool map; */
    cv::Mat img;
    img = cv::imread("res/map.png", cv::IMREAD_GRAYSCALE);
    img.convertTo(img, CV_32F);
    int *map[1000];
    cv::Mat temp_mat(1000, 1500, CV_8U);
    for (int i = 0; i < img.rows; ++i){
        map[i] = new int[1500];
        for (int j = 0; j < img.cols; ++j) {
            map[i][j] = (img.at<float>(i, j) >= 255);
            temp_mat.at<uchar>(i, j) = (img.at<float>(i, j) >= 250) * 255;
        }
    }
    cv::imwrite("res/read_map.png", temp_mat);

    /* set start and destination point*/
    // startpos = get_start_point(img);
    Position startpos = Position(1235, 330);
    Position targetpos = Position(385, 675);
    // targetpos = get_target_point(target);

    printf("startpos: [%.0f, %.0f], targetpos: [%f, %f]\n", startpos.x,
           startpos.y, targetpos.x, targetpos.y);
    int max_iter = 15000;
    int max_node = 1500;
    float std = 500;
    float radius = 15;
    float step_size = 30;


    auto start = std::chrono::system_clock::now();
    std::vector<Position> path = path_search(map, startpos, targetpos, radius,
                                        step_size, max_iter, max_node, std);
    auto end = std::chrono::system_clock::now();
    for (size_t i = 0; i < path.size() - 1; i++) {
        printf("[%4.0f, %4.0f] -> ", path[i].x, path[i].y);
        if (i % 4 == 0) printf("\n");
    }
    if (path.size() > 0)
        printf("[%4.0f, %4.0f]\n", path[path.size() - 1].x,
               path[path.size() - 1].y);

    float sec = std::chrono::duration_cast<std::chrono::duration<float>>(end - start).count();
    printf("\nTime = %.3fs\n", sec);
    return 0;
}