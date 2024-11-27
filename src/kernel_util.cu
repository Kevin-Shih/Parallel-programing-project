#include "kernel_util.h"
#include <cuda.h>
#include <cuda_runtime.h>

namespace kernel_utils {

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
    double dist = kernel_utils::distance(start->pos, target->pos);
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

TreeNode * random_position(Position const &target, float std, std::mt19937 &generator)
{

    Position tmp_pos = {-1, -1};
    while (tmp_pos.x >= 1500 || tmp_pos.x < 0) {
        tmp_pos.x = kernel_utils::normal(target.x, std, generator);
    }
    while (tmp_pos.y >= 1000 || tmp_pos.y < 0) {
        tmp_pos.y = kernel_utils::normal(target.y, std, generator);
    }
    TreeNode *new_node = new TreeNode(tmp_pos);
    return new_node;
}

bool node_near_obstacle(int **map, const TreeNode *node, float radius) {
    return point_near_obstacle(map, node->pos, radius);
}

__global__ void near_obstacle(int *obs_flag, int **map, int low_x, int low_y) {
    const int thisX = low_x + threadIdx.x + blockDim.x * blockIdx.x;
    const int thisY = low_y + threadIdx.y + blockDim.y * blockIdx.y;
    __shared__ uint8_t flag;
    flag = 0;
    if (map[thisY][thisX] == 0)
        flag = 1;
    __syncthreads();
    if(thisX == low_x && thisY == low_y){
        *obs_flag = flag;
    }
}

bool point_near_obstacle(int **map, const Position& point, float radius) {
    auto vec = kernel_utils::get_bound(point, radius);

    float low_x = max(0, (int)rint(vec[0]));
    float low_y = max(0, (int)rint(vec[1]));
    float high_x = min((int)1500 - 1, (int)rint(vec[2]));
    float high_y = min((int)1000 - 1, (int)rint(vec[3]));
    int flag = 0;
    dim3 threadsPerBlock(16, 16);
    dim3 numsBlock((high_x-low_x+threadsPerBlock.x-1) / threadsPerBlock.x, (high_y-low_y+threadsPerBlock.y-1) / threadsPerBlock.y);
    int **d_map, *d_flag;
    cudaHostRegister(map, 1500*1000*sizeof(int), cudaHostRegisterMapped);
    cudaHostGetDevicePointer(&d_map, map, 0);
    cudaHostRegister(&flag, sizeof(int), cudaHostRegisterMapped);
    cudaHostGetDevicePointer(&d_flag, &flag, 0);
    near_obstacle<<<numsBlock, threadsPerBlock>>>(d_flag, d_map, low_x, low_y);

    return flag;
}

bool intersection(int **map, const Position& start, const Position& end, float radius) {
    double dist = kernel_utils::distance(start, end);
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

TreeNode* nearest(TreeNode* root, const TreeNode* target, int **map, float radius) {
    TreeNode* nearest_node = nullptr;
    double min_dist = std::numeric_limits<double>::infinity();

    std::queue<TreeNode*> queue;
    queue.push(root);

    while (!queue.empty()) {
        TreeNode* current = queue.front();
        queue.pop();

        if (!intersection(map, current->pos, target->pos, radius)) {
            double dist = kernel_utils::distance(current->pos, target->pos);
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

// void grow_tree(TreeNode **node_list, int **map, Tree *tree, 
//             float radius, float step_size, float std, std::mt19937 &generator){
//     int thisX = blockIdx.x * blockDim.x + threadIdx.x;
//     TreeNode *root = tree->root, *end = tree->end;
//     TreeNode *near_node = NULL, *new_node = NULL;
//     TreeNode *rand_node = random_position(end->pos, std, generator);
//     if (!node_near_obstacle(map, rand_node, radius)) { 
//         // if is valid rand_node
//         near_node = nearest(root, rand_node, map, radius);
//         if (near_node) { 
//             // if has valid near_node
//             new_node = get_new_node(near_node, rand_node, step_size);
//         }
//     }
//     node_list[thisX] = new_node;
// }

// void check_valid(TreeNode *near_node, const std::vector<std::vector<int>>& map, Tree *tree, 
//             float radius, float step_size, float std, std::mt19937 &generator){
//     int thisX = blockIdx.x * blockDim.x + threadIdx.x;
//     if (!node_near_obstacle(map, rand_node, radius)) { 
//         // if is valid rand_node
//         near_node = nearest(root, rand_node, map, radius);
        
//     }
// }