#include<cstdio>
#include<cstdlib>
#include<iostream>
#include<time>
#include<vector>
#include<Util.h>
using namespace std;


Tree* RRT(bool **map, float start[2], float target[2], float radius, float step_size, 
          int max_iter, int max_node = 500, float std=350) {
    TreeNode *root = &TreeNode(start);
    TreeNode *end = &TreeNode(target);
    Tree *tree = &Tree(root, end);
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
            TreeNode *rand_node = random_position(end->pos, std);
            if (point_near_obstacle(map, rand_node->pos, radius))
                continue;
            near_node = nearest(root, rand_node, map, radius);
            if (!near_node)
                continue;
            new_node = get_new_node(near_node, rand_node, step_size);
            if (!new_node) // if step too small
                continue;
        }
        n_count ++;
        float dist = distance(new_node->pos, end->pos);
        printf("%4dth node:  pos = [%.1f, %.1f], dist = %4.1f cm           \r",
                n_count, new_node->pos[0], new_node->pos[1], dist);
        if (n_count >= max_node || tree->success)
            break;
    }
    printf("\nFinish RRT construction in %d iters with %d nodes.\n",
            i + 1, n_count);
    return tree;
}

vector<float[2]> path_search(bool **map, float startpos[2], float endpos[2],
                                 float radius= 15, float step_size= 30, int max_iter= 10000,
                                 int max_node= 500, float std= 500, string path_name= ""){

    Tree *tree = RRT(map, startpos, endpos, radius, step_size, max_iter, max_node, std);
    vector<float[2]> path;
    if (tree->success){
        TreeNode *current = tree->end;
        while (current) {
            path.insert(path.begin(), current->pos);
            current = current->parent;
        }
    } else
        printf("failed.");
    // plot(map, tree, startpos, endpos, path, path_name);
    return path;
}

int main(int argc, char** argv){
    /* read img as bool map; */
    Mat img, dst;
    img = imread("map.png", 1);
    cvtColor(img, dst, CV_BGR2GRAY);
    // dst.convertTo(dst, CV_32F);
    bool map[dst.rows][dst.cols];
    for(int i = 0 ;i < dst.rows; ++i)
        for(int j = 0; j < dst.cols; ++j)
                map[i][j]= dst.at<float>(i,j)[0] == 255;

    // if (dst.at<float>(i,j)[0] == 255 &&
    //             dst.at<float>(i,j)[1] == 255 &&
    //             dst.at<float>(i,j)[2] == 255)

    /* set start and destination point*/
    // startpos = get_start_point(img);
    float startpos[2] = {1235, 330};
    float targetpos[2] = {330, 235};
    // targetpos = get_target_point(target);

    /*
    t = np.asarray(["rack", "cushion", "lamp", "stair", "cooktop"])
    # c = np.asarray([(0, 255, 133), (255, 9, 92), (160, 150, 20), (173, 255, 0), (7, 255, 224)])
    t_pos = np.asarray([(880, 290), (1300, 570), (1135, 820), (1200, 110), (385, 675)])
    # idx = np.asarray(np.where(np.all(img_rgb == c[t == target][0], axis= -1)))[::-1, :].T # RC to XY
    # targetpos = np.asarray([np.mean(idx[:, 0]), np.mean(idx[:, 1])], dtype= np.int32)
    targetpos = t_pos[t == target][0]
    */

    printf("startpos: [%f, %f], targetpos: [%f, %f]\n", startpos[0], startpos[1], targetpos[0], targetpos[1]);
    int max_iter = 10000;
    float radius = 10;
    float step_size = 30;

    vector<float[2]> path = path_search(map, startpos, targetpos, radius, step_size, max_iter, 500, 500);
    for (size_t i = 0; i < path.size() - 1; i++)
    {
        printf("[%f, %f] ->", path[i][0], path[i][1]);
    }
    printf("[%f, %f]\n", path[path.size() - 1][0], path[path.size() - 1][1]);
    
    return 0;
}