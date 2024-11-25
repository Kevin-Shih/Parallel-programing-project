#include<cstdio>
#include<cstdlib>
#include<iostream>
#include<vector>

using namespace std;
using namespace cv;

// need 1
float distance(vector<float> &x, vector<float> &y);

// need 1
class TreeNode {
public:
    float pos[2];
    vector<TreeNode *> child;
    TreeNode * parent;
    TreeNode(float pos[2], class TreeNode *parent);
    TreeNode(float pos[2]);
};
// need 1
class Tree {
public:
    TreeNode *root;
    TreeNode *end;
    bool success;
    Tree(TreeNode *root);
    Tree(TreeNode *root, TreeNode *end);
};

// need 1
TreeNode* get_new_node(TreeNode *start, TreeNode *target, float step_size, int min_step_size = 3);

// need 1
TreeNode* random_position(float target[2], float std = 350);

// need 1
vector<int> get_bound(float point[2], float radius);

// need 2
bool point_near_obstacle(bool **map, float point[2], float radius);

// need 2
bool intersection(bool **map, TreeNode *start, TreeNode *end, float radius);

// need 2
TreeNode* nearest(TreeNode *root, TreeNode *target, bool **map, float radius);

// matplotlib visualization
/*
def setup_fig(image, w= 12, h= 9):

def plot(img, tree:Tree, startpos, endpos, path= [], path_name= None):

startpos = []
def click_event(event, x, y, flags, params):

def get_start_point(map):

def get_target_point(targetname):
*/
