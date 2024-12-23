#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <vector>

#include "Util.h"

using namespace std;
using namespace chrono;
using namespace rrt_utils;

typedef duration<float> float_secs;

string _map_names[] = {"res/map.png", "res/maze1.png", "res/maze2.png", "res/maze3.png", "res/maze2_mid.png", "res/maze2_big.png"};
Position _startposs[] = {Position(1235, 330), Position(10, 445), Position(20, 405),
                         Position(1265, 65), Position(20, 405)*2.5, Position(20, 405)*4};
Position _targetposs[] = {Position(390, 665), Position(585, 975), Position(215, 975),
                          Position(180, 945), Position(215, 975)*2.5, Position(215, 975)*4};
struct arguments {
        int testruns = 1;
        int max_iter = 250000;
        int max_node = 100000;
        float std = 1000;
        float radius = 15;
        float step_size = 50;
        string map_name = "res/map.png";
        Position startpos = Position(1235, 330);
        Position targetpos = Position(390, 665);
        int plot = 0;
        int verbose = 0;
        int flag = 0;
};

void usage(const char *progname) {
    printf("Usage: %s [options]\n", progname);
    printf("Program Options:\n");
    printf("  -i  --iter    <INT>   Test iterations(>1)\n");
    printf("  -m  --map     <INT>   Input map (0, 1, 2, 3)\n");
    printf("  -r  --radius  <FLOAT> Radius to inflate the obstacles\n");
    printf("  -l  --steplen <FLOAT> Step length for getting new nodes(>15)\n");
    printf("  -s  --std     <FLOAT> Std for generate rand node\n");
    printf("  -p  --plot            Whether to plot the result and save\n");
    printf("  -v  --verbose <INT>   Whether to print info\n");
    printf("  -h  --help            This message\n");
}

arguments process_opt(int argc, char *argv[]) {
    const char *optstring = "i:m:r:l:s:v::ph";
    int opt;
    static struct option long_options[] = {{"map", 1, NULL, 'm'},     {"radius", 1, NULL, 'r'},
                                           {"steplen", 1, NULL, 'l'}, {"std", 1, NULL, 's'},
                                           {"plot", 0, NULL, 'p'},    {"verbose", 2, NULL, 'v'},
                                           {"help", 0, NULL, '?'}};
    arguments args;
    while ((opt = getopt_long(argc, argv, optstring, long_options, NULL)) != -1) {
        switch (opt) {
            case 'i': {
                args.testruns = atoi(optarg);
                break;
            }
            case 'm': {
                int i = atoi(optarg);
                if (i < 5) {
                    args.map_name = _map_names[i];
                    args.startpos = _startposs[i];
                    args.targetpos = _targetposs[i];
                }
                break;
            }
            case 'r': {
                args.radius = atof(optarg);
                break;
            }
            case 'l': {
                args.step_size = atof(optarg);
                break;
            }
            case 's': {
                args.std = atof(optarg);
                break;
            }
            case 'p': {
                args.plot = 1;
                break;
            }
            case 'v': {
                if (optarg) {
                    args.verbose = atoi(optarg);
                } else {
                    args.verbose = 1;
                }
                break;
            }
            case 'h':
            default:
                usage(argv[0]);
                args.flag = -1;
                return args;
        }
    }
    if (args.testruns > 1) {
        args.verbose = 0;
        args.plot = 0;
    }
    return args;
}

Tree *RRT(arguments args, vector<vector<uint8_t>> &map, Position start, Position target,
          float step_size, int max_iter, int max_node, float std, std::mt19937 &generator) {
    TreeNode *root = new TreeNode(start);
    TreeNode *end = new TreeNode(target);
    Tree *tree = new Tree(root, end);
    vector<TreeNode*> vec;
    vec.push_back(root);
    int i, n_count = 0;
    for (i = 0; i < max_iter; i++) {
        TreeNode *near_node = nearest(vec, end);
        TreeNode *new_node = NULL;
        float dist = distance(near_node->pos, end->pos);
        if (dist < 1.5 * step_size && !intersection(map, near_node->pos, target)) {
            end->parent = near_node;
            near_node->child.push_back(end);
            tree->success = true;
            new_node = end;
        } else {
            for (int attempt = 0; attempt < max_iter; ++attempt) {
                mt19937 thread_generator(generator());
                TreeNode *rand_node = random_position(end->pos, std, thread_generator);
                if (map[rand_node->pos.y][rand_node->pos.x]) {
                    near_node = nearest(vec, rand_node);
                    uniform_real_distribution<double> distribution(max(15.0f, step_size/5), step_size);
                    double rng_step_size = distribution(thread_generator);
                    new_node = get_new_node(map, near_node, rand_node, rng_step_size);
                    if (new_node) {
                        vec.push_back(new_node);
                        break;
                    }
                }
            }
        }
        n_count++;
        if (args.verbose > 1) {
            dist = distance(new_node->pos, end->pos);
            printf("%4dth node:  pos = [%.1f, %.1f], dist = %4.1f cm    \r", n_count,
                   new_node->pos.x, new_node->pos.y, dist);
        }
        if (n_count >= max_node || tree->success) {
            break;
        }
    }
    if (args.verbose > 1) printf("\n");
    if (tree->success) {
        if (args.verbose > 0) {
            printf("Finish RRT construction in with %d nodes.\n", n_count);
        }
    } else {
        printf(
            "Failed! RRT construction terminated with %d "
            "nodes.\n",
            n_count);
    }
    return tree;
}

result path_search(arguments args, vector<vector<uint8_t>> &map, Position startpos, Position endpos,
                   float step_size = 30, int max_iter = 10000, int max_node = 500,
                   float std = 500) {
    std::random_device rd;
    static thread_local std::mt19937 rng(rd());
    auto start = system_clock::now();
    Tree *tree = RRT(args, map, startpos, endpos, step_size, max_iter, max_node, std, rng);
    auto end = system_clock::now();
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
    return result{tree, path, duration_cast<float_secs>(end - start).count()};
}

int main(int argc, char **argv) {
    arguments args = process_opt(argc, argv);
    if (args.flag) return 1;
    if (args.verbose > 1) {
        printf("startpos: [%.0f, %.0f], targetpos: [%.0f, %.0f]\n", args.startpos.x,
               args.startpos.y, args.targetpos.x, args.targetpos.y);
    }

    /* read img as bool map; */
    Mat img;
    img = imread(args.map_name, IMREAD_GRAYSCALE);
    vector<vector<uint8_t>> map(img.rows, vector<uint8_t>(img.cols, 1));
    vector<float> times;
    
    auto start = system_clock::now();
    inflate_map(img, map, args.radius);
    auto mid = system_clock::now();

    if (args.plot) { // plot how the map is read (with obstacles inflated)
        Mat temp_mat(img.rows, img.cols, CV_8U);
        for (int i = 0; i < img.rows; ++i) {
            for (int j = 0; j < img.cols; ++j) {
                temp_mat.at<uint8_t>(i, j) = map[i][j] ? 255 : 0;
            }
        }
        Point start_point = Point(args.startpos.x, args.startpos.y);
        Point target_point = Point(args.targetpos.x, args.targetpos.y);
        circle(temp_mat, start_point, 6, Scalar(0, 0, 0), 10, LINE_AA);
        circle(temp_mat, target_point, 6, Scalar(0, 0, 0), 10, LINE_AA);
        imwrite("res/read_map.png", temp_mat);
    }

    for (int runs = 0; runs < args.testruns; runs++) {
        auto [tree, path, time] =
            path_search(args, map, args.startpos, args.targetpos, args.step_size, args.max_iter,
                        args.max_node, args.std);
        float total_time = duration_cast<float_secs>(mid - start).count() + time;
        times.push_back(total_time);

        if (args.testruns == 1) printf("Time = %.3fs\n", total_time);
        if (args.verbose > 1) {
            printf("\nStart position\n");
            for (size_t i = 0; i < path.size() - 1; i++) {
                printf("[%4.0f, %4.0f] -> ", path[i].x, path[i].y);
                if (i % 4 == 0) cout << endl;
            }
            if (path.size() % 4 != 2) cout << endl;
            if (path.size() > 0)
                printf("[%4.0f, %4.0f]\nTarget position\n", path[path.size() - 1].x,
                       path[path.size() - 1].y);
        }
        if (args.plot) {
            img = imread(args.map_name, IMREAD_COLOR_BGR);
            plot(img, tree, args.startpos, args.targetpos, path, "result_omp");
        }
    }

    if (args.testruns > 1) {
        
        vector<float> original_times(times);
        vector<float> erased_time;
        
        int flag = 1;
        while (flag) {
            flag = 0;
            float mean = rrt_utils::mean(times);
            float std = rrt_utils::std(times, mean);
            for (size_t i = 0; i < times.size(); i++) {
                // 90% confident, 1.64sigma
                if (times[i] > (mean + 1.64 * std) || times[i] < (mean - 1.64 * std)) {
                    erased_time.push_back(times[i]);
                    times.erase(times.begin()+i);
                    flag = 1;
                }
            }
        }

        int count = 0;
        for (float time : original_times) {
            if(find(erased_time.begin(), erased_time.end(), time) != erased_time.end()){
                printf("Run%3d = %.3f (Outlier)\n", ++count, time);
            }else{
                printf("Run%3d = %.3f\n", ++count, time);
            }
        }

        sort(times.begin(), times.end());
        float mean = rrt_utils::mean(times);
        float std = rrt_utils::std(times, mean);
        float p25 = find_percentile(times, 25);
        float median = find_percentile(times, 50);
        float p75 = find_percentile(times, 75);
        printf("All Time in seconds, Total valid test runs = %ld.\n", times.size());
        
        printf("Avg. = %.3f, Std. = %.3f, P25 = %.3f, Median = %.3f, P75 = %.3f\n", mean, std, p25,
               median, p75);
    }
    return 0;
}
