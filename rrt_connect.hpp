#ifndef rrt_connect_hpp
#define rrt_connect_hpp
#include <random>
#include <fstream>
#include "tree.hpp"
#include "rrt.hpp"


class RRTConnect{
    public:
    RRT *tree_st=NULL,*tree_gl=NULL;
    double ext_eps,sf,*map,term_th,exploit_th;
    int x_size,y_size,episodes;
    NodeId terminal_id,closest_id,terminal_id_gl,terminal_id_st;
    Point start,end;
    bool is_terminal;
    unsigned D;
    std::ofstream nodes,path,joints,ofs;
    // debug variables
    double min_dist, min_ee_dist;
    RRTConnect(unsigned D, double* map, int x_size, int y_size);
    bool plan(double* start, double* goal, double*** plan, int* planlength);
    void backtrack(double*** plan, int* planlength);
    int connect(Node* node, RRT* tree);
};
#endif
