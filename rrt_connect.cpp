#include "rrt_connect.hpp"
#include <fstream>
#include <math.h>

#if !defined(GETMAPINDEX)
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)
#endif

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654
#define LINKLENGTH_CELLS 10
#define LOGGING 0

int IsValidArmConfiguration(std::vector<double> angles, 
        int numofDOFs, double* map,int x_size, int y_size);

template<typename T>
std::ostream& operator << (std::ostream& os, std::vector<T> list){
    for(int i=0; i<list.size(); i++){
        os<<list[i]<<" ";
    }
    return os;
}

template<typename T>
void log(std::vector<T> pt, std::ofstream& ofs){
    if(LOGGING==0)
        return;
    for(int i=0; i<pt.size(); i++)
        ofs<<pt[i]<<" ";
    ofs<<std::endl;
    return;
}

RRTConnect::RRTConnect(unsigned D, double* map, int x_size, int y_size){
    this->tree_st=new RRT(D, map, x_size, y_size);
    this->tree_gl=new RRT(D, map, x_size, y_size);
    this->tree_st->ext_eps=this->tree_gl->ext_eps=this->ext_eps=0.5;
    this->tree_st->sf=this->tree_gl->sf=this->sf=0.001;
    this->tree_st->episodes=this->tree_gl->episodes=this->episodes=50000;
    this->tree_st->term_th=this->tree_gl->term_th=this->term_th=0.01;
    this->map=map;
    this->x_size=x_size;
    this->y_size=y_size;
    this->is_terminal=false;
    this->exploit_th=0.15;
    this->D=D;
    this->nodes=std::ofstream("nodes.txt");
    this->path=std::ofstream("path.txt");
    this->joints=std::ofstream("joints.txt");
    this->ofs=std::ofstream("RRT_Connect.txt",std::ios::app);
    std::cout<<"Sampling interval is "<<this->sf<<std::endl;
}

int RRTConnect::connect(Node* node, RRT* tree){
    int flag=1;
    while(true){
        flag=tree->extend(node->point);
        if(flag==2||flag==0)
            break;
    }
    if(flag==0)
        return 0;
    else{
        this->terminal_id=node->id;
        return 1;
    }
}

void RRTConnect::backtrack(double*** plan, int* planlength){
    *plan = NULL;
    *planlength = 0;
    std::vector<NodeId> result_st,result_gl;
    NodeId cur=this->terminal_id_st;
    while(cur!=0){
    if(!IsValidArmConfiguration(this->tree_st->node_list[cur]->point,this->D,map,this->x_size,this->y_size)){
            std::cout<<"Traj Position is Invalid\n";
        }
        result_st.push_back(cur);
        cur=this->tree_st->parent_map[cur];
    }
    cur=this->terminal_id_gl;
    while(cur!=0){
        if(!IsValidArmConfiguration(this->tree_gl->node_list[cur]->point,this->D,map,this->x_size,this->y_size)){
            std::cout<<"Traj Position is Invalid\n";
        }
        result_gl.push_back(cur);
        cur=this->tree_gl->parent_map[cur];
    }
    double quality=this->tree_st->get_quality(result_st)+this->tree_gl->get_quality(result_gl);
    int len = result_st.size()+result_gl.size();
    ofs<<"Quality is "<<quality<<std::endl;
    ofs<<"Path length is "<<len<<std::endl;
    int num_samples=(int)result_st.size()+(int)result_gl.size();
    *plan = (double**) malloc(num_samples*sizeof(double*));
    int i=0;
    for(auto it=result_st.rbegin(); it!=result_st.rend(); it++){
        (*plan)[i] = (double*) malloc(this->D*sizeof(double));
        std::cout<<this->tree_st->node_list[*it]->point<<std::endl;
        for(int j=0; j<this->D; j++){
            (*plan)[i][j]=this->tree_st->node_list[*it]->point[j];
        }
        i++;
    }
    for(auto it=result_gl.begin(); it!=result_gl.end(); it++){
        (*plan)[i] = (double*) malloc(this->D*sizeof(double));
        std::cout<<this->tree_gl->node_list[*it]->point<<std::endl;
        for(int j=0; j<this->D; j++){
            (*plan)[i][j]=this->tree_gl->node_list[*it]->point[j];
        }
        i++;
    }
    *planlength=num_samples;
    return;
}

bool RRTConnect::plan(double* start, 
            double* goal, 
            double*** plan, 
            int* planlength){
    Point q_near;
    int signal;
    // Seeds which work: (seed=10,eps_th=0.5,explt_th=0.15)
    unsigned seed=0,exp_seed=0;
    std::default_random_engine eng(seed);
    std::default_random_engine exp_eng(exp_seed);
    std::uniform_real_distribution<double> u_dist(0,2*PI);
    std::uniform_real_distribution<double> explt(0.0,1.0);
    
    this->start=Point(start,start+(int)this->D);
    this->end=Point(goal,goal+(int)this->D);
    if(!IsValidArmConfiguration(this->start,this->D,map,this->x_size,this->y_size)){
        std::cout<<"Initial Position is Invalid\n";
        return 0;
    }
    if(!IsValidArmConfiguration(this->end,this->D,map,this->x_size,this->y_size)){
        std::cout<<"Final Position is Invalid\n";
        return 0;
    }
    std::cout<<this->tree_st->forward_kinematics(this->start)<<" Start Position"<<std::endl;
    std::cout<<this->tree_st->forward_kinematics(this->end)<<" End Position"<<std::endl;
    // Initialize
    this->tree_st->insert(Point(start,start+(int)this->D));
    this->tree_gl->insert(Point(goal,goal+(int)this->D));
    // Set start and goals for tree_st
    this->tree_st->start=Point(start,start+(int)this->D);
    this->tree_st->end=Point(goal,goal+(int)this->D);
    // Set start and goals for tree_gl
    this->tree_gl->start=Point(goal,goal+(int)this->D);
    this->tree_gl->end=Point(start,start+(int)this->D);
    //Initialise pointers
    RRT *tree_a=this->tree_st, *tree_b=this->tree_gl, *temp=NULL;
    // Run for fixed number of episodes
    for(int i=0; i<this->episodes; i++){
        // if(i%2==0)
            // std::cout<<"Extending Tree St\n";
        // else
            // std::cout<<"Extending Tree Gl\n";
        Point q_rand(this->D);
        if(explt(exp_eng)>this->exploit_th){
            for(int j=0; j<this->D; j++)q_rand[j]=u_dist(eng);
        }
        else{ // exploit
            q_rand=tree_a->end;
        }
        signal=tree_a->extend(q_rand);
        if(signal!=0){
            int connect_sig=this->connect(tree_a->node_list[tree_a->counter], tree_b);
            if(connect_sig){
                this->is_terminal=1;
                if(tree_a==this->tree_st){
                    // std::cout<<"Connected Tree St to Tree Gl \n";
                    this->terminal_id_st=tree_a->counter;
                    this->terminal_id_gl=tree_b->counter;
                }
                else{
                    // std::cout<<"Connected Tree Gl to Tree St \n";
                    this->terminal_id_st=tree_b->counter;
                    this->terminal_id_gl=tree_a->counter;
                }
                break;
            }
        }
        if(i%1000==0){
            // std::cout<<i<<" Nodes sampled\n";
        }
        // swap
        temp=tree_a;
        tree_a=tree_b;
        tree_b=temp;
    }
    bool flag=false;
    if(this->is_terminal){
        std::cout<<"Path Found!\n";
        ofs<<this->start<<std::endl;
        ofs<<this->end<<std::endl;
        int total=this->tree_gl->node_list.size()+this->tree_st->node_list.size();
        ofs<<"Nodes in graph "<<total<<std::endl;
        this->backtrack(plan,planlength);
        flag=1;
    }
    else{
        std::cout<<"No Path found\n";
        this->tree_st->terminal_id=this->tree_st->closest_id;
        flag=0;
    }
    log(this->tree_st->forward_kinematics(this->start),nodes);
    log(this->tree_st->forward_kinematics(this->end),nodes);
    nodes.close();
    path.close();
    joints.close();
    return flag;
};
