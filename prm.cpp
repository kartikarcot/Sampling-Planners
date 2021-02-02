#include "prm.hpp"
#include <fstream>
#include <math.h>
#include <queue>

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

namespace prm{
    class Comp{
        public:
        bool operator() (Node* A, Node* B) const{
            //min_heap
            return static_cast<PRMNode*>(A)->cost > static_cast<PRMNode*>(B)->cost;
        }
    };
}

PRMNode::PRMNode(Point point, NodeId id):Node(point,id){
    this->cost=1e5;
}

std::vector<double> PRM::forward_kinematics(const Point& angles){
    double x0,y0,x1,y1;
        int i;
        //iterate through all the links starting with the base
        x1 = ((double)this->x_size)/2.0;
        y1 = 0;
        for(i = 0; i < this->D; i++)
        {
            //compute the corresponding line segment
            x0 = x1;
            y0 = y1;
            x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
            y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);
        }
    return {x1,y1};
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

int IsValidArmConfiguration(std::vector<double> angles, 
        int numofDOFs, double* map,int x_size, int y_size);

template<typename T>
std::ostream& operator << (std::ostream& os, std::vector<T> list){
    for(int i=0; i<list.size(); i++){
        os<<list[i]<<" ";
    }
    return os;
}

static Point operator + (Point A, Point B){
    Point res(A.size());
    for(int i=0; i<A.size(); i++)
        res[i]=A[i]+B[i];
    return res;
}

static Point operator - (Point A, Point B){
    Point res(A.size());
    for(int i=0; i<A.size(); i++)
        res[i]=A[i]-B[i];
    return res;
}

static Point operator * (double k, Point A){
    Point res(A.size());
    for(int i=0; i<A.size(); i++)
        res[i]=k*A[i];
    return res;
}

PRM::PRM(unsigned D, double* map, int x_size, int y_size):Tree(D){
    this->sf=0.1;
    this->map=map;
    this->x_size=x_size;
    this->y_size=y_size;
    this->episodes=60000;
    this->is_terminal=false;
    //debug variable initialisations
    this->min_dist=1000000;
    this->min_ee_dist=1000000;
    this->nodes=std::ofstream("nodes.txt");
    this->path=std::ofstream("path.txt");
    this->joints=std::ofstream("joints.txt");
    this->ofs=std::ofstream("PRM.txt",std::ios::app);
    this->K=10;
}

bool PRM::present(const Point &pt){
    for(auto& item:this->node_list){
        if(item.second->point==pt)
            return true;
    }
    return false;
}


NodeId PRM::insert(const Point &pt, NodeId adj_pt_id){
    if(this->present(pt)){
        // std::cout<<"Already present\n";
        return 0;
    }
    ++this->counter;
    this->node_list[this->counter]=new PRMNode(pt,this->counter);
    return this->counter;
}

void PRM::connect( NodeId adj_pt_id, NodeId cur_id){
    PRMNode* cur_cast=static_cast<PRMNode*>(this->node_list[cur_id]);
    PRMNode* adj_cast=static_cast<PRMNode*>(this->node_list[adj_pt_id]);
    cur_cast->adj_list.push_back(node_list[adj_pt_id]);
    adj_cast->adj_list.push_back(node_list[cur_id]);
}

bool PRM::new_config(const Point& q_near, const Point& q){
    double dist=distance(q_near, q);
    Point unit_vec=(1/dist)*(q-q_near),q_sampled;
    int no_samples = dist/(this->sf);
    double step_dist = this->sf;
    int break_flg=0,i=0;
    for(i=0; i<=no_samples; i++){
        q_sampled = q_near+step_dist*i*unit_vec;
        if(!IsValidArmConfiguration(q_sampled, this->D, this->map, 
                this->x_size, this->y_size)){
                break_flg=1;
                break;
        }
    }

    //advanced or reached
    if(!break_flg && i!=0){
        if(!IsValidArmConfiguration(q, this->D, this->map,
        this->x_size, this->y_size)){
            std::cout<<"Invalid config advanced fully "<<i<<" "<<no_samples<<std::endl;
        }
        return true;
    }
    else{
        return false;
    }
}

std::vector<Node*> PRM::neighborhood(const Point &pt, double th, int K){
    std::vector<Node*> result;
    for(auto& iter:node_list){
        if(distance(iter.second->point,pt)<=th)
            result.push_back(iter.second);
    }
    Comp comp(pt,false);
    std::sort(result.begin(), result.end(), comp);
    int k=MIN(result.size(),K);
    return std::vector<Node*>(result.begin(), result.begin()+k);
}

int PRM::extend(const Point &q){
    auto result=this->neighborhood(q, 1.0, this->K);
    int flag=0;
    auto cur_id=this->insert(q,0);
    if(cur_id!=0){
        for(auto& item: result){
            auto signal = this->new_config(item->point, q);
            if(signal){
                flag=1;
                this->connect(item->id, cur_id);
            }
        }
    }
    return flag;
}

bool PRM::dijkstra(int start, int end, std::vector<NodeId>& result){
    std::priority_queue<Node*,std::vector<Node*>, prm::Comp> pq;
    static_cast<PRMNode*>(this->node_list[start])->cost=0;
    this->parent_map[start]=0;
    pq.push(this->node_list[start]);
    std::unordered_map<NodeId,bool> visited;
    std::cout<<"Visited end value "<<visited[end]<<std::endl;
    std::cout<<"End "<<end<<" "<<this->node_list[end]->point<<std::endl;
    std::cout<<"Start "<<start<<" "<<this->node_list[start]->point<<std::endl;
    while(!pq.empty()&&!visited[end]){
        auto cur=pq.top();
        auto cur_cast=static_cast<PRMNode*>(cur);
        pq.pop();
        if(visited[cur->id]){
            // std::cout<<"Visited\n";
            continue;
        }
        // std::cout<<"Visited "<<cur->id<<std::endl;
        visited[cur->id]=true;
        // std::cout<<"Adjacency list\n";
        for(auto &item:cur_cast->adj_list){
            // std::cout<<item->id<<std::endl;
            PRMNode* item_cast = static_cast<PRMNode*>(item);
            if(!visited[item->id]){
                double dist=distance(cur->point,item->point);
                if(item_cast->cost>cur_cast->cost+dist){
                    item_cast->cost=cur_cast->cost+dist;
                    this->parent_map[item_cast->id]=cur_cast->id;
                }
                pq.push(item);
            }
        }
    }
    if(visited[end]){
        std::cout<<"Path found\n";
        int cur=end;
        while(cur!=0){
            // std::cout<<"Path "<<cur<<std::endl;
            result.push_back(cur);
            cur=this->parent_map[cur];
        }
        std::cout<<"start "<<result[result.size()-1]<<" "<<result[0]<<std::endl;
        return true;
    }
    return false;
}

bool PRM::backtrack(double*** plan, int* planlength, int start_id, int end_id){
    *plan = NULL;
    *planlength = 0;
    std::vector<NodeId> result;
    NodeId cur=start_id;
    bool flag=this->dijkstra(start_id, end_id, result);
    if(flag){
        ofs<<"Quality is "<<get_quality(result)<<std::endl;
        ofs<<"Path length is "<<result.size()<<std::endl;
    }
    int num_samples=(int)result.size();
    *plan = (double**) malloc(num_samples*sizeof(double*));
    int i=0;
    for(auto it=result.rbegin(); it!=result.rend(); it++){
        std::cout<<node_list[*it]->point<<"\n";
        (*plan)[i] = (double*) malloc(this->D*sizeof(double));
        
        for(int j=0; j<this->D; j++){
            (*plan)[i][j]=this->node_list[*it]->point[j];
        }
        i++;
    }
    *planlength=num_samples;
    return flag;
}

bool PRM::plan(double* start, 
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
    std::cout<<this->forward_kinematics(this->start)<<" Start Position"<<std::endl;
    std::cout<<this->forward_kinematics(this->end)<<" End Position"<<std::endl;
    Point q_rand(this->D);
    for(int i=0; i<this->episodes; i++){
        for(int j=0; j<this->D; j++)
            q_rand[j]=u_dist(eng);
        if(IsValidArmConfiguration(q_rand, this->D, this->map, 
                this->x_size, this->y_size)){
            this->extend(q_rand);
        }
        if(i%1000==0){
            // std::cout<<i<<" Nodes sampled\n";
        }
    }
    std::cout<<"Nodes in graph are "<<this->node_list.size()<<std::endl;
    int flag_1=this->extend(this->start);
    int start_id=this->counter;
    int flag_2=this->extend(this->end);
    int end_id=this->counter;
    // find path
    bool flag=false;
    std::cout<<flag_1<<" "<<flag_2<<std::endl;
    if(flag_1&&flag_2){
        flag=this->backtrack(plan, planlength, start_id, end_id);
        if(flag){
            ofs<<this->start<<std::endl;
            ofs<<this->end<<std::endl;
            ofs<<"Nodes in graph "<<this->node_list.size()<<std::endl;
        }
        
    }
    nodes.close();
    path.close();
    joints.close();
    return flag;
};
