//
//  kdtree.cpp
//  
//
//  Created by A S KARTHIK SAI VISHNU KUMAR on 08/03/20.
//

#include "tree.hpp"

Node::Node(){
    this->id=-1;
}

Node::Node(Point point, NodeId id){
    this->point=point;
    this->id=id;
}
double distance(Point A, Point B){
    double sum=0.0;
    for(int i=0; i<A.size(); i++){
        sum+=pow(A[i]-B[i],2);
    }
    sum = sqrt(sum);
    return sum;
}

template<typename T>
std::ostream& operator << (std::ostream& os, std::vector<T> list){
    for(int i=0; i<list.size(); i++){
        os<<list[i]<<" ";
    }
    return os;
}

Comp::Comp(const Point& point, bool reverse){
    this->key=point;
    this->reverse=reverse;
}

bool Comp::operator() (const Node* A, const Node* B) const{
    if(reverse){//max_heap
        return distance(A->point,this->key) > distance(B->point, this->key);
    }
    //min_heap
    return distance(A->point,this->key) < distance(B->point, this->key);
}

BoundedPQ::BoundedPQ(int k, Point pt){
    this->k=k;
    Comp comp(pt);
    this->bpq = new pqueue(comp);
}

void BoundedPQ::push(Node* node){
    this->bpq->insert(node);
    if(this->bpq->size()>k){
        this->bpq->erase(--this->bpq->end());
    }
}

void BoundedPQ::pop(){
    this->bpq->erase(this->bpq->begin());
}

Node* BoundedPQ::top(){
    return *(this->bpq->begin());
}

Tree::Tree(unsigned D){
        this->D=D;
        this->counter=0;
}

Node* Tree::get_node(NodeId id){
        if(node_list.find(id)==node_list.end()){
            throw "Key not found!";
        }
        return node_list[id];
}

std::vector<Node*> Tree::get_nearest_nodes(const Point &pt, int k){
        if(this->node_list.size()==0){
            std::cout<<"Empty tree!\n";
            return std::vector<Node*>();
        }
        BoundedPQ bp(k,pt);
        for(auto& iter:node_list){
            bp.push(iter.second);
        }
        std::vector<Node*> result(bp.bpq->begin(), bp.bpq->end());
        return result;
}

Tree::~Tree(){
    std::cout<<"Tree Destructor called\n";
    for(auto& iter:this->node_list){
        delete iter.second;
    }
}

double Tree::get_quality(std::vector<NodeId> result){
    int len=result.size();
    double sum=0;
    for(int i=0; i<len-1; i++){
        sum+=distance(this->node_list[result[i]]->point, \
                        this->node_list[result[i+1]]->point);
    }
    return sum;
}