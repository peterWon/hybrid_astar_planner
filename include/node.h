#ifndef _NODE_H_
#define _NODE_H_

#include "pose_2d.h"

class Node{
public:
    Node(const double h, const double c, const double t, 
        Node* const precessor, const Pose2D& pose,const std::vector<Pose2D>& subpath):
        _h_cost(h),_cost_so_far(c),_total_cost(t),_precessor(precessor),
        _pose(pose), _subpath(subpath){}
    ~Node(){}
    //for speed up searching, assign an id for each node
    //id is from the image coodinate along with a dimension for theta, 
    //ie. id / (width*height) = theta_id, id % (width*height) = r*width+c
    void setID(int id){_id = id;}
    int getID() const{ return _id;}
    const double& heuristicCost() const { return _h_cost;}
    const double& costSoFar() const { return _cost_so_far;}
    const double& totalCost() const { return _total_cost;}
    Node* precessor() const {return _precessor;}
    const Pose2D& pose() const {return _pose;}
    const std::vector<Pose2D>& subpath() const {return _subpath;}

    // for dealing with circle path
    void setHCost(double h){_h_cost = h;}
    void setCostSoFar(double c){_cost_so_far = c;}
    void setTotalCost(double t){_total_cost = t;}
    void setPrecessor(Node* p){_precessor = p;}
    void setPose(const Pose2D& p){_pose = p;}
    void setSubpath(const std::vector<Pose2D>& path){_subpath = path;}
private:
    double _h_cost;
    double _cost_so_far;
    double _total_cost;
    Node* _precessor;
    Pose2D _pose;
    int _id;
    std::vector<Pose2D> _subpath;//for processor to itself
};

#endif