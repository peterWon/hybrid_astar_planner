#ifndef _HYBRID_ASTAR_H_
#define _HYBRID_ASTAR_H_

#include <map>
#include "car.h"
#include "node.h"
#include "pose_2d.h"
#include "grid_map.h"
#include "astar.h"

enum SUBPATH_TYPE{
        LEFT_FORWORD=0,
        STRAIGHT_FORWORD,
        RIGHT_FORWORD,
        LEFT_BACKWARD,
        STRAIGHT_BACKWORD,
        RIGHT_BACKWARD
};
class HybridAStar{
public:
    HybridAStar();
    ~HybridAStar();
    bool plan(const Pose2D& start, const Pose2D& goal, std::vector<Pose2D>& path);
    void initialize(Car* car, GridMap* map, 
                    double time_tolerance=20.0,
                    double step_size=1.0,
                    double turning_penalty_factor = 5.0,
                    double goal_tolerance_dist = 0.5,
                    double goal_tolerance_theta = 0.1);
private:
    void setNodeID(Node* node);
    int computeNodeID(const Pose2D& pose);
    double length(const std::vector<Pose2D>& path);
    bool reedsSheppShot(const Pose2D& start, const Pose2D& goal, 
                        std::vector<Pose2D>& rs_path, double& length);
    bool isReachedGoal(const Pose2D& pos, const Pose2D& goal);
    bool checkPath(std::vector<Pose2D>& path);
    
    double computeHCost(const Pose2D& start, const Pose2D& goal);
    void tracePath(Node* const e, Node* const s, std::vector<Pose2D>* path);
    double normalizeTheta(double theta);
    std::map<SUBPATH_TYPE, std::vector<Pose2D>> neighbors(const Pose2D& pose);
    // return a subpath of a specific type
    std::vector<Pose2D> subpath(const SUBPATH_TYPE& type,
                                const Pose2D& start,
                                const double step,/*straight forward length or turing theta*/
                                const double res,
                                const double turning_radius);
    Car* _car;
    GridMap* _map;
    AStar* _astar;
    std::map<int, Node*> _closeset, _openset;

    double _time_tolerance = 20.0; //seconds
    double _step_size = 1.0;  // m
    
    double _heuristic_factor = 1.0;

    double _goal_tolerance_dist = 0.5;
    double _goal_tolerance_theta = 0.1;

    double _turning_penalty_factor = 1.5;
    double _backward_penalty_factor = 1.5;
};

#endif