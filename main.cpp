#include <vector>
#include <string>
#include <chrono>
#include <ctime>
#include <glog/logging.h>

#include "hybrid_astar.h"
#include "param_reader.h"

int main(int argc, char**argv){
    if(argc !=2 ) return -1;
    ParamReader reader;
    if(reader.loadParam(argv[1])){
        std::string map_info_path, car_info_path;
        std::vector<double> start_and_goal;
        
        if(!reader.getValue("map_info_path", map_info_path)){
            LOG(FATAL)<<"map_info_path must be set!";
            return -1;
        }
        if(!reader.getValue("car_info_path", car_info_path)){
            LOG(FATAL)<<"car_info_path must be set!";
            return -1;
        }
        if(!reader.getValueVec("start_and_goal", start_and_goal)){
            LOG(FATAL)<<"start_and_goal must be set!";
            return -1;
        }
        CHECK(start_and_goal.size()==6)<<"start_and_goal must has 6 components!";
        double time_tolerance = 30, step_size = 1.0, turning_penalty_factor = 5.0;
        double goal_tolerance_dist = 0.5, goal_tolerance_theta = 0.1;
        reader.getValue("time_tolerance", time_tolerance); 
        reader.getValue("step_size", step_size); 
        reader.getValue("turning_penalty_factor", turning_penalty_factor); 
        reader.getValue("goal_tolerance_dist", goal_tolerance_dist); 
        reader.getValue("goal_tolerance_theta", goal_tolerance_theta); 
        GridMap* map = new GridMap();
        Car* car = new Car();
        map->load(map_info_path);
        car->load(car_info_path);
        
        HybridAStar* planner = new HybridAStar();
        planner->initialize(car, map, time_tolerance, step_size, 
            turning_penalty_factor, goal_tolerance_dist, goal_tolerance_theta);
        Pose2D start(start_and_goal[0], start_and_goal[1], start_and_goal[2]);
        Pose2D goal(start_and_goal[3], start_and_goal[4], start_and_goal[5]);

        std::vector<Pose2D> solution = {};
        auto ts = std::chrono::system_clock::now();
        if(planner->plan(start, goal, solution)){
            LOG(ERROR)<<"hybrid astar path size: "<<solution.size();
            auto te = std::chrono::system_clock::now();
            std::chrono::duration<double> dt = te-ts;
            LOG(INFO)<<"succeed to find the path! cost time "<<dt.count()<<"s.";
            cv::Mat img = map->drawResult(*car, solution);
            cv::imshow("xxx", img);
            cv::waitKey(0);
        }else{
            auto te = std::chrono::system_clock::now();
            std::chrono::duration<double> dt = te-ts;
            LOG(INFO)<<"failed to find the path! cost time "<<dt.count()<<"s.";
            std::vector<Pose2D> sg = {start, goal};
            cv::Mat wtf = map->drawResult(*car, sg);
            cv::imshow("wtf", wtf);
            cv::waitKey(0);
        }

        delete car;
        delete map;
        delete planner;
    }
    return 0;
}