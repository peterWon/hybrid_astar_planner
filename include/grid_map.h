#ifndef _GRID_MAP_H_
#define _GRID_MAP_H_

#include <string>
#include <opencv2/opencv.hpp>
#include "car.h"
#include "pose_2d.h"
#include "node.h"

/*******************************************
 * the map's coodinate is Cartesian coodinate system.
 * we use opencv's mat to store map data, but should translate coodinate from
 * cv's grid coordinate to map's coodinate.
*/
class GridMap{
public:
    GridMap(){}
    ~GridMap(){}
    void load(const std::string& map_info_path);
    int getWidth() const {return _width;}
    int getHeight() const {return _height;}
    double getResolution() const {return _resolution;}
    bool isCollision(const Car& car, const Pose2D& pose);
    uint8_t* getData(){return _map.data;}
    void rc2xy(const int r, const int c, double& x, double& y);
    void xy2rc(const double x, const double y, int &r, int& c);

    // for debugging
    cv::Mat drawResult(const Car& car, const std::vector<Pose2D>& path);
    cv::Mat drawNode(const Car& car, const Node& node);
private:
    int _width;
    int _height;
    double _resolution;
    cv::Mat _map;

    const int _obstacle_thresh = 50;
};

#endif