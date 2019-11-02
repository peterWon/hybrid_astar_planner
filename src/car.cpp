#include "car.h"
#include "param_reader.h"
#include <cmath>
#include <limits>
#include <glog/logging.h>

void Car::load(const std::string& cfg_file){
    ParamReader reader;
    if(!reader.loadParam(cfg_file)){
        LOG(FATAL)<<"load car parameter failed!";        
        return;
    }
    if(!reader.getValue("wheelbase", _wheelbase)){
        LOG(ERROR)<<"wheelbase not set!";
    }
    if(!reader.getValue("max_steer", _max_steer)){
        LOG(ERROR)<<"max_steer not set";
    }    
    
    std::vector<double> foots;
    if(!reader.getValueVec("foot_print", foots)){
        LOG(ERROR)<<"foots not set!";
    }
    if(foots.size() != 8) {
        LOG(FATAL)<<"foot_print must be 4 points with 8 numbers!";        
        return;
    }
    double maxx, minx, maxy, miny;
    maxx = std::numeric_limits<double>::min();
    minx = std::numeric_limits<double>::max();
    maxy = std::numeric_limits<double>::min();
    miny = std::numeric_limits<double>::max();
    for(int i = 0; i < 4; ++i){
        _foot_print.push_back({foots[2*i], foots[2*i+1]});
        maxx = maxx > foots[2*i] ? maxx : foots[2*i];
        minx = minx < foots[2*i] ? minx : foots[2*i];
        maxy = maxy > foots[2*i+1] ? maxy : foots[2*i+1];
        miny = miny < foots[2*i+1] ? miny : foots[2*i+1];
    }
    _l = maxx - minx;
    _w = maxy - miny;
    _minx = minx;
    _miny = miny;
    _maxx = maxx;
    _maxy = maxy;
    CHECK(_l > 0 && _w > 0)<<"the car foot print is not correctly defined!";
}