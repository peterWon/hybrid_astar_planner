#ifndef _CAR_H_
#define _CAR_H_

#include <map>
#include <cmath>
#include <vector>


class Car{
public:
    Car(){}
    ~Car(){}
    void load(const std::string& cfg_file);
    const std::vector<std::pair<double, double>>& footPrint() const {return _foot_print;};
    const double& width()const{return _w;}
    const double& length()const{return _l;}
    const double& wheelbase()const{return _wheelbase;}
    const double minTurningRadius() const{return _wheelbase / std::tan(_max_steer);}
    const double& maxSteer(){return _max_steer;}
    const double& minx()const{return _minx;}
    const double& maxx()const{return _maxx;}
    const double& miny()const{return _miny;}
    const double& maxy()const{return _maxy;}
private:
    std::vector<std::pair<double, double>> _foot_print;
    double _w;
    double _l;
    double _wheelbase;
    double _max_steer;
    double _minx, _miny, _maxx, _maxy;
};

#endif