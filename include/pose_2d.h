#ifndef _POSE_2D_H_
#define _POSE_2D_H_

class Pose2D{
    public:
    Pose2D(double x, double y, double a):_x(x), _y(y), _theta(a){}
    ~Pose2D(){}
    void setX(double x){_x = x;}
    void setY(double y){_y = y;}
    void setTheta(double theta){_theta = theta;}
    double getX() const {return _x;}
    double getY() const {return _y;}
    double getTheta() const {return _theta;}
    const double& x() const {return _x;}
    const double& y() const {return _y;}
    const double& theta() const {return _theta;}
private:
    double _x;
    double _y;
    double _theta;
};

#endif