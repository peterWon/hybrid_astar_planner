#include "grid_map.h"
#include "param_reader.h"
#include "glog/logging.h"

void GridMap::load(const std::string& map_info_path){
    ParamReader reader;
    if(!reader.loadParam(map_info_path)){
        LOG(FATAL)<<"error load map info file!";
    }
    std::string map_file_path;
    if(!reader.getValue("map_file_path", map_file_path)
    || !reader.getValue("resolution", _resolution)){
        LOG(FATAL)<<"map info file param not correct!";
        return;
    }
    _map = cv::imread(map_file_path, CV_LOAD_IMAGE_GRAYSCALE);
    for(int r = 0; r < _map.rows; ++r){
        for(int c = 0; c < _map.cols; ++c){
            _map.at<uchar>(r,c) = 255 - _map.at<uchar>(r,c);
        }
    }
    _width = _map.cols;
    _height = _map.rows;
}

bool GridMap::isCollision(const Car& car, const Pose2D& pose){
    double wx, wy;
    int r,c;
    const double padding = 0;
    const double step = _resolution;
    for(double lx =car.minx()-padding; lx < car.maxx()+padding; lx += step){
        for(double ly = car.miny()-padding; ly < car.maxy()+padding; ly += step){
            wx = lx*std::cos(pose.theta()) - ly*std::sin(pose.theta()) + pose.x();
            wy = lx*std::sin(pose.theta()) + ly*std::cos(pose.theta()) + pose.y();
            xy2rc(wx, wy, r, c);
            if(r < 0 || c < 0 || r>_height-1 || c>_width-1) return true;
            if(_map.at<uchar>(r, c)>_obstacle_thresh) return true;
        }
    }
    return false;
}

void GridMap::rc2xy(const int r, const int c, double& x, double& y){
    x = c * _resolution;
    y = (_height - r - 1)*_resolution;
}  

void GridMap::xy2rc(const double x, const double y, int &r, int& c){
    c = floor(x / _resolution);
    r = _height - floor(y / _resolution) - 1;
}

cv::Mat GridMap::drawResult(const Car& car, const std::vector<Pose2D>& path){
    cv::Mat img = _map.clone();
    if(path.empty()) return img;
    cv::cvtColor(img, img, CV_GRAY2BGR);
    std::vector<cv::Point2f> foot_prints;
    double wx,wy;
    int r, c;
    Pose2D former = path.at(0);
    for(int i = 0; i < path.size(); i+=1){
        /* xy2rc(path.at(i).x(), path.at(i).y(), r, c);
        img.at<cv::Vec3b>(r,c)[0] = 0;
        img.at<cv::Vec3b>(r,c)[1] = 0;
        img.at<cv::Vec3b>(r,c)[2] = 255; */
        const auto& pose = path.at(i);
        if(i>0 && i<path.size()-1){
            double dx = pose.x() - former.x();
            double dy = pose.y() - former.y();
            if(std::sqrt(dx*dx + dy*dy)<0.3){
                continue;
            }
        }
        
        for(const auto& foot: car.footPrint()){
            wx = pose.x() + foot.first * std::cos(pose.theta()) - foot.second*std::sin(pose.theta());
            wy = pose.y() + foot.first * std::sin(pose.theta()) + foot.second*std::cos(pose.theta());
            xy2rc(wx, wy, r, c);
            if(r < 0 || c < 0 || r>_height-1 || c>_width-1) continue;
            foot_prints.push_back(cv::Point2f(c, r));
        }
        cv::Scalar color = cv::Scalar(0,255,0);
        if(foot_prints.size()==4){
            cv::line(img, foot_prints[0], foot_prints[1], color);
            cv::line(img, foot_prints[1], foot_prints[2], color);
            cv::line(img, foot_prints[2], foot_prints[3], color);
            cv::line(img, foot_prints[3], foot_prints[0], color);
            cv::circle(img, (foot_prints[0]+foot_prints[1]) / 2, 1, cv::Scalar(0,0,255));
        }        
        foot_prints.clear();
        former = pose;
    }

    return img;
}

cv::Mat GridMap::drawNode(const Car& car, const Node& node){
    cv::Mat img = _map.clone();
    cv::cvtColor(img, img, CV_GRAY2BGR);
    std::vector<cv::Point2f> foot_prints_node, foot_prints_node_1;
    double wx,wy;
    int r, c;

    for(const auto& foot: car.footPrint()){
        wx = node.pose().x() + foot.first * std::cos(node.pose().theta()) - foot.second*std::sin(node.pose().theta());
        wy = node.pose().y() + foot.first * std::sin(node.pose().theta()) + foot.second*std::cos(node.pose().theta());
        xy2rc(wx, wy, r, c);
        if(r < 0 || c < 0 || r>_height-1 || c>_width-1) continue;
        foot_prints_node.push_back(cv::Point2f(c, r));

        wx = node.precessor()->pose().x() + foot.first * std::cos(node.precessor()->pose().theta()) - foot.second*std::sin(node.precessor()->pose().theta());
        wy = node.precessor()->pose().y() + foot.first * std::sin(node.precessor()->pose().theta()) + foot.second*std::cos(node.precessor()->pose().theta());
        xy2rc(wx, wy, r, c);
        if(r < 0 || c < 0 || r>_height-1 || c>_width-1) continue;
        foot_prints_node_1.push_back(cv::Point2f(c, r));
    }
    if(foot_prints_node_1.size()==4 && foot_prints_node.size()==4){
        cv::line(img, foot_prints_node[0], foot_prints_node[1], cv::Scalar(0,255,0));
        cv::line(img, foot_prints_node[1], foot_prints_node[2], cv::Scalar(0,255,0));
        cv::line(img, foot_prints_node[2], foot_prints_node[3], cv::Scalar(0,255,0));
        cv::line(img, foot_prints_node[3], foot_prints_node[0], cv::Scalar(0,255,0));
        cv::circle(img, (foot_prints_node[0]+foot_prints_node[1]) / 2, 1, cv::Scalar(0,0,255));

        cv::line(img, foot_prints_node_1[0], foot_prints_node_1[1], cv::Scalar(0,0,255));
        cv::line(img, foot_prints_node_1[1], foot_prints_node_1[2], cv::Scalar(0,0,255));
        cv::line(img, foot_prints_node_1[2], foot_prints_node_1[3], cv::Scalar(0,0,255));
        cv::line(img, foot_prints_node_1[3], foot_prints_node_1[0], cv::Scalar(0,0,255));
        cv::circle(img, (foot_prints_node_1[0]+foot_prints_node_1[1]) / 2, 1, cv::Scalar(0,0,255));
    }
    return img;
}