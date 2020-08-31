#ifndef __BARRIER_CLASS_HPP__
#define __BARRIER_CLASS_HPP__


class BR_Class{
public:

    int No_X;              // 障碍物序号
    double safedist;       // 障碍物本身安全距离，默认为0
    geometry_msgs::Point   position;      //当前位置

    BR_Class(){};
    BR_Class(int nox, double x,double y,double z ,double dist = 0){    
        No_X       = nox;          // 障碍物编号
        position.x   = x;
        position.y   = y;
        position.z   = z;
        safedist  = dist; //障碍物额外的安全距离，默认为0
    }
};
#endif




