#include "barrier_avoid/barrier_function.hpp"

int main(int argc,char **argv)
{
    ros::init(argc, argv, "barrier_fun");
    BarFun_Class bfclass(10);      //hz 
    //设置参数
    bfclass.setFlyMaxSpeed(0.05);  //最大速度
    bfclass.setFlySafeDist(1.5);   //安全距离
    bfclass.setFlyBfGamma(1);      //该值越大，避障范围越小，贴的越近
    bfclass.setFlyXi(0.2);         //该值越大，避障范围越大，贴的越远

    bfclass.run();   
    return 0;
}
