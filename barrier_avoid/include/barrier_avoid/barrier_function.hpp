#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"
#include "QuadProg++.hpp"
#include "fly_class.hpp"
#include "barrier_class.hpp"
#include <vector>
#include <math.h>
#include "Array.hpp"
#include <vector>
#include <fstream>
#include "Eigen/Dense"
#include <nav_msgs/Path.h>
#include "barrier_avoid/my_point.h"//自定义的服务数据类型

using namespace quadprogpp;
//using namespace dji_osdk_ros;

static MatrixXd allBarP;    // 障碍物位置
class BarFun_Class
{
public:
    BarFun_Class(int arg_hz):hz(arg_hz),bar_cnt(0){ 
        fly.myInit(0, nh);
        fly.setFlyPose(0,0,0);
        fly_aim.x=0.0;
        fly_aim.y=0.0;
        fly_aim.z=1.0;
    }

    
    
    
     void run() {
         ros::Rate loop_rate(hz);
	 
	 flight_control_client = nh.serviceClient<dji_osdk_ros::FlightTaskControl>("flight_task_control");
	 
	 // 订阅目标位置
         aim_service=nh.advertiseService("/moniter_aimpose",&BarFun_Class::flyAimCallback,this);
         // 订阅障碍物位置
         barrier_service=nh.advertiseService("/barpose",&BarFun_Class::barPoseCallback,this);
         

        if(takeoff()) {
             ROS_INFO("Takeoff command sent successfully");
        } 
        
        
          
          while(ros::ok()){
             fly.avoidBarrier(fly_aim.x, fly_aim.y, fly_aim.z, allBarPositon, br, true);
             fly.pubPath();                                
             ros::spinOnce();                              
             loop_rate.sleep();                            
          }

     if (land()){
            ROS_INFO("Land command sent successfully");
          }
         
    }

    //初始的目标在哪，还没想清楚，先写个接口
    void setFlyAim(double x,double y,double z) {
        fly_aim.x=x;
        fly_aim.y=y;
        fly_aim.z=z;
    }
    void setFlyMaxSpeed(double maxspeed)  { fly.MAXSPEED  = maxspeed; }//最大速度
    void setFlySafeDist(double safedist)  { fly.SAFE_DIST = safedist; }//安全距离
    void setFlyBfGamma (double bfgamma)   { fly.BF_GAMMA  = bfgamma;  }//该值越大，避障范围越小，贴的越近
    void setFlyXi(double xi)              { fly.XI        = xi;       }//该值越大，避障范围越大，贴的越远


private:
    FLY_Class fly;                                          
    int hz, bar_cnt;                                        
    ros::NodeHandle nh;                                     
    ros::Publisher pub_pose;       
    ros::ServiceServer barrier_service,aim_service;  
 // ros::ServiceClient flight_control_client;
    vector<BR_Class> br;
    vector<geometry_msgs::Point> allBarPositon;
    geometry_msgs::Point fly_aim;                           
   
   //添加障碍物
    bool barPoseCallback(barrier_avoid::my_point::Request &req,barrier_avoid::my_point::Response &res)
    {
        //新增一个障碍物
        BR_Class bar(bar_cnt,req.x,req.y,req.z,req.r);
        allBarPositon.push_back(bar.position);
        br.push_back(bar);
        //更新障碍物位置矩阵
        bar_cnt ++;
        res.result="ok";   
        return true;
    }
    //更新目标点
    bool flyAimCallback(barrier_avoid::my_point::Request &req,barrier_avoid::my_point::Response &res)
    {
        fly_aim.x=req.x;
        fly_aim.y=req.y;
        fly_aim.z=req.z;
        res.result="ok";   
        return true;
    }
};
