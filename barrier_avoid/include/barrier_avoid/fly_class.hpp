#ifndef __CRAZYFLIE_CLASS_HPP__
#define __CRAZYFLIE_CLASS_HPP__

#include <vector>
#include <math.h>
#include "Array.hpp"
#include <nav_msgs/Path.h>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "barrier_class.hpp"
#include <dji_osdk_ros/FlightTaskControl.h>
#include <ros/ros.h>
#include <dji_osdk_ros/common_type.h>


using namespace quadprogpp;
using namespace std;
using namespace Eigen;

using namespace dji_osdk_ros;
    
ros::ServiceClient flight_control_client;

class FLY_Class{
public:
    double MAXSPEED, SAFE_DIST, BF_GAMMA, XI;   //最大速度，飞机安全距离，避障参数
    int BAR_CNT;                                //障碍物数量
    nav_msgs::Path ros_path;                    //飞机路径
    static MatrixXd allPosition;                //所有飞机坐标，其实只有一个
    std::vector<double>  aim_dirc, v_set;       //目标方向，速度设定
    geometry_msgs::Point        cur_posit;      //当前位置
    geometry_msgs::Point        aim_posit;      //局部规划的目标点
    //geometry_msgs::Pose              pos;     //坐标
    //geometry_msgs::Quaternion quaternion;     //朝向


    FLY_Class() {   };
    void myInit(int nox, ros::NodeHandle n_ros){
        No_X    = nox;//飞机编号
        n       = n_ros;

        BAR_CNT=0;
        MAXSPEED  =  0.05; //最大速度
        SAFE_DIST =  1.5;   //安全距离
        BF_GAMMA  =  1  ;  //该值越大，避障范围越小，贴的越近
        XI        =  0.2; //该值越大，避障范围越大，贴的越远

        sub_pose = n.subscribe("eskf_fusion/pose", 10, &FLY_Class::subPose, this);     
        pub_path  = n.advertise<nav_msgs::Path>("crazyflie/cmd_vel", 1);       
    }
 
    /*
    * 二次规划计算 Ver2.0
    * example:
    * min f(x) = 0.5*x'*G*x + g0'*x;
    * s.t.
    *      E*x  = e0;
    *      I*x <= i0;
    */
    double avoidBarrier(double xr, double yr, double zr,vector<geometry_msgs::Point> allBarPositon,vector<BR_Class> br, bool roubust){
        //更新障碍物作标矩阵
        BAR_CNT= allBarPositon.size();
        MatrixXd allP(3,BAR_CNT);
        for (size_t i = 0; i < BAR_CNT; i++)
        {
            allP(0,i)=allBarPositon[i].x;
            allP(1,i)=allBarPositon[i].y;
            allP(2,i)=allBarPositon[i].z;
        }
        cout<<"allbarPosition:  "<<allP<<endl;

        // 设定二次规划相关程序
        int n = 3, m = BAR_CNT;

        MatrixXd        u, A, b;
        quadprogpp::Matrix<double> G, CE, CI;
        quadprogpp::Vector<double> g0, ce0, ci0, x;
        u.resize(n, 1);
        A = MatrixXd::Zero(n, BAR_CNT);//
        b = MatrixXd::Zero(1, BAR_CNT);
        G.resize(n, n);         g0.resize(n);
        CE.resize(n, 0);        ce0.resize(0);
        CI.resize(n, BAR_CNT);  ci0.resize(BAR_CNT);
        x.resize(n);
        aim_dirc.resize(n); v_set.resize(n);
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {G[i][j]  = 0;}
            G[i][i] = 1;
        }
        //计算初始移动方向向量
        aim_dirc[0] = -cur_posit.x+xr;
        aim_dirc[1] = -cur_posit.y+yr;
        aim_dirc[2] = -cur_posit.z+zr;
        float vecNorm,res = 0.0;
        int i_mat = 0;
        float temp = 0;//vx2+vy2+vz2

        for (int idx = 0; idx < aim_dirc.size(); idx++){
            temp += aim_dirc[idx]*aim_dirc[idx];
        }

        // 避开障碍物
        for(int idx = 0; idx < BAR_CNT; idx++){
            A.col(i_mat) = -(allPosition.col(No_X) - allP.col(idx));
            vecNorm = A.col(i_mat).norm();//到障碍物的距离
            b(0, i_mat) = vecNorm*(BF_GAMMA*(vecNorm - (br[idx].safedist+SAFE_DIST)*(1+XI*roubust)));
            cout<<"bar safedist"<<br[idx].safedist <<endl;
           i_mat++;
        }      
        for (int i = 0; i < m; i++) {
            for (int j = 0; j < n; j++) {
                CI[j][i] = -A(j, i);
            }
            ci0[i] = b(0, i);
        }
        for(int i = 0; i < n; i++)
        { 
            g0[i]  = -aim_dirc[i]; 
             x[i] = aim_dirc[i]; 
        }
        res = quadprogpp::solve_quadprog(G, g0, CE, ce0, CI, ci0, x);//通过求解二次规划问题，计算速度
        double max_speed=0;
        for (int i = 0; i < n; i++) {
           v_set[i] = x[i];//速度，位移增量
           if(abs(v_set[i])>max_speed) max_speed=abs(v_set[i]);
        }
        //cout<<"maxspeed:  "<<max_speed<<endl;
          if(max_speed>MAXSPEED)
          {
            for (size_t i = 0; i <n; i++)
            {
              v_set[i]=v_set[i]*MAXSPEED/max_speed;
            }
          }
        //cout<<"all_positoin:  "<<allPosition<<endl;
        return res;
    }

   //发布速度增量
    void pubPath(void) {   
        //更新位置
        cur_posit.x = cur_posit.x + v_set[0];
        cur_posit.y = cur_posit.y + v_set[1];
        cur_posit.z = cur_posit.z + v_set[2];          

        
        //发布位姿信息
        ros_path.header.frame_id = "cfPose";
        ros_path.header.stamp = ros::Time::now();
        geometry_msgs::PoseStamped pose;
        pose.header = ros_path.header;
        pose.pose.position.x =cur_posit.x;
        pose.pose.position.y =cur_posit.y;
        pose.pose.position.z = cur_posit.z;
        Eigen::Quaterniond ortn=Eigen::Quaterniond() ;//随便设置的，只用了坐标
        pose.pose.orientation.x = 1;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 0;
        ros_path.poses.push_back(pose);

        //发布话题
        pub_path.publish(ros_path);
    }
    
    void subPose(const  nav_msgs::OdometryConstPtr& msg)
    {
        cur_posit.x = msg->pose.pose.position.x;
        cur_posit.y = msg->pose.pose.position.y;
        cur_posit.z = msg->pose.pose.position.z; 
        allPosition(0, No_X) = cur_posit.x;
        allPosition(1, No_X) = cur_posit.y;
        allPosition(2, No_X) = cur_posit.z;
    }

    //设置位置
  void setFlyPose(double x, double y, double z){
      cur_posit.x=x;
      cur_posit.y=y;
      cur_posit.z=z;
      allPosition(0, No_X) = x;
      allPosition(1, No_X) = y;
      allPosition(2, No_X) = z;
  }
  


private:
    int No_X;
    ros::NodeHandle n;                      // ROS 节点句柄
    ros::Publisher  pub_path;               // 发布规划的路径
    ros::Subscriber sub_pose;               // 订阅实际位姿
};


 bool takeoff()
{
  dji_osdk_ros::FlightTaskControl flightTaskControl;
  flightTaskControl.request.task = dji_osdk_ros::FlightTaskControl::Request ::TASK_TAKEOFF;
  flight_control_client.call(flightTaskControl);

  return flightTaskControl.response.result;
}

bool land()
{
  dji_osdk_ros::FlightTaskControl flightTaskControl;
  flightTaskControl.request.task = dji_osdk_ros::FlightTaskControl::Request ::TASK_LAND;
  flight_control_client.call(flightTaskControl);

  return flightTaskControl.response.result;
}


MatrixXd FLY_Class::allPosition = MatrixXd::Zero(3, 1);
#endif
