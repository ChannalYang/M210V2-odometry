#include <ros/ros.h>
#include "barrier_avoid/my_point.h"
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/StdVector>
#include <monitor/my_point.h>
#include <math.h>

namespace Monitor{
  
#define PI 3.141592657
#define EARTH_RADIUS 6378137
#define offset_aim 0.05
  
  class Monitor{
    
  public:
    Monitor(ros::NodeHandle& nh);
    
  private:
    ros::NodeHandle nh;
    ros::Subscriber pose_sub_;
    ros::ServiceServer aim_service;
    ros::ServiceClient aim_client;
    geometry_msgs::Point fly_aim, current_position;
    float disp;
    
    
    void PositionCallBcak(const nav_msgs::Odometry&);
    bool AimCallBack(monitor::my_point::Request&,monitor::my_point::Response&);
    float distance(const geometry_msgs::Point&, const geometry_msgs::Point&);
    float rad(float degree);
    float haveSin(float x);
    
  };
  
  
  
}