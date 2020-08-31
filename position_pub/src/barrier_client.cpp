#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include <sstream>
#include "barrier_avoid/my_point.h"


int main(int argc, char **argv)
{
 ros::init(argc,argv,"barrier_client");
 ros::NodeHandle node;

 ros::service::waitForService("/barpose");
 ros::ServiceClient barrier_client = node.serviceClient<barrier_avoid::my_point>("/barpose");
 
 barrier_avoid::my_point bar_point;
 bar_point.request.x=6.0;
 bar_point.request.y=7.0;
 bar_point.request.z=2.0;
 bar_point.request.r=1.0;
 
 barrier_client.call(bar_point);

 barrier_avoid::my_point bar_point1;
 bar_point1.request.x=5.0;
 bar_point1.request.y=2.0;
 bar_point1.request.z=1.0;

 barrier_client.call(bar_point1);
 ROS_INFO("result:%s",bar_point.response.result.c_str());
 return 0;
}