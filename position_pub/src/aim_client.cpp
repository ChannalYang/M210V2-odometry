#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include <fstream>
#include "barrier_avoid/my_point.h"
#include "stdio.h"
#include <queue>
#include <iostream>
#include <stdlib.h>

int main(int argc, char **argv)
{
 ros::init(argc,argv,"aim_client");
 ros::NodeHandle node;

 ros::service::waitForService("/aimpose");
 ros::ServiceClient aim_client = node.serviceClient<barrier_avoid::my_point>("/aimpose");
 barrier_avoid::my_point aim_point;

 std::queue<float> position_x, position_y, position_z;
 float x,y,z,temp;
 float dist[50][3];///home/dji/catkin_onboard/src/position_pub/src/aim_point.txt
 
 std::ifstream infile("/home/dji/catkin_onboard/src/position_pub/src/aim_point.txt");
 if(!infile.is_open()){
     ROS_INFO("aim_point generation failed!111");
     exit(1);
 }
 
 while(!infile.eof()){
   for(int i=0;;i++){
    infile >> x;
    position_x.push(x);
    infile >> y;
    position_y.push(y);
    infile >> z;
    position_z.push(z);
  }
}
  
  infile.close();
  std::cout << position_x.front();
  std::cout << position_y.front();
  std::cout << position_z.front();
  
 aim_point.request.x = position_x.front();
 aim_point.request.y = position_y.front();
 aim_point.request.z = position_z.front();

 position_x.pop();
 position_y.pop();
 position_z.pop();
 
 aim_client.call(aim_point);

 ROS_INFO("show person result:%s",aim_point.response.result.c_str());
 return 0;
}
