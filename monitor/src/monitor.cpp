#include "monitor/monitor.hpp"




namespace Monitor {

Monitor::Monitor(ros::NodeHandle& nh){
  
  pose_sub_ = nh.subscribe("eskf_fusion/pose",10, &Monitor::PositionCallBcak, this);

  ros::service::waitForService("/moniter_aimpose");
 
  aim_client = nh.serviceClient<barrier_avoid::my_point>("/monitor_aimpose");
  
//  aim_service = nh.advertiseService("/aimpose",&Monitor::AimCallBack,this);
  monitor::my_point aim_point;
  
  disp = distance(current_position,fly_aim);
  
  std::cout << disp;
  
  if(disp <= offset_aim){
    aim_service = nh.advertiseService("/aimpose",&Monitor::AimCallBack,this);
    aim_point.request.x = fly_aim.x;
    aim_point.request.y = fly_aim.y;
    aim_point.request.z = fly_aim.z;
    aim_client.call(aim_point);
  }
  
}

void Monitor::PositionCallBcak(const nav_msgs::Odometry& position_curr)
{
  current_position.x = position_curr.pose.pose.position.x;
  current_position.y = position_curr.pose.pose.position.y;
  current_position.z = position_curr.pose.pose.position.z;
  
}

bool Monitor::AimCallBack(monitor::my_point::Request& req, monitor::my_point::Response& res)
{
  fly_aim.x=req.x;
  fly_aim.y=req.y;
  fly_aim.z=req.z;
  res.result="ok";   
  return true;
}

float Monitor::distance(const geometry_msgs::Point& curr, const geometry_msgs::Point& aim)
{
  float curr_x = rad(curr.x);
  float curr_y = rad(curr.y);
  float curr_z = curr.z;
  float aim_x = rad(aim.x);
  float aim_y = rad(aim.y);
  float aim_z = aim.z;
  
  float a = fabs(curr_x - aim_x);
  float b = fabs(curr_y - aim_y);
  float c = fabs(curr_z - aim_z);
  
  float h = haveSin(a) + cos(curr_x)*cos(aim_x)*haveSin(b);
  float distance = 2 * EARTH_RADIUS * asin(sqrt(h));
  distance = sqrt(distance * distance + (curr_z - aim_z) * (curr_z - aim_z));
  
  return distance;
  
}

float Monitor::Monitor::rad(float degree)
{
  return PI * degree / 180.0;
}
  
float Monitor::haveSin(float x)
{
  float v = sin(x / 2);
  return v*v;
}

 
}



int main(int argc, char *argv[])
{
	ros::init(argc, argv, "moniter");
	ros::NodeHandle nh;
	Monitor::Monitor Monitor(nh);
	ros::spin();
	return 0;
}
