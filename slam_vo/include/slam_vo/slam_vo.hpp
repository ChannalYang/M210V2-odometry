#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose.h>
#include "config.hpp"
#include "visual_odometry.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <message_filters/time_synchronizer.h>

#include <stdio.h>

class slam_vo_{
public:
  slam_vo(ros::NodeHandle& nh);
  ~slam_vo();
  
  void LeftImgCallback(const sensor_msgs::Image& left_image);
  void DispImgCallback(const sensor_msgs::Image& disp_image);
  
  void VOposeCallback(const sensor_msgs::ImageConstPtr& msg_stero,const sensor_msgs::ImageConstPtr& msg_depth);

public:
  ros::Subscriber img_left_sub_;
  ros::Subscriber img_right_sub_;
  ros::Publisher vo_pose_pub_;
  ros::Subscriber Tcw_initial_sub_;
  
};





