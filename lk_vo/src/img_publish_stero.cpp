#include<ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>
#include <sensor_msgs/Image.h> 
#include <vector>
#include <iostream>
#include <string.h>
#include<fstream>
#include<stdlib.h>
using namespace std;
unsigned int currentFrame=0;
string getFrameStr(unsigned int frame);
int main(int argc,char **argv){
  ros::init(argc,argv,"img_publish_meno");
  ros::NodeHandle nh;   
  ros::Publisher pub=nh.advertise<sensor_msgs::Image>("stereo_odometry/position_from_stereo",1);
  ros::Rate loop_rate(10);
    for(;currentFrame < 729 ;currentFrame++)
{
	string image_name = "/home/jack/桌面/光流/LK-VO/LK_VO-master/2011_09_26/2011_09_26_drive_0087_sync/image_03/data/" + getFrameStr(currentFrame) + ".png";
	if(!nh.ok()) break;
	cout<<getFrameStr(currentFrame)<<".png"<<endl;
	cv::Mat image = cv::imread(image_name, CV_LOAD_IMAGE_COLOR);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
	msg->header.frame_id=getFrameStr(currentFrame);
	pub.publish(msg);
        loop_rate.sleep();
    }

}

string getFrameStr(unsigned int frame)
{
	if(frame>9999)
		return "00000"+to_string(frame);
	else if(frame>999)
		return "000000"+to_string(frame);
	else if(frame>99)
		return "0000000"+to_string(frame);
	else if(frame>9)
		return "00000000"+to_string(frame);
	else if(frame<=9)
		return "000000000"+to_string(frame);
}
