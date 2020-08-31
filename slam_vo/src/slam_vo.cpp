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
#include <slam_vo/config.hpp>
#include <slam_vo/visual_odometry.hpp>
#include<slam_vo/camera.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <message_filters/time_synchronizer.h>

#include <stdio.h>
#include <geometry_msgs/PoseStamped.h>

using namespace slam_vo;
using namespace cv;
using namespace std;
using namespace message_filters;

VisualOdometry::Ptr vo(new VisualOdometry) ;

int count11=0;
ros::Subscriber img_left_sub_;
ros::Subscriber img_right_sub_;
ros::Publisher vo_pose_pub_;
ros::Subscriber Tcw_initial_sub_;



void VOposeCallback(const sensor_msgs::ImageConstPtr& msg_left,const sensor_msgs::ImageConstPtr& msg_right);//,const geometry_msgs::PoseStampedConstPtr& msg_Tcw);
 

int main ( int argc, char** argv )
{

    ros::init(argc,argv,"slam_vo");
    ros::NodeHandle nh;
		slam_vo::Config::setParamFile ("/home/dji/catkin_onboard/src/slam_vo/src/m210_stereo_param_maybethebest.yaml");
    Subscriber<sensor_msgs::Image>img_left_sub_(nh,"/stereo_depth_perception/rectified_vga_front_left_image", 10);
    
    Subscriber<sensor_msgs::Image>  img_right_sub_ (nh,"/stereo_depth_perception/rectified_vga_front_right_image", 10);  //need to change,right image

    Subscriber<geometry_msgs::PoseStamped> Tcw_initial_sub_(nh,"/GPS_IMU",10);
//geometry_msgs::PoseStamped,const geometry_msgs::PoseStampedConstPtr& msg_Tcw
    
    vo_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("dji_osdk_ros/positon_stereo",10);
    
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> *topic_synchronizer;

    topic_synchronizer = new message_filters::TimeSynchronizer <sensor_msgs::Image, sensor_msgs::Image>(img_left_sub_, img_right_sub_,10);
    
    topic_synchronizer->registerCallback(boost::bind(&VOposeCallback,_1, _2));//three messages;

    ros::spin();
    
    return 0;
}


void VOposeCallback(const sensor_msgs::ImageConstPtr& msg_left,const sensor_msgs::ImageConstPtr& msg_right){
    
    std::cout <<"it is "<<count11<<" frame"<<endl;
    count11++;
    cout<<"                  get id  left from DJI::"<<msg_left->header.frame_id<<endl; 
    cout<<"                  get id  right  from DJI::"<<msg_right->header.frame_id<<endl; 
    //cout<<"                  get id  pose  from GPS_IMU::"<<msg_Tcw->header.frame_id<<endl;
    

    /**************VO which can be used succesuffuly *****************/
     CameraParam::Ptr camera(new CameraParam(0)) ;

    Mat left=cv_bridge::toCvShare(msg_left, "bgr8")->image;
    Mat right=cv_bridge::toCvShare(msg_right, "bgr8")->image;

     if (left.data==nullptr || right.data==nullptr) {	
       cout<<"NO CAMMERA"<<endl;
       return;
    }
    
    Frame::Ptr pFrame = Frame::createFrame();
     pFrame->camera_ = camera;
     pFrame->left_ = left;
     pFrame->right_ = right;
    /// pFrame->time_stamp=msg_left->header.stamp.toSec();//the time_stamp is same as msg_depth because of TimeSynchronizer;
    // if(vo->flag == 0 ) 
    // pFrame->T_c_w_=msg_Tcw;
		//boost::timer timer;
     vo->addFrame( pFrame ); 
  //  cout<<"VO cost time "<<timer.elapsed()<<endl;
     SE3 Twc = pFrame->T_c_w_.inverse();
    
     geometry_msgs::PoseStamped msg_pose;
      
     msg_pose.pose.position.x=Twc.translation() (0,0);
     msg_pose.pose.position.y=Twc.translation() (1,0);  
     msg_pose.pose.position.z=Twc.translation() (2,0);
   
     msg_pose.pose.orientation.x=Twc.unit_quaternion().coeffs()(0,0);
     msg_pose.pose.orientation.y=Twc.unit_quaternion().coeffs()(1,0);   
     msg_pose.pose.orientation.z=Twc.unit_quaternion().coeffs()(2,0);
     msg_pose.pose.orientation.w=Twc.unit_quaternion().coeffs()(3,0);
     msg_pose.header=msg_left->header;
    /*
     msg_pose.pose.position.x=msg_Tcw->pose.position.x;
     msg_pose.pose.position.y=msg_Tcw->pose.position.y;  
     msg_pose.pose.position.z=msg_Tcw->pose.position.z;
     
     msg_pose.pose.orientation.x=msg_Tcw->pose.orientation.x;
     msg_pose.pose.orientation.y=msg_Tcw->pose.orientation.y;
     msg_pose.pose.orientation.z=msg_Tcw->pose.orientation.z;
     msg_pose.pose.orientation.w=msg_Tcw->pose.orientation.w;
    */

    ROS_INFO("Q :::x= %f y= %f z= %f w= %f \n\n",
	     msg_pose.pose.orientation.x,
	     msg_pose.pose.orientation.y,
	     msg_pose.pose.orientation.z,
	     msg_pose.pose.orientation.w
	);
    
    ROS_INFO("POSE :::x= %f y= %f z= %f  \n",msg_pose.pose.position.x,
	     msg_pose.pose.position.y,
	     msg_pose.pose.position.z

	);   
    vo_pose_pub_.publish(msg_pose);	
   
}    

