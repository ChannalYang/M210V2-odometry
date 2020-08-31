#include "ros/ros.h"
#include <iostream>
#include<fstream>
//#include "libstereo-odometry.h"
#include "/home/dji/catkin_onboard/src/stereo_odometry/libstereo-odometry/include/libstereo-odometry.h"
#include <mrpt/hwdrivers/CCameraSensor.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CConfigFileMemory.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/otherlibs/tclap/CmdLine.h>
#include <mrpt/gui.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Path.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <cstring>  // size_t
#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <nav_msgs/Path.h>//ros path

// DJI SDK includes
#include <dji_osdk_ros/common_type.h>
#include <dji_osdk_ros/StereoVGASubscription.h>

using namespace std;
using namespace rso;
using namespace cv;
using namespace mrpt::system;
using namespace mrpt::obs;
using namespace mrpt::gui;
using namespace ros;
using namespace sensor_msgs;
using namespace cv_bridge;
using namespace Eigen;

#include <mrpt/version.h>
#if MRPT_VERSION>=0x130
using mrpt::obs::CObservationPtr;
#else
using mrpt::slam::CObservationPtr;
#endif


//全局变量
geometry_msgs::PoseStamped stereo_pose;    //飞机路径
nav_msgs::Path stereo_path;
ros::Publisher  stereo_pose_pub;               // 发布规划的路径
ros::Publisher  stereo_path_pub;
//CObservationStereoImages obj;
mrpt::hwdrivers::CCameraSensor myCam;       // The generic image source
CStereoOdometryEstimator stereo_odom_engine;// Declare Stereo Odometry object:
std::vector<std::string>  paramSections;
//const string arg_app_cfg_file="/home/dji/catkin_onboard/src/stereo_odometry/src/vo-db.ini"; 
//const string arg_app_cfg_file="/home/dji/catkin_onboard/src/stereo_odometry/src/vo-true.ini"; 
const string arg_app_cfg_file="/home/dji/catkin_onboard/src/stereo_odometry/src/vo.ini"; 
vector<double> v_pose(6,0.0);// read pose of the camera on the robot
CPose3D pose;
CStereoOdometryEstimator::TStereoOdometryRequest odom_request;
ros::ServiceClient stereo_vga_subscription_client;
dji_osdk_ros::StereoVGASubscription subscription;
 CObservationStereoImagesPtr obs;

bool vga_imgs_subscribed = false;
//CObservationStereoImagesPtr obj;
//CObservationStereoImages obj;
bool end = false; // Signal for closing if the user command so.
bool pause_each_frame=false;
static int frm_count = 0;

CObservationStereoImages obj;

 bool stereoflag = 0;

		
// CPose3D poseOnRobot(0,0,0,DEG2RAD(-90),DEG2RAD(0),DEG2RAD(-100) );	// read this from app config file
CPose3D poseOnRobot(v_pose[0],v_pose[1],v_pose[2],DEG2RAD(v_pose[3]),DEG2RAD(v_pose[4]),DEG2RAD(v_pose[5]));
		

void ros2mrpt(const sensor_msgs::ImageConstPtr& left, const sensor_msgs::ImageConstPtr& right, CObservationStereoImagesPtr& obs)
{
    cv::Mat Qinleft = cv_bridge::toCvCopy(left, "mono8")->image;

    IplImage ipl = Qinleft;
    obs->imageLeft.loadFromIplImage(&ipl);

   // CvImage* frame2 = cv_bridge::toCvCopy(right,"mono8").get();
    cv::Mat Qinright = cv_bridge::toCvCopy(right,"mono8")->image;
    IplImage ipr = Qinright;
    obs->imageRight.loadFromIplImage(&ipr);

    stereoflag = 1;
}



bool imgSubscriptionHelper(dji_osdk_ros::StereoVGASubscription &service)
{
  std::string action;
  if(service.request.unsubscribe_vga){
    action = "unsubscribed";
  }else{
    action = "subscribed";
  }

stereo_vga_subscription_client.call(service);
  if (service.response.result == true)
  {
    ROS_INFO("Successfully %s to VGA images", action.c_str());
    if(service.request.unsubscribe_vga){
      vga_imgs_subscribed = false;
    }else{
      vga_imgs_subscribed = true;
    }
  }
  else
  {
    ROS_ERROR("Failed to %s to VGA images", action.c_str());
    return false;
  }

  return true;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"stereo_vo");
    ros::NodeHandle nh;
    stereo_pose_pub  = nh.advertise<geometry_msgs::PoseStamped>("stereo_odometry/position_from_stereo", 10);        // 发布消息
    stereo_path_pub  = nh.advertise<nav_msgs::Path>("stereo_pose", 10);        // 发布消息

    
    stereo_vga_subscription_client = nh.serviceClient<dji_osdk_ros::StereoVGASubscription>("stereo_vga_subscription");

     subscription.request.vga_freq = subscription.request.VGA_20_HZ;
     subscription.request.front_vga = 1;
     subscription.request.unsubscribe_vga = 0;
     
      if(!imgSubscriptionHelper(subscription)){
      return -1;
       }
     ros::Duration(1).sleep();
     
    message_filters::Subscriber<sensor_msgs::Image> imageL;
    message_filters::Subscriber<sensor_msgs::Image> imageR;
    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> *topic_synchronizer;
  
     imageL.subscribe(nh, "dji_osdk_ros/stereo_vga_front_left_images", 1);
     imageR.subscribe(nh, "dji_osdk_ros/stereo_vga_front_right_images", 1);

   //imageL.subscribe(nh, "/stereo_depth_perception/rectified_vga_front_left_image", 1);
  // imageR.subscribe(nh, "/stereo_depth_perception/rectified_vga_front_right_image", 1);

   //   imageL.subscribe(nh, "/cam0/image_raw", 1);
   //   imageR.subscribe(nh, "/cam1/image_raw", 1);
    //时间同步,接受双目图像
     
  
     topic_synchronizer = new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image>(imageL, imageR, 10);
  //   typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
   //  message_filters::Synchronizer<sync_pol> sync(sync_pol(10), imageL,imageR);
     
     subscription.request.vga_freq = subscription.request.VGA_20_HZ;
     subscription.request.front_vga = 1;
     subscription.request.unsubscribe_vga = 0;
     
    //  if(!imgSubscriptionHelper(subscription)){
    //  return -1;
    //   }
    //  ros::Duration(1).sleep();

     
     topic_synchronizer->registerCallback(boost::bind(&ros2mrpt, _1, _2, CObservationStereoImagesPtr (&obj)));
     
   //  sync.registerCallback(boost::bind(&ros2mrpt,_1,_2));

    // // const string sCfgFile = arg_sensor_cfg_file.getValue();
    // // const string sSection = arg_section_name.getValue();
    // // myCam.loadConfig(mrpt::utils::CConfigFile(sCfgFile),sSection);

 //    	std::vector<std::string> paramSections;
 	paramSections.push_back("RECTIFY");
 	paramSections.push_back("DETECT");
 	paramSections.push_back("MATCH");
 	paramSections.push_back("IF-MATCH");
 	paramSections.push_back("LEAST_SQUARES");
 	paramSections.push_back("GUI");
 	paramSections.push_back("GENERAL");

  //    CStereoOdometryEstimator stereo_odom_engine;// Declare Stereo Odometry object:
      
     // const string arg_app_cfg_file="/home/dji/catkin_onboard/src/stereo_odometry/src/vo.ini"; 
      CConfigFile app_iniFile( arg_app_cfg_file );

      stereo_odom_engine.loadParamsFromConfigFileName( arg_app_cfg_file, paramSections );
      stereo_odom_engine.setVerbosityLevel( app_iniFile.read_int("GENERAL","vo_verbosity",2,false) );
      
      
      
      // read pose of the camera on the robot
      //vector<double> v_pose(6,0.0);
	app_iniFile.read_vector( "GENERAL", "camera_pose_on_robot", v_pose, v_pose, false);
	
	if( stereo_odom_engine.params_general.vo_debug || stereo_odom_engine.params_general.vo_save_files )
		createDirectory( stereo_odom_engine.params_general.vo_out_dir );
		

	// CPose3D poseOnRobot(0,0,0,DEG2RAD(-90),DEG2RAD(0),DEG2RAD(-100) );	// read this from app config file
	CPose3D poseOnRobot(v_pose[0],v_pose[1],v_pose[2],DEG2RAD(v_pose[3]),DEG2RAD(v_pose[4]),DEG2RAD(v_pose[5]));

        //what??
	//FILE *f = mrpt::system::os::fopen( mrpt::format("%s/camera_pose.txt", stereo_odom_engine.params_general.vo_out_dir.c_str()).c_str(),"wt");

      
		
	//CStereoOdometryEstimator::TStereoOdometryRequest odom_request;
	CConfigFile camera_cfgFile( arg_app_cfg_file );
	odom_request.stereo_cam.loadFromConfigFile( "CAMERA", camera_cfgFile );
	//std::cout <<"camera      :" << odom_request.stereo_cam.rightCamera.cx <<"camera      :"<< odom_request.stereo_cam.leftCamera.fy <<endl;		
	float cy = odom_request.stereo_cam.rightCamera.cy();
      //myCam.loadConfig(mrpt::utils::CConfigFile(arg_app_cfg_file),sSection);
       //myCam.initialize();

    // imshow("left",imgL);
    // imshow("right",imgR);

    CObservationStereoImagesPtr obs = CObservationStereoImagesPtr(&obj);
    
  /*  ofstream dataFile;
    dataFile.open("/home/dji/catkin_onboard/src/stereo_odometry/src/delta_position.txt", std::ofstream::app);
    int count=0;
    */
    while(ros::ok()){
        ros::spinOnce();
        if(stereoflag){
            stereoflag = 0;
            cout << "Frame:-------------------------------------------- " << endl;

                CStereoOdometryEstimator::TStereoOdometryResult  odom_result;
                //CObservationStereoImagesPtr obs = CObservationStereoImagesPtr(&obj);
                // obs.copy(obj.Create());

                odom_request.stereo_imgs = obs;

               cout <<"left_image size :"<<odom_request.stereo_imgs->imageLeft.getSize() << "\t";
               cout <<"right_image size :"<<odom_request.stereo_imgs->imageRight.getSize() << "\t";


                // Estimate visual odometry:
                stereo_odom_engine.processNewImagePair( odom_request, odom_result );

                // Compute the current position
                cout<<"odom_result.valid:   "<<odom_result.valid<<endl;
                if( odom_result.valid )
                {

                 //   CPose3D pri_pose(pose);

                    //// composition
                    // CPose3D aux;
                    // aux.composeFrom(poseOnRobot,odom_result.outPose);
                    // aux.inverseComposeFrom(aux,poseOnRobot);
                    // pose.composeFrom(pose,aux);
                    // cout << "CPose3D: " << pose << endl;

                    // matrix version
                    CMatrixDouble44 mat01,mat02,mat03,mat04,mat05;
                    pose.getHomogeneousMatrix(mat01);					    // pose
                    poseOnRobot.getHomogeneousMatrix(mat02);			// k
                    odom_result.outPose.getHomogeneousMatrix(mat03);	// deltaPose

                    mat03(0,3)=mat03(0,3)+0.0057;
                    mat03(1,3)=mat03(0,3)-0.00009;

                    mat04 = mat02*mat03*mat02.inverse();
                    CPose3D pose1 = CPose3D(mat01);
                    CPose3D pose_cul = CPose3D(mat04);

                    mat05.multiply(mat04,mat01);
                    pose = CPose3D(mat05);//  x, y, z,  yaw,  pitch, roll;
		   // cout<<"befor transform matrix:"<<mat01<<endl;
		  /*  if(count<2000){
		    cout<<"delta matrix :"<<mat03<<endl;
		    dataFile<<"x="<<mat03(0,3)<<"   ";
		    dataFile<<"y="<<mat03(1,3)<<"   ";
		    dataFile<<"z="<<mat03(2,3)<<endl;
		      count++;
		    }
		    else{
		      dataFile.close();
		    }*/
		 //   cout<<"fina pose matrix"<<mat05<<endl;
                     //cout << "Matrix: "  << mat01 << endl << mat04 << endl << mat05 << endl;

		  //cout<<"delta matrix :"<<mat03<<endl;;
                 //  cout<<"pose5:::::"<<pose <<endl;
                  // cout << "	done: [Resulting pose: " << odom_result.outPose << "]" << endl;



                    //  //发布位姿信息
                    //pose od ov_result
                   stereo_pose.header.frame_id = "stereoPose";
                   stereo_pose.header.stamp = ros::Time::now();
                   stereo_pose.pose.position.x = pose.m_coords[0];
                   stereo_pose.pose.position.y = pose.m_coords[1];
                   stereo_pose.pose.position.z = pose.m_coords[2];
                  //  stereo_pose.pose.position.x = pose.x();
                  //  stereo_pose.pose.position.y = pose.y();
                  //  stereo_pose.pose.position.z = pose.z();
                  //  stereo_pose.pose.position.x = pose.yaw();
                  //  stereo_pose.pose.position.y = pose.pitch();
                  //  stereo_pose.pose.position.z = pose.roll();
                    // stereo_pose.pose.position.x = odom_result.outPose.x();
                    // stereo_pose.pose.position.y = odom_result.outPose.y();
                    // stereo_pose.pose.position.z = odom_result.outPose.z();
                  // cout<<"pose::::::"<<pose.x()<<" "<<pose.y()<<" "<<pose.z()<<" "<<endl;

                   //Eigen::AngleAxisd rotation_vector(pose.x_incr(), pose.y_incr(), pose.z_incr());
                   Eigen::Vector3d euler_angle (pose.yaw(), pose.pitch(), pose.roll());

                   Eigen::AngleAxisd rollAngle(AngleAxisd(euler_angle(2),Vector3d::UnitX()));
                   Eigen::AngleAxisd pitchAngle(AngleAxisd(euler_angle(1),Vector3d::UnitY()));
                   Eigen::AngleAxisd yawAngle(AngleAxisd(euler_angle(0),Vector3d::UnitZ()));

                   Eigen::Quaterniond quaternion;
                   quaternion=yawAngle*pitchAngle*rollAngle; 
             //   Eigen::Quaterniond ortn=Eigen::Quaterniond() ;//随便设置的，只用了坐标
                   stereo_pose.pose.orientation.x = quaternion.x();
                   stereo_pose.pose.orientation.y = quaternion.y();
                   stereo_pose.pose.orientation.z = quaternion.z();
                   stereo_pose.pose.orientation.w = quaternion.w();
                  // cout<<"orientation::::::"<<quaternion.x()<<" "<<quaternion.y()<<" "<<quaternion.z()<<" "<<endl;
                  // ros_stereo_path.poses.push_back(mypose)
                   stereo_pose_pub.publish(stereo_pose);


                   //rviz path
                   stereo_path.header.frame_id= "stereoPath";
                   stereo_path.header.stamp = ros::Time::now();
                   stereo_path.poses.push_back(stereo_pose);
                   stereo_path_pub.publish(stereo_path);
                }
                else
                {
                    DUMP_VO_ERROR_CODE( odom_result.error_code )
                }


                cout<<"outPutPose:   "<< stereo_pose.pose.position << stereo_pose.pose.orientation <<endl;

                cout << "Frame:   finished."<<frm_count<<"-----------------------------------------" << endl << endl;
               frm_count++;
        }
    }


    ros::spin(); 
    ROS_INFO("im ok");
    return 0;
}

