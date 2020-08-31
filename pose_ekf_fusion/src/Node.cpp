#include "pose_ekf_fusion/Node.hpp"
#include "ros/ros.h"

//x--longtitude
//y--latitude
//z--height(m)


namespace eskf
{
  
ESKFNode::ESKFNode(ros::NodeHandle& nh)
{

  ROS_INFO("Subscribing to imu.");
  subImu_ = nh.subscribe("dji_osdk_ros/imu", 1000, &ESKFNode::inputCallback, this,  ros::TransportHints().tcpNoDelay(true));
  
/*  ROS_INFO("Subscribing to extended state");
  subExtendedState_ = nh_.subscribe("extended_state", 1, &ESKFNode::extendedStateCallback, this);
*/

  int fusion_mask = default_fusion_mask_;
  ros::param::get("~fusion_mask", fusion_mask);

  if((fusion_mask & MASK_EV_POS) || (fusion_mask & MASK_EV_YAW) || (fusion_mask & MASK_EV_HGT)) {
    ROS_INFO("Subscribing to vision");
    subVisionPose_ = nh.subscribe("stereo_odometry/position_from_stereo", 10, &ESKFNode::visionCallback, this);
  } 
//  if((fusion_mask & MASK_GPS_POS) || (fusion_mask & MASK_GPS_VEL) || (fusion_mask & MASK_GPS_HGT)) {
//     ROS_INFO("Subscribing to gps");
//     subGpsPose_ = nh.subscribe("gps_imu_position/inertial_pose", 10, &ESKFNode::gpsCallback, this);
//  } 
 
 /* if(fusion_mask & MASK_OPTICAL_FLOW) {
    ROS_INFO("Subscribing to optical_flow");
    subOpticalFlowPose_ = nh_.subscribe("optical_flow", 1, &Node::opticalFlowCallback, this);
  }*/

  eskf_.setFusionMask(fusion_mask);

  pubPose_ = nh.advertise<nav_msgs::Odometry>("eskf_fusion/pose", 10);

  int publish_rate = default_publish_rate_;
  ros::param::get("~publish_rate", publish_rate);
  pubTimer_ = nh.createTimer(ros::Duration(1.0f/publish_rate), &ESKFNode::publishState, this);
}

void ESKFNode::inputCallback(const sensor_msgs::ImuConstPtr& imuMsg) {

  vec3 wm = vec3(imuMsg->angular_velocity.x, imuMsg->angular_velocity.y, imuMsg->angular_velocity.z); //  measured angular rate
  vec3 am = vec3(imuMsg->linear_acceleration.x, imuMsg->linear_acceleration.y, imuMsg->linear_acceleration.z); //  measured linear acceleration
  
  std::cout << "wm" << wm[0] << "\t" << wm[1] << "\t" << wm[2] << std::endl;
  std::cout << "am" << am[0] << "\t" << am[1] << "\t" << am[2] << std::endl;
  
  if (prevStampImu_.sec != 0) {
    const double delta = (imuMsg->header.stamp - prevStampImu_).toSec();
    std::cout << "delta" << delta << std::endl;

    if (!init_) {
      init_ = true;
      ROS_INFO("Initialized ESKF");
    }

    //  run kalman filter
    eskf_.run(wm, am, static_cast<uint64_t>(imuMsg->header.stamp.toSec()*1e6f), delta);
  }
  prevStampImu_ = imuMsg->header.stamp;
  
  const vec3 imu_position = eskf_.getPosition();
  std::cout << "imu position" << imu_position[0] << "\t" << imu_position[1] << "\t" << imu_position[2] << std::endl;
  const vec3 imu_velocity = eskf_.getVelocity();
  std::cout << "imu velocity" << imu_velocity[0] << "\t" << imu_velocity[1] << "\t" << imu_velocity[2] << std::endl;
}
  
void ESKFNode::visionCallback(const geometry_msgs::PoseStampedConstPtr& poseMsg) {
  if(prevStampVisionPose_.sec != 0) {
    const double delta = (poseMsg->header.stamp - prevStampVisionPose_).toSec();
    // get measurements
    quat z_q = quat(poseMsg->pose.orientation.w, poseMsg->pose.orientation.x, poseMsg->pose.orientation.y, poseMsg->pose.orientation.z);
    vec3 z_p = vec3(poseMsg->pose.position.x, poseMsg->pose.position.y, poseMsg->pose.position.z);
    // update vision
    eskf_.updateVision(z_q, z_p, static_cast<uint64_t>(poseMsg->header.stamp.toSec()*1e6f), delta);
  }
  prevStampVisionPose_ = poseMsg->header.stamp;
}

void ESKFNode::gpsCallback(const geometry_msgs::PoseStampedConstPtr& gpsMsg) {
  
  std::cout << "gps" << gpsMsg->pose.position.x << "\t" << gpsMsg->pose.position.y << "\t" << gpsMsg->pose.position.z << std::endl;
  
  if (prevStampGpsPose_.sec != 0) {
    const double delta = (gpsMsg->header.stamp - prevStampGpsPose_).toSec();
    // get gps measurements
    vec3 z_v = vec3(gpsMsg->pose.orientation.x, gpsMsg->pose.orientation.y, gpsMsg->pose.orientation.z);
    vec3 z_p = vec3(gpsMsg->pose.position.x, gpsMsg->pose.position.y, gpsMsg->pose.position.z);
    // update gps
    eskf_.updateGps(z_v, z_p, static_cast<uint64_t>(gpsMsg->header.stamp.toSec() * 1e6f), delta);
  }
  prevStampGpsPose_ = gpsMsg->header.stamp;
}

/*
void ESKFNode::opticalFlowCallback(const mavros_msgs::OpticalFlowRadConstPtr& opticalFlowMsg) {
  if (prevStampOpticalFlowPose_.sec != 0) {
    const double delta = (opticalFlowMsg->header.stamp - prevStampOpticalFlowPose_).toSec();
    // get optical flow measurements
    vec2 int_xy = vec2(opticalFlowMsg->integrated_x, opticalFlowMsg->integrated_y);
    vec2 int_xy_gyro = vec2(opticalFlowMsg->integrated_xgyro, opticalFlowMsg->integrated_ygyro);
    uint32_t integration_time = opticalFlowMsg->integration_time_us;
    scalar_t distance = opticalFlowMsg->distance;
    uint8_t quality = opticalFlowMsg->quality;
    // update optical flow
    eskf_.updateOpticalFlow(int_xy, int_xy_gyro, integration_time, distance, quality, static_cast<uint64_t>(opticalFlowMsg->header.stamp.toSec() * 1e6f), delta);
  }
  prevStampOpticalFlowPose_ = opticalFlowMsg->header.stamp;
}
*/

void ESKFNode::rangeFinderCallback(const sensor_msgs::RangeConstPtr& rangeMsg) {
  if (prevStampRangeFinderPose_.sec != 0) {
    const double delta = (rangeMsg->header.stamp - prevStampRangeFinderPose_).toSec();
    // get rangefinder measurements
    eskf_.updateRangeFinder(rangeMsg->range, static_cast<uint64_t>(rangeMsg->header.stamp.toSec() * 1e6f), delta);
  }
  prevStampRangeFinderPose_ = rangeMsg->header.stamp;
}

void ESKFNode::publishState(const ros::TimerEvent&) {

  // get kalman filter result
  const quat e2g = eskf_.getQuat();
  const vec3 position = eskf_.getPosition();
  const vec3 velocity = eskf_.getVelocity();

  static size_t trace_id_ = 0;
  std_msgs::Header header;
  header.frame_id = "path";
  header.seq = trace_id_++;
  header.stamp = ros::Time::now();

  nav_msgs::Odometry odom;
  odom.header = header;
  odom.pose.pose.position.x = position[0];
  odom.pose.pose.position.y = position[1];
  odom.pose.pose.position.z = position[2];
  odom.twist.twist.linear.x = velocity[0];
  odom.twist.twist.linear.y = velocity[1];
  odom.twist.twist.linear.z = velocity[2];
  odom.pose.pose.orientation.w = e2g.w();
  odom.pose.pose.orientation.x = e2g.x();
  odom.pose.pose.orientation.y = e2g.y();
  odom.pose.pose.orientation.z = e2g.z();

  std::cout << "pose " << position[0] << "\t" << position[1] << "\t" << position[2] << "\t" << std::endl;
  std::cout << "velocity" << velocity[0] << "\t" <<  velocity[1] << "\t" << velocity[2] << "\t" << std::endl;
  
  pubPose_.publish(odom);
}

}
