#ifndef ESKF_NODE_HPP_
#define ESKF_NODE_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
//#include <pose_ekf_fusion/OpticalFlowRad.h>
//#include <mavros_msgs/ExtendedState.h>
#include <message_filters/subscriber.h>
#include <pose_ekf_fusion/ESKF.hpp>

namespace eskf {

  class ESKFNode {
  public:
    static constexpr int default_publish_rate_ = 100;
    static constexpr int default_fusion_mask_ = MASK_EV;
    int prevStampOpticalFlowPose_;
  //  int prevStampOpticalFlowPose_;

    ESKFNode( ros::NodeHandle& nh);
//    ~ESKFNode();

  private:
  //  ros::NodeHandle nh_;

    // publishers
    ros::Publisher pubPose_;

    //  subsribers
    ros::Subscriber subImu_;
    ros::Subscriber subVisionPose_;
    ros::Subscriber subGpsPose_;
 //   ros::Subscriber subOpticalFlowPose_;
    ros::Subscriber subMagPose_;
    ros::Subscriber subExtendedState_;
    ros::Subscriber subRangeFinderPose_;

    // implementation
    eskf::ESKF eskf_;
    ros::Time prevStampImu_;
    ros::Time prevStampVisionPose_;
    ros::Time prevStampGpsPose_;
 //   ros::Time prevStampOpticalFlowPose_;
    ros::Time prevStampMagPose_;
    ros::Time prevStampRangeFinderPose_;
    ros::Timer pubTimer_;
    bool init_;

    //  callbacks
    void inputCallback(const sensor_msgs::ImuConstPtr&);
    void visionCallback(const geometry_msgs::PoseStampedConstPtr&);
    void gpsCallback(const geometry_msgs::PoseStampedConstPtr&);
 //   void opticalFlowCallback(const mavros_msgs::OpticalFlowRadConstPtr&);
//    void magCallback(const sensor_msgs::MagneticFieldConstPtr&);
 //   void extendedStateCallback(const mavros_msgs::ExtendedStateConstPtr&);
    void rangeFinderCallback(const sensor_msgs::RangeConstPtr&);
    void publishState(const ros::TimerEvent&);
  };
} //  namespace eskf

#endif // ESKF_NODE_HPP_
