#include "localization_wrapper.h"

#include <iomanip>

//#include <glog/logging.h>

#include "base_type.h"

LocalizationWrapper::LocalizationWrapper(ros::NodeHandle& nh) {
    // Load configs.
    double acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise;
    nh.param("acc_noise",       acc_noise, 1e-2);
    nh.param("gyro_noise",      gyro_noise, 1e-4);
    nh.param("acc_bias_noise",  acc_bias_noise, 1e-6);
    nh.param("gyro_bias_noise", gyro_bias_noise, 1e-8);

    double x, y, z;
    nh.param("I_p_Gps_x", x, 0.);
    nh.param("I_p_Gps_y", y, 0.);
    nh.param("I_p_Gps_z", z, 0.);
    const Eigen::Vector3d I_p_Gps(x, y, z);

    std::string log_folder = "/home";
    ros::param::get("log_folder", log_folder);

    // Log.
    file_state_.open(log_folder + "/state.csv");
    file_gps_.open(log_folder +"/gps.csv");

    // Initialization imu gps localizer.
    imu_gps_localizer_ptr_ = 
        std::make_unique<ImuGpsLocalization::ImuGpsLocalizer>(acc_noise, gyro_noise,
                                                              acc_bias_noise, gyro_bias_noise,
                                                              I_p_Gps);
    // Subscribe topics.
    gps_position_sub_ = nh.subscribe("dji_osdk_ros/gps_position", 10,  &LocalizationWrapper::GpsPositionCallback, this);

    imu_sub_ = nh.subscribe("dji_osdk_ros/imu", 10,  &LocalizationWrapper::ImuCallback, this);
    //gps_position_sub_ = nh.subscribe("/fix", 10,  &LocalizationWrapper::GpsPositionCallback, this);

    //imu_sub_ = nh.subscribe("/imu/data", 10,  &LocalizationWrapper::ImuCallback, this);
    
    state_pub_ = nh.advertise<geometry_msgs::PoseStamped>("gps_imu_position/inertial_pose",10);
    imugps_path_pub = nh.advertise<nav_msgs::Path>("gps_imu_position/imugpspath",10);
}

LocalizationWrapper::~LocalizationWrapper() {
    file_state_.close();
    file_gps_.close();
}

void LocalizationWrapper::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {

    ImuGpsLocalization::ImuDataPtr imu_data_ptr = std::make_shared<ImuGpsLocalization::ImuData>();
    imu_data_ptr->timestamp = imu_msg_ptr->header.stamp.toSec();
    imu_data_ptr->acc << imu_msg_ptr->linear_acceleration.x, 
                         imu_msg_ptr->linear_acceleration.y,
                         imu_msg_ptr->linear_acceleration.z;
    imu_data_ptr->gyro << imu_msg_ptr->angular_velocity.x,
                          imu_msg_ptr->angular_velocity.y,
                          imu_msg_ptr->angular_velocity.z;
    		  
    ImuGpsLocalization::State fused_state;
    const bool ok = imu_gps_localizer_ptr_->ProcessImuData(imu_data_ptr, &fused_state);
    if (!ok) {
        return;
    }
   // std::cout << fused_state.lla[0] << std::endl;
    // Publish fused state.
    ConvertStateToRosTopic(fused_state);
    state_pub_.publish(posefused);
    imugps_path_pub.publish(imugps_path);
    //std::cout << "here is imu fused" << std::endl; 
    std::cout << "imu:   " << posefused.pose.position.x << "\t" <<posefused.pose.position.y << "\t" << posefused.pose.position.z << std::endl;
    
    // Log fused state.
    LogState(fused_state);
}

void LocalizationWrapper::GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr) {
    // Check the gps_status.
    
  //  if (gps_msg_ptr->status.status != 2) {
 //       LOG(WARNING) << "[GpsCallBack]: Bad gps message!";
  //    std::cout << "bad gps" << std::endl;
  //      return;
 //   }
  std::cout << "gps" << gps_msg_ptr->latitude << "\t" << gps_msg_ptr->longitude << "\t" << gps_msg_ptr->altitude << std::endl;

    ImuGpsLocalization::GpsPositionDataPtr gps_data_ptr = std::make_shared<ImuGpsLocalization::GpsPositionData>();
    gps_data_ptr->timestamp = gps_msg_ptr->header.stamp.toSec();
    gps_data_ptr->lla << gps_msg_ptr->latitude,
                         gps_msg_ptr->longitude,
                         gps_msg_ptr->altitude;
    gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gps_msg_ptr->position_covariance.data());
    
    imu_gps_localizer_ptr_->ProcessGpsPositionData(gps_data_ptr);
    
   // std::cout << "gps" << gps_data_ptr->lla[0] << "\t" << gps_data_ptr->lla[1] << "\t" << gps_data_ptr->lla[2] << std::endl;

    LogGps(gps_data_ptr);
}

void LocalizationWrapper::LogState(const ImuGpsLocalization::State& state) {
    const Eigen::Quaterniond G_q_I(state.G_R_I);
    file_state_ << std::fixed << std::setprecision(15)
                << state.timestamp << ","
                << state.lla[0] << "," << state.lla[1] << "," << state.lla[2] << ","
                << state.G_p_I[0] << "," << state.G_p_I[1] << "," << state.G_p_I[2] << ","
                << state.G_v_I[0] << "," << state.G_v_I[1] << "," << state.G_v_I[2] << ","
                << G_q_I.x() << "," << G_q_I.y() << "," << G_q_I.z() << "," << G_q_I.w() << ","
                << state.acc_bias[0] << "," << state.acc_bias[1] << "," << state.acc_bias[2] << ","
                << state.gyro_bias[0] << "," << state.gyro_bias[1] << "," << state.gyro_bias[2] << "\n";
}

void LocalizationWrapper::LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data) {
    file_gps_ << std::fixed << std::setprecision(15)
              << gps_data->timestamp << ","
              << gps_data->lla[0] << "," << gps_data->lla[1] << "," << gps_data->lla[2] << "\n";
}

void LocalizationWrapper::ConvertStateToRosTopic(const ImuGpsLocalization::State& state) {  
  
 //   std::cout << state.lla[0] << std::endl;
  
    geometry_msgs::PoseStamped pose;
    //pose.header = posefused.header;//Qin
    pose.header.stamp = posefused.header.stamp;
    //pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "path";

    pose.pose.position.x = state.lla[0];
    pose.pose.position.y = state.lla[1];
    pose.pose.position.z = state.lla[2];
    
    // pose.pose.position.x = state.G_p_I[0];
    // pose.pose.position.y = state.G_p_I[1];
    // pose.pose.position.z = state.G_p_I[2];

    const Eigen::Quaterniond G_q_I(state.G_R_I);
    pose.pose.orientation.x = G_q_I.x();
    pose.pose.orientation.y = G_q_I.y();
    pose.pose.orientation.z = G_q_I.z();
    pose.pose.orientation.w = G_q_I.w();

    posefused.pose = pose.pose;
    
    //std::cout << "gps_position" << pose.pose.position.x << "\t" << pose.pose.position.y << "\t"<< pose.pose.position.z << std::endl; 

   imugps_path.header.frame_id = "path";
   imugps_path.header.stamp=ros::Time::now();

   geometry_msgs::PoseStamped pose1;

    //pose.pose.position.x = state.lla[0];
    //pose.pose.position.y = state.lla[1];
    //pose.pose.position.z = state.lla[2];
    
    pose1.pose.position.x = state.G_p_I[0];
    pose1.pose.position.y = state.G_p_I[1];
    pose1.pose.position.z = state.G_p_I[2];

    const Eigen::Quaterniond G_q_I2(state.G_R_I);
    pose1.pose.orientation.x = G_q_I2.x();
    pose1.pose.orientation.y = G_q_I2.y();
    pose1.pose.orientation.z = G_q_I2.z();
    pose1.pose.orientation.w = G_q_I2.w();

    imugps_path.poses.push_back(pose1);

    std::cout.precision(10);
    std::cout << "gps_position" << pose.pose.position.x << "\t" << pose.pose.position.y << "\t"<< pose.pose.position.z << std::endl; 
    
}
