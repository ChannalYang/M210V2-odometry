#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include "lidtocam.h"
#include <dji_osdk_ros/SetupCameraStream.h>

using namespace sensor_msgs;
//全局变量
ros::Publisher image_lk_pub;
//回调函数
void callback_lk(const sensor_msgs::ImageConstPtr& msgRGB)
{   //显示图像ID
	cout<<msgRGB->header.frame_id<<endl;
	//将ros图像格式转换为cv：：Mat格式
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    image = cv_ptrRGB->image;
    cvtColor(image,image_gray,CV_BGR2GRAY);//灰度图
    vector<Point2f> corners;
    cout << "currentFrame:"<< currentFrame << endl;
    vector<Point2f> keypointsCopy;
    loadMatrix();
    //光流跟踪
	if(currentFrame==0)
    {
        goodFeaturesToTrack(image_gray,corners,maxConerNumber,qualityLevel,minDistance);
        currentFrame++;
        for(auto kp:corners)
        {
            keypoints.push_back(kp);
            keypointsCopy.push_back(kp);							
        }
        image_gray_last = image_gray.clone();;
        return;
    }
    opticalTrack();
    keypointsCopy.clear();
    for(auto kp:keypoints)
    {
        keypointsCopy.push_back(kp);
    }
    if(keypoints.size()<50)        /////////////////////注意这里先不写，先看后面，别忘了/////////////////////////
    {  
        //当跟踪点<50，加入新的特征点,新特征点不能和原特征点太近，dis>9
        addPoint =true;
        //pnp求R,t
        Mat r,t;
		solvePnP(pnp3dPoints, keypoints, K_Mat, Mat(), r, t, false, SOLVEPNP_EPNP); 
        Mat R;
        Rodrigues(r, R);                          
        vector<Point2d> tmp_key;
        for(auto kp:keypoints)
        {
            tmp_key.push_back(kp);
        }
        bundleAdjustment(pnp3dPoints,tmp_key,K_Mat,R,t);
        Eigen::Matrix3d eigenR;
        Eigen::Vector3d eigent;
        cv2eigen(R,eigenR);
        cv2eigen(t,eigent);
        PoseR.push_back(eigenR);
        Poset.push_back(eigent);
        //加入新的特征点
        corners.clear();
        goodFeaturesToTrack(image_gray,corners,maxConerNumber,qualityLevel,minDistance);
        rejectWithF(keypoints,keypointsLast);
        for(int i=0;i<corners.size();i++)
        {
            float x1 = corners[i].x;
            float y1 = corners[i].y;
            vector<float> dis;
            for(auto kp:keypoints)
            {
                float x2 = kp.x;
                float y2 = kp.y;
                dis.push_back((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
            }
            sort(dis.begin(),dis.end());
            if(dis[0]>225){
                keypointsCopy.push_back(corners[i]);
            }			
        }
        keypoints.clear();
        for(auto kp:keypointsCopy)
        {
            keypoints.push_back(kp);
        }
        //对加入后的所有特征点进行光流跟踪
        opticalTrack();
        //三角化新加入后所有特征点，更新pnp3dPoints
        pnp3dPoints.clear();
        vector<Point2f> cam_last = pixelToCam(keypointsLast);
        vector<Point2f> cam_cur = pixelToCam(keypoints);
        vector<Point3d> points;
        triangulation(R, t, points, cam_last, cam_cur); 
        for(size_t i = 0; i < points.size();i++)
        {
            Mat keypoints_3d = R*(Mat_<double>(3,1)<<points[i].x, points[i].y, points[i].z)+t;
            Point3d tmp_p;
            tmp_p.x = keypoints_3d.at<double>(0,0);
            tmp_p.y = keypoints_3d.at<double>(1,0);
            tmp_p.z = keypoints_3d.at<double>(2,0);
            pnp3dPoints.push_back(tmp_p);
            //验证pnp R,t误差
            keypoints_3d /= keypoints_3d.at<double>(2,0);
        }
        keypointsCopy.clear();
        for(auto kp:keypoints)
        {
            keypointsCopy.push_back(kp);
        }
        /////////////////////////////////在这里添加///////////////////////////////////
    }
    keypointsCopy.clear();
    for(auto kp:keypoints)
    {
        keypointsCopy.push_back(kp);
    }
    if(!initial_Flag)
    {								
        initialRT(keypointsLast,keypoints);  //对极几何+三角测量								
    }
	else if(!addPoint)
    {   //pnp
        solvedPnP(pnp3dPoints,keypointsLast,keypoints);
    }
    addPoint = false;
	visualization();   
    //把第一帧作为参考系,输出相对位姿。
	Eigen::Matrix4d poseck1_ck;
    poseck1_ck << PoseR[num](0, 0), PoseR[num](0, 1), PoseR[num](0, 2), Poset[num][0],
				  PoseR[num](1, 0), PoseR[num](1, 1), PoseR[num](1, 2), Poset[num][1],
				  PoseR[num](2, 0), PoseR[num](2, 1), PoseR[num](2, 2), Poset[num][2],
				                 0,	               0,	             0,	            1;
    if(currentFrame == 1)
    {
        PoseAll.push_back(poseck1_ck.inverse());
    }
    else if(currentFrame>=2)
    {
        PoseAll.push_back(PoseAll[num-1]*poseck1_ck.inverse());
    }
    Eigen::Matrix <double,3,3> matx3d_R;
    Eigen::Matrix <double,3,1> matx_T;
    matx_T   << PoseAll[num](0,3),PoseAll[num](1,3),PoseAll[num](2,3);
    matx3d_R << PoseAll[num](0,0),PoseAll[num](0,1),PoseAll[num](0,2),
                PoseAll[num](1,0),PoseAll[num](1,1),PoseAll[num](1,2),
                PoseAll[num](2,0),PoseAll[num](2,1),PoseAll[num](2,2);
                
    Eigen::Quaterniond ortn=Eigen::Quaterniond(matx3d_R);
    //输出位姿信息
    geometry_msgs::PoseStamped pose1;
    pose1.header=msgRGB->header;
    pose1.pose.position.x = PoseAll[num](0,3);
    pose1.pose.position.y = PoseAll[num](1,3);
    pose1.pose.position.z = PoseAll[num](2,3);

    pose1.pose.orientation.x = ortn.x();
    pose1.pose.orientation.y = ortn.y();
    pose1.pose.orientation.z = ortn.z();
    pose1.pose.orientation.w = ortn.w();

    //发布话题  
    image_lk_pub.publish(pose1);
    num++;
    image_gray_last = image_gray.clone();
    currentFrame++;
}
int main(int argc,char** argv)
{
    ros::init(argc, argv, "lkvo_node");
    ros::NodeHandle nh;
    ros::ServiceClient setup_camera_stream_client = nh.serviceClient<dji_osdk_ros::SetupCameraStream>("setup_camera_stream");
    ros::Subscriber image_lk_sub=nh.subscribe("dji_osdk_ros/fpv_camera_images",1000,callback_lk);
    image_lk_pub =nh.advertise<geometry_msgs::PoseStamped>("/lk_pose",10);
    dji_osdk_ros::SetupCameraStream setupCameraStream_;
    setupCameraStream_.request.cameraType = setupCameraStream_.request.FPV_CAM;
    setupCameraStream_.request.start = 1;
    setup_camera_stream_client.call(setupCameraStream_);
    ros::spin();
    return 0;
}
