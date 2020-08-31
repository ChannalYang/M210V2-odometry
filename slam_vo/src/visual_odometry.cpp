/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>

#include "slam_vo/config.hpp"
#include "slam_vo/visual_odometry.hpp"
#include "slam_vo/g2o_types.hpp"

namespace slam_vo
{

VisualOdometry::VisualOdometry() :
    state_ ( INITIALIZING ), flag( YES ) ,ref_ ( nullptr ), curr_ ( nullptr ), map_ ( new Map ), num_lost_ ( 0 ), num_inliers_ ( 0 ), matcher_flann_( new cv::flann::LshIndexParams(5,10,2) )
{
    num_of_features_    =600;// Config::get<int> ( "number_of_features" );
    scale_factor_       =1.2;//Config::get<double> ( "scale_factor" );
    level_pyramid_      = 6;//Config::get<int> ( "level_pyramid" );
    match_ratio_        = 1.7;//Config::get<float> ( "match_ratio" );
    max_num_lost_       = 5;//Config::get<float> ( "max_num_lost" );
    min_inliers_        = 10;//Config::get<int> ( "min_inliers" );
    key_frame_min_rot   = 0.1;//Config::get<double> ( "keyframe_rotation" );
    key_frame_min_trans = 0.1;//Config::get<double> ( "keyframe_translation" );
    map_point_erase_ratio_ = 0.5;//Config::get<double> ( "map_point_erase_ratio" );  
    orb_ = cv::ORB::create ( num_of_features_, scale_factor_, level_pyramid_ );
    set_lost_=0;
}

VisualOdometry::~VisualOdometry()
{

}
bool VisualOdometry::addFrame ( Frame::Ptr frame )
{

    switch ( state_ )
    {
    case INITIALIZING:
    {

        state_ = OK;
        curr_ = ref_ = frame;	
        extractKeyPoints();
        computeDescriptors();
        k=setRef3DPoints();
 	if(k == true) flag = NO;
	else state_=INITIALIZING;
        break;
    }
    case OK:
    {
	curr_ = frame;
	cout<<"k=="<<k<<endl;
	if(k == true){
	    extractKeyPoints();
	    computeDescriptors();
	    featureMatching();
	    poseEstimationPnP();

	    if ( checkEstimatedPose() == true ) // a good estimation
	    {
		T_c_r_estimated_pre=T_c_r_estimated_;
		curr_->T_c_w_ = T_c_r_estimated_ * ref_->T_c_w_;  // T_c_w = T_c_r*T_r_w 
		cout<<"Tcw"<<curr_->T_c_w_.matrix().inverse()<<endl;
		ref_ = curr_;
		k=setRef3DPoints();
		num_lost_ = 0;
	    }
	    else // bad estimation due to various reasons , ignore it;
	    {
		num_lost_++;
		if ( num_lost_ > max_num_lost_ )
		{
		    state_ = INITIALIZING;
		    flag = YES;
		    num_lost_ = 0 ;
		}
	    }
	    set_lost_=0;
            return false;
	} 
	else{
	    extractKeyPoints();
	    computeDescriptors();
	    curr_->T_c_w_=T_c_r_estimated_pre*ref_->T_c_w_;
  	    ref_ = curr_;
	    k=setRef3DPoints();
	    set_lost_++;
	    if ( set_lost_ > 2 )
            {
                state_ = INITIALIZING;
		flag= YES ;
		set_lost_=0;
            }
	}
        
        break;
    }
    }

    return true;
}

void VisualOdometry::extractKeyPoints()
{
	//boost::timer timer;
   // cv::Mat outimg1;
    orb_->detect ( curr_->left_, keypoints_curr_ );
  //  cv::drawKeypoints(curr_->color_,keypoints_curr_,outimg1,cv::Scalar::all(-1),cv::DrawMatchesFlags::DEFAULT);
   // imshow("ORB",outimg1);
  //  cout<<"have keypoints:"<<keypoints_curr_.size()<<" "<<endl;
  //  cout<<"keypoints cost time "<<timer.elapsed()<<endl;
}

void VisualOdometry::computeDescriptors()
{
		//boost::timer timer;
    orb_->compute ( curr_->left_, keypoints_curr_, descriptors_curr_ );
	//	cout<<"descriptors.size="<<descriptors_curr_.size()<<endl;
   // cout<<"descriptors cost time "<<timer.elapsed()<<endl;
}

void VisualOdometry::featureMatching()
{
		//boost::timer timer;
    
    vector<cv::DMatch> matches;
    matcher_flann_.match( descriptors_ref_, descriptors_curr_, matches );
    // select the best matches
    float min_dis = std::min_element (
                        matches.begin(), matches.end(),
                        [] ( const cv::DMatch& m1, const cv::DMatch& m2 )
    {
        return m1.distance < m2.distance;
    } )->distance;

    feature_matches_.clear();

    double min_dist=10000, max_dist=0;

    for ( int i = 0; i < descriptors_ref_.rows; i++ )
    {
        double dist = matches[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }
    
     for ( int i = 0; i < descriptors_ref_.rows; i++ )
    {
	
        if ( matches[i].distance <= max ( match_ratio_*min_dist, 30.0 ) )
        {
            feature_matches_.push_back ( matches[i] );
	   
        }
    }
    
  //   Mat img_match;
    //   cv::drawMatches(ref_->left_,keypoints_ref_,curr_->left_,keypoints_curr_,feature_matches_,img_match);
     //  imshow("good match",img_match);
   //  cv::waitKey(0);
   // cout<<"good matches: "<<feature_matches_.size()<<endl;
	//cout<<"match cost time "<<timer.elapsed()<<endl;
}

bool VisualOdometry::setRef3DPoints()
{
    // select the features with depth measurements 
    pts_3d_ref_.clear();
    keypoints_ref_.clear();
    descriptors_ref_ = Mat();
	//	boost::timer timer;
    ref_->createdepth();
	//	cout<<"creat depth cost time "<<timer.elapsed()<<endl;
		boost::timer timer1;
    int count=0;
    for ( size_t i=0; i<keypoints_curr_.size(); i++ )
    {
        double d = ref_->findDepth(keypoints_curr_[i]); 
				//cout<<"d= "<<d<<"  ";
        if ( d > 0)
        {
            Vector3d p_cam = ref_->camera_->pixel2camera(
                Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), d
            );
            pts_3d_ref_.push_back( cv::Point3f( p_cam(0,0), p_cam(1,0), p_cam(2,0) ));
	    keypoints_ref_.push_back(keypoints_curr_[i]);
            descriptors_ref_.push_back(descriptors_curr_.row(i));
	    count++;
        }	    
     }
	//	cout<<"set 3D points cost time "<<timer1.elapsed()<<endl;
     // 3D point is not enough
	//cout<<"count=="<<count<<endl;
     if (count<60){
	return false;
    }
    else
	return true;

}

void VisualOdometry::poseEstimationPnP()
{
	//e	boost::timer timer;
    // construct the 3d 2d observations
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;
    
    for ( cv::DMatch m:feature_matches_ )
    {
        pts3d.push_back( pts_3d_ref_[m.queryIdx] );
        pts2d.push_back( keypoints_curr_[m.trainIdx].pt );
    }
    
    Mat K = ( cv::Mat_<double>(3,3)<<
        ref_->camera_->fx_, 0, ref_->camera_->cx_,
        0, ref_->camera_->fy_, ref_->camera_->cy_,
        0,0,1
    );
		//cout<<"fx="<<ref_->camera_->fx_;
	//	cout<<"fy="<<ref_->camera_->fy_;
	//	cout<<"cx="<<ref_->camera_->cx_;
	//	cout<<"cy="<<ref_->camera_->cy_;
    Mat rvec, tvec, inliers;
    cv::solvePnPRansac( pts3d, pts2d, K, Mat(), rvec, tvec, true, 100, 8.0, 0.99, inliers );
    num_inliers_ = inliers.rows;
   // cout<<"pnp inliers: "<<num_inliers_<<endl;
    T_c_r_estimated_ = SE3(
        SO3(rvec.at<double>(0,0), rvec.at<double>(1,0), rvec.at<double>(2,0)), 
        Vector3d( tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0))
    );
		//cout<<"tcr="<<T_c_r_estimated_.matrix()<<endl;
    
    
    // using bundle adjustment to optimize the pose 
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block* solver_ptr = new Block( std::unique_ptr<Block::LinearSolverType>(linearSolver) );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg (std::unique_ptr<Block>(solver_ptr) );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );
    
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat (
        T_c_r_estimated_.rotation_matrix(), 
        T_c_r_estimated_.translation()
    ) );
    optimizer.addVertex ( pose );

    // edges
    for ( int i=0; i<inliers.rows; i++ )
    {
        int index = inliers.at<int>(i,0);
        // 3D -> 2D projection
        EdgeProjectXYZ2UVPoseOnly* edge = new EdgeProjectXYZ2UVPoseOnly();
        edge->setId(i);
        edge->setVertex(0, pose);
        edge->camera_ = curr_->camera_.get();
        edge->point_ = Vector3d( pts3d[index].x, pts3d[index].y, pts3d[index].z );
        edge->setMeasurement( Vector2d(pts2d[index].x, pts2d[index].y) );
        edge->setInformation( Eigen::Matrix2d::Identity() );
        optimizer.addEdge( edge );
    }
    
    optimizer.initializeOptimization();
    optimizer.optimize(10);
    
    T_c_r_estimated_ = SE3 (
        pose->estimate().rotation(),
        pose->estimate().translation()
    );
	//	cout<<"PNP cost time:"<<timer.elapsed()<<endl;
}

bool VisualOdometry::checkEstimatedPose()
{
    // check if the estimated pose is good
    if ( num_inliers_ < min_inliers_ )
    {
     //   cout<<"reject because inlier is too small: "<<num_inliers_<<endl;
        return false;
    }
    // if the motion is too large, it is probably wrong
    /*Sophus::Vector6d d = T_c_r_estimated_.log();
    if ( d.norm() > 10.0 )
    {
     //   cout<<"reject because motion is too large: "<<d.norm()<<endl;
        return false;
    }*/
    return true;
}

}
