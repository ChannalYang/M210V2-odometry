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

#include "slam_vo/frame.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include<iostream>
#include <stdio.h>
using namespace cv;

namespace slam_vo
{
Frame::Frame()
: id_(-1), time_stamp_(-1), camera_(nullptr), is_key_frame_(false)
{

}

Frame::Frame ( long id, double time_stamp, SE3 T_c_w, CameraParam::Ptr camera, Mat left, Mat right )
: id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), left_(left), right_(right), is_key_frame_(false)
{

}

Frame::~Frame()
{

}

Frame::Ptr Frame::createFrame()
{
    static long factory_id = 0;
    return Frame::Ptr( new Frame(factory_id++) );
}

void  Frame::createdepth(){


        
    cv::Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);
    int sgbmWinSize = 7, numberOfDisparities =64;
    sgbm->setPreFilterCap(63);
    sgbm->setBlockSize(sgbmWinSize);
    sgbm->setP1(8*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(3);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    
    sgbm->setMode(StereoSGBM::MODE_SGBM);
    Mat disp_;
    
    sgbm->compute(left_,right_, disp_);
    
    depth_.create(disp_.rows,disp_.cols,CV_32FC1);
    for (int h=0;h<disp_.rows;++h)
    {
        for (int w=0;w<disp_.cols;++w)
        {
            float disparity = (float)disp_.at<short int>(h, w)*0.0625-0.5;

            if(disparity >= 6 && disparity <= 64)
            {
             depth_.at<float>(h,w) = (float)52.67/disparity;
            }
            else
            {
               depth_.at<float>(h,w) = -1.0;
            }
        }
    }

}


double Frame::findDepth ( const cv::KeyPoint& kp )
{

    int x = cvRound(kp.pt.x);
    int y = cvRound(kp.pt.y);
   // double d1=depth_.ptr<short int>(y)[x];
    //double d =(double)111*0.45/(double)(depth_.ptr<short int>(y)[x]*0.0625);
    //std::cout<<"d1="<<d1<<" ";
    //std::cout<<"d="<<d<<endl;
    float d = depth_.ptr<float>(y)[x];
    if ( d!=0 )
    {
        return double(d);///camera_->depth_scale_;
    }
    else 
    {
        // check the nearby points 
        int dx[4] = {-1,0,1,0};
        int dy[4] = {0,-1,0,1};
        for ( int i=0; i<4; i++ )
        {
            d = depth_.ptr<float>(y+dy[i])[x+dx[i]];
            if ( d!=0 )
            {
                return double(d);///camera_->depth_scale_;
            }
        }
    }
    if(d==0)
    return -1.0;
}

void Frame::setPose ( const SE3& T_c_w )
{
    T_c_w_ = T_c_w;
}


Vector3d Frame::getCamCenter() const
{
    return T_c_w_.inverse().translation();
}

bool Frame::isInFrame ( const Vector3d& pt_world )
{
    Vector3d p_cam = camera_->world2camera( pt_world, T_c_w_ );
    // cout<<"P_cam = "<<p_cam.transpose()<<endl;
    if ( p_cam(2,0)<0 ) return false;
    Vector2d pixel = camera_->world2pixel( pt_world, T_c_w_ );
    // cout<<"P_pixel = "<<pixel.transpose()<<endl<<endl;
    return pixel(0,0)>0 && pixel(1,0)>0 
        && pixel(0,0)<left_.cols 
        && pixel(1,0)<left_.rows;
}

}
