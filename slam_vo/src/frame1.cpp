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

using namespace slam_vo;

namespace M210_STEREO
{
Frame::Frame()
    :id(-1)
    , time_stamp(-1)
    , color_image(nullptr)
    , depth_image(nullptr)
    , is_key_frame_(false)
{

}


Frame::Frame(uint64_t id, uint32_t time_stamp, cv::Mat cimg, cv::Mat dimg, bool key)
    : id(id)
    , time_stamp(time_stamp)
    , color_image(cimg)
    , depth_image(dimg)
    , is_key_frame_(key)
{
  
};


Frame::~Frame()
{

}

double Frame::findDepth ( const cv::KeyPoint& kp )
{
    int x = cvRound(kp.pt.x);
    int y = cvRound(kp.pt.y);
    ushort d = depth_image.ptr<ushort>(y)[x];
    if ( d!=0 )
    {
        return double(d)/camera_->depth_scale_;
    }
    else 
    {
        // check the nearby points 
        int dx[4] = {-1,0,1,0};
        int dy[4] = {0,-1,0,1};
        for ( int i=0; i<4; i++ )
        {
            d = depth_image.ptr<ushort>( y+dy[i] )[x+dx[i]];
            if ( d!=0 )
            {
                return double(d)/camera_->depth_scale_;
            }
        }
    }
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
        && pixel(0,0)<color_image.cols 
        && pixel(1,0)<color_image.rows;
}

}
