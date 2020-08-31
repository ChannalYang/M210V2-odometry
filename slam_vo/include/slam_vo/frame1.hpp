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

#ifndef FRAME_H
#define FRAME_H

#include "slam_vo/common_include.hpp"
#include "slam_vo/camera.hpp"
#include <opencv2/features2d/features2d.hpp>

using namespace slam_vo;

namespace M210_STEREO 
{
  
  static const int VGA_HEIGHT = 480;
  static const int VGA_WIDTH  = 640;
  
class MapPoint;

class Frame{

public:
  typedef std::shared_ptr<Frame> Ptr;
  uint64_t  id;
  uint32_t  time_stamp;
  cv::Mat   color_image;
  cv::Mat   depth_image;
  bool      is_key_frame_;  // whether a key-frame
  SE3       T_c_w_;      // transform from world to camera
  CameraParam::Ptr  camera_; 
  
public:
  Frame();
  Frame(uint64_t id, uint32_t time_stamp, cv::Mat cimg, cv::Mat dimg, bool key);
  ~Frame();
    
  static Frame::Ptr createFrame(uint64_t id, uint32_t time_stamp, cv::Mat color_img, cv::Mat depth_img)
  {
    return std::make_shared<Frame>(id, time_stamp, color_img, depth_img);
  }
  
  
  double findDepth( const cv::KeyPoint & kp );
    
    // Get Camera Center
  Vector3d getCamCenter() const;
    
  void setPose( const SE3& T_c_w );
    
    // check if a point is in this frame 
  bool isInFrame( const Vector3d& pt_world );
  
};

}
#endif // FRAME_H
