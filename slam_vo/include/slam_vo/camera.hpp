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

#ifndef CAMERA_H
#define CAMERA_H

#include "slam_vo/common_include.hpp"
#include "config.hpp"

namespace slam_vo
{

class CameraParam
{
public:
  enum POSITION
  {
    FRONT_LEFT  = 0,
    FRONT_RIGHT = 1,
    DOWN_LEFT   = 2,
    DOWN_RIGHT  = 3
  };

  typedef std::shared_ptr<CameraParam> Ptr;
 float   fx_, fy_, cx_, cy_;
  CameraParam();
  CameraParam(uint8_t position);
  ~CameraParam();

public:
  static CameraParam::Ptr createCameraParam(uint8_t position);

  inline cv::Mat getIntrinsic() { return this->param_intrinsic_; }

  inline cv::Mat getDistortion() { return this->param_distortion_; }

  inline double getPrincipalX() { return this->param_intrinsic_.at<double>(0, 2); }

  inline double getPrincipalY() { return this->param_intrinsic_.at<double>(1, 2); }

  inline double getFocalX() { return this->param_intrinsic_.at<double>(0, 0); }

  inline double getFocalY() { return this->param_intrinsic_.at<double>(1, 1); }

protected:
  bool initIntrinsicParam();

  bool initDistortionParam();

protected:
  cv::Mat param_intrinsic_;
  cv::Mat param_distortion_;
  float   depth_scale_;
  uint8_t location_;
  
public:
  Vector3d world2camera( const Vector3d& p_w, const SE3& T_c_w );
  Vector3d camera2world( const Vector3d& p_c, const SE3& T_c_w );
  Vector2d camera2pixel( const Vector3d& p_c );
  Vector3d pixel2camera( const Vector2d& p_p, double depth=1 ); 
  Vector3d pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth=1 );
  Vector2d world2pixel ( const Vector3d& p_w, const SE3& T_c_w );

};

}
#endif // CAMERA_H
