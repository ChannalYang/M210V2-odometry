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

#include "slam_vo/camera.hpp"

using namespace slam_vo;

CameraParam::CameraParam(){};

CameraParam::CameraParam(uint8_t location)
  : location_(location)
{
  if(!initIntrinsicParam())
  {
    std::cerr << "Failed to init intrinsic parameter";
  }

  if(!initDistortionParam())
  {
    std::cerr << "Failed to init distortion parameter";
  }
}

CameraParam::~CameraParam()
{

}

CameraParam::Ptr CameraParam::createCameraParam(uint8_t position)
{
  return std::make_shared<CameraParam>(position);
}

bool CameraParam::initDistortionParam()
{
  if(location_ == FRONT_LEFT)
  {
    param_distortion_ = Config::get<cv::Mat>("leftDistCoeffs");
  }
  else if(location_ == FRONT_RIGHT)
  {
    param_distortion_ = Config::get<cv::Mat>("rightDistCoeffs");
  }
  else
  {
    std::cerr << "Please specify the location of the camera\n";
    return false;
  }
  return true;
}

bool CameraParam::initIntrinsicParam()
{
  if(location_ == FRONT_LEFT)
  {
    param_intrinsic_ = Config::get<cv::Mat>("leftCameraIntrinsicMatrix");
		cx_=getPrincipalX();
		cy_=getPrincipalY();
		fx_=getFocalX();
		fy_=getFocalY();

  }
  else if(location_ == FRONT_RIGHT)
  {
    param_intrinsic_ = Config::get<cv::Mat>("rightCameraIntrinsicMatrix");
  }
  else
  {
    std::cerr << "Please specify the location of the camera\n";
    return false;
  }
  return true;
}

Vector3d CameraParam::world2camera(const Vector3d& p_w, const SE3& T_c_w)
{
    return T_c_w*p_w;
}

Vector3d CameraParam::camera2world ( const Vector3d& p_c, const SE3& T_c_w )
{
    return T_c_w.inverse() *p_c;
}

Vector2d CameraParam::camera2pixel ( const Vector3d& p_c )
{
    return Vector2d (
               fx_ * p_c ( 0,0 ) / p_c ( 2,0 ) + cx_,
               fy_ * p_c ( 1,0 ) / p_c ( 2,0 ) + cy_
           );
}

Vector3d CameraParam::pixel2camera ( const Vector2d& p_p, double depth )
{
    return Vector3d (
               ( p_p ( 0,0 )-cx_) *depth/fx_,
               ( p_p ( 1,0 )-cy_) *depth/fy_,
               depth
           );
}

Vector2d CameraParam::world2pixel ( const Vector3d& p_w, const SE3& T_c_w )
{
    return camera2pixel ( world2camera(p_w, T_c_w) );
}

Vector3d CameraParam::pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth )
{
    return camera2world ( pixel2camera ( p_p, depth ), T_c_w );
}





