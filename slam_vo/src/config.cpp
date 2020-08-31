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

#include "slam_vo/config.hpp"
namespace slam_vo{

Config* Config::single_instance_ = NULL;

Config::Config()
{
}

Config::~Config()
{
  if (file_.isOpened())
  {
    file_.release();
  }
}

Config&
Config::instance()
{
  return *Config::single_instance_;
}

Config*
Config::instancePtr()
{
  return Config::single_instance_;
}

void
Config::setParamFile(const std::string& file_name)
{
  if(!Config::single_instance_)
  {
    Config::single_instance_ = new Config();
  }

  Config::instancePtr()->file_ = cv::FileStorage( file_name, cv::FileStorage::READ );
cout<<"succefull open files";

  if(!Config::instancePtr()->file_.isOpened())
  {
    std::cerr << "Failed to open " << file_name << " file\n";
    Config::instancePtr()->file_.release();
  }
}
}


