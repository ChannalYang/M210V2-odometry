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

#ifndef CONFIG_H
#define CONFIG_H

#include "slam_vo/common_include.hpp" 

namespace slam_vo 
{
class Config
{
private:
    
    static Config* single_instance_;
    
    cv::FileStorage file_;
    
    Config () ; // private constructor makes a singleton
public:
    ~Config();  // close the file when deconstructing 
    
    static Config& instance();

    static Config* instancePtr();

    static void setParamFile(const std::string& file_name);
    
    // access the parameter values
    template <typename T>
    static T get(const std::string& key)
  {
    T t;
    Config::instancePtr()->file_[key] >> t;
    return t;
  }
};
}

#endif // CONFIG_H




