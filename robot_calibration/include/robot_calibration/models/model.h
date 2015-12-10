/*
 * Copyright (C) 2015 Fetch Robotics Inc.
 * Copyright (C) 2013-2014 Unbounded Robotics Inc.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROBOT_CALIBRATION_MODELS_MODEL_H
#define ROBOT_CALIBRATION_MODELS_MODEL_H

#include <string>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <robot_calibration/calibration_offset_parser.h>

#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/JointState.h>
#include <robot_calibration_msgs/CalibrationData.h>

/** \brief Calibration code lives under this namespace */
namespace robot_calibration
{

class Model
{
public:
  Model(const std::string& name, KDL::Tree model, std::string root, std::string tip, bool inv);

  virtual ~Model() {}
 
  virtual std::vector<geometry_msgs::PointStamped> project(
    const robot_calibration_msgs::CalibrationData& data,
    const CalibrationOffsetParser& offsets)  = 0;  

  virtual std::vector<geometry_msgs::PointStamped> project_(
    const robot_calibration_msgs::CalibrationData& data,
    std::vector<geometry_msgs::PointStamped> arm_pts,
    const CalibrationOffsetParser& offsets) = 0;


  KDL::Frame getChainFK(const CalibrationOffsetParser& offsets,
                        const sensor_msgs::JointState& state);

  KDL::Chain chain_;

  const std::string &getName() const
  {
    return name_;
  }

  bool getInv() const
  {
    return inv_;
  }

protected:
//private:
  std::string root_;
  std::string tip_;
  std::string name_; 
  bool inv_;
};

/** \brief Converts our angle-axis-with-integrated-magnitude representation to a KDL::Rotation */
KDL::Rotation rotation_from_axis_magnitude(const double x, const double y, const double z);

/** \brief Converts from KDL::Rotation to angle-axis-with-integrated-magnitude */
void axis_magnitude_from_rotation(const KDL::Rotation& r, double& x, double& y, double& z);

}  // namespace robot_calibration

#endif  // ROBOT_CALIBRATION_MODELS_
