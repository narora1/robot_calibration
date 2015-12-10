/*
 * Copyright (C) 2014-2015 Fetch Robotics Inc.
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

// Author: Michael Ferguson

#include <robot_calibration/ceres/optimizer.h>

#include <ceres/ceres.h>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_calibration_msgs/CalibrationData.h>

#include <robot_calibration/calibration_offset_parser.h>
#include <robot_calibration/ceres/camera3d_to_arm_error.h>
#include <robot_calibration/ceres/ground_plane_error.h>
#include <robot_calibration/ceres/gripper_depth_error.h>
#include <robot_calibration/ceres/gripper_color_error.h>
#include <robot_calibration/ceres/data_functions.h>
#include <robot_calibration/ceres/outrageous_error.h>
#include <robot_calibration/models/camera3d.h>
#include <robot_calibration/models/camera2d.h>
#include <robot_calibration/models/chain.h>
#include <robot_calibration/models/model.h>
#include <robot_calibration/models/multichain.h>
#include <boost/shared_ptr.hpp>
#include <string>
#include <map>

namespace robot_calibration
{

Optimizer::Optimizer(const std::string& robot_description)
{
  if (!model_.initString(robot_description))
    std::cerr << "Failed to parse URDF." << std::endl;
}

Optimizer::~Optimizer()
{
}

// Determine if a sample of data has an observation from
// the desired sensor
bool hasSensor(
  const robot_calibration_msgs::CalibrationData& msg,
  const std::string& sensor)
{
  for (size_t i = 0; i < msg.observations.size(); i++)
  {
    if (msg.observations[i].sensor_name == sensor)
      return true;
  }
  return false;
}

int Optimizer::optimize(OptimizationParams& params,
                        std::vector<robot_calibration_msgs::CalibrationData> data,
                        bool progress_to_stdout)
{
  // Load KDL from URDF
  if (!kdl_parser::treeFromUrdfModel(model_, tree_))
  {
    std::cerr << "Failed to construct KDL tree" << std::endl;
    return -1;
  }

  // Create models
  for (size_t i = 0; i < params.models.size(); ++i)
  {
    if (params.models[i].type == "chain")
    {
      ROS_INFO_STREAM("Creating chain '" << params.models[i].name << "' from " <<
                                            params.base_link << " to " <<
                                            params.models[i].params["frame"]);
      ChainModel* model = new ChainModel(params.models[i].name, tree_, params.base_link, params.models[i].params["frame"], 0);
      models_[params.models[i].name] = model;
    }
    else if (params.models[i].type == "camera3d")
    {
      ROS_INFO_STREAM("Creating camera3d '" << params.models[i].name << "' in frame " <<
                                               params.models[i].params["frame"]);
      Camera3dModel* model = new Camera3dModel(params.models[i].name, tree_, params.base_link, params.models[i].params["frame"], 0);
      models_[params.models[i].name] = model;
    }
    else if (params.models[i].type == "camera2d")
    {
      ROS_INFO_STREAM("Creating camera2d '" << params.models[i].name << "' in frame " <<
          params.models[i].params["frame"]);
      Camera2dModel* model = new Camera2dModel(params.models[i].name, tree_, params.base_link, params.models[i].params["frame"], 0);
      models_[params.models[i].name] = model;
    }
    else if (params.models[i].type == "multichain")
    {
       ROS_INFO_STREAM("Creating multichain '" << params.models[i].name );
       //MultiChainModel * model = 
       //std::cout << params.models[i].params["chains"].size() << std::endl;
       //std::cout << params.models[i].params["chains"].["name"] << std::endl;
       XmlRpc::XmlRpcValue chains_ = params.models[i].params["chains"];
       //std::string name_ = cast<std::string>(chains_[j]["name"]) ;
       //std::string frame_ = static_cast<std::string>(chains_[j]["frame"]);
       //                bool inv_ = chains_[j]["inv"];

       MultiChainModel* models = ( new MultiChainModel(params.models[i].name, tree_, params.base_link, params.models[i].params["frame"] , 0));
       for(int j = 0; j < chains_.size(); j++)
       {
         //MultiChainModel* model = new MultiChainModel(params.models[i].name, tree_, params.base_link, params.models[i].params["frame"]);
//         //std::cout << static_cast<std::string>(chains_[j]["name"]) << std::endl;
         std::string name_ = static_cast<std::string>(chains_[j]["name"]) ;
         std::string frame_ = static_cast<std::string>(chains_[j]["frame"]);
         bool inv_ = chains_[j]["inv"];
         std::cout << "MULTICHAIN " << name_ << "  " << inv_ << std::endl;
         //std::cout << "model chian size" << models->chain.size() << std::endl;
         //std::cout << inv_ << std::endl;
         MultiChainModel* model =( new MultiChainModel(name_, tree_, params.base_link, frame_, inv_));

         std::cout << "   model " << model->getName() << "  " << int(model->getInv()) << " : " << model->getInv() << std::endl;
         //std::cout << "..." << std::endl;
         models->chain.push_back(model);
         //std::cout << "....." << std::endl;
         //std::cout << "model chian size" << models->chain.size() << std::endl;
       }
        models_[params.models[i].name] = models;
    }   
    else
    {
      // ERROR unknown
    }
  }

  // Setup  parameters to calibrate
  offsets_.reset(new CalibrationOffsetParser());
  for (size_t i = 0; i < params.free_params.size(); ++i)
  {
    offsets_->add(params.free_params[i]);
  }
  for (size_t i = 0; i < params.free_frames.size(); ++i)
  { //std::cout << params.free_frames.size() << std::endl;
    offsets_->addFrame(params.free_frames[i].name,
                       params.free_frames[i].x,
                       params.free_frames[i].y,
                       params.free_frames[i].z,
                       params.free_frames[i].roll,
                       params.free_frames[i].pitch,
                       params.free_frames[i].yaw);
  }

  // Allocate space
  double* free_params = new double[offsets_->size()];
  for (int i = 0; i < offsets_->size(); ++i)
    free_params[i] = 0.0;

  double z_ = 0;
  // Houston, we have a problem...
  ceres::Problem* problem = new ceres::Problem();

  // For each sample of data:
  for (size_t i = 0; i < data.size(); ++i)
  {
    for (size_t j = 0; j < params.error_blocks.size(); ++j)
    {
      if (params.error_blocks[j].type == "camera3d_to_arm")
      {
        // This error block can process data generated by the LedFinder,
        // CheckboardFinder, or any other finder that can sample the pose
        // of one or more data points that are connected at a constant offset
        // from a link a kinematic chain (the "arm").


        std::string camera_name = static_cast<std::string>(params.error_blocks[j].params["camera"]);
        std::string arm_name = static_cast<std::string>(params.error_blocks[j].params["arm"]);

        // Check that this sample has the required features/observations
        if (!hasSensor(data[i], camera_name) || !hasSensor(data[i], arm_name))
          continue;

        // Create the block
        ceres::CostFunction * cost = Camera3dToArmError::Create(
            dynamic_cast<Camera3dModel*>(models_[camera_name]),
            dynamic_cast<ChainModel*>(models_[arm_name]),
            offsets_.get(), data[i]);

        int index = -1;
        for (size_t k =0; k < data[i].observations.size() ; k++)
        {
           if (data[i].observations[k].sensor_name == camera_name)
          {
            index = k;
            break;
          }
        }

        if(index == -1)
        {
          std::cerr << "Sensor name doesn't exist" << std::endl;
          return 0;
        }
        if (progress_to_stdout)
        {
          double ** params = new double*[1];
          params[0] = free_params;
          double * residuals = new double[data[i].observations[index].features.size() * 3];  // TODO: should check that all features are same length?

          cost->Evaluate(params, residuals, NULL);
          std::cout << "INITIAL COST (" << i << ")" << std::endl << "  x: ";
          for (size_t k = 0; k < data[i].observations[index].features.size(); ++k)
            std::cout << "  " << std::setw(10) << std::fixed << residuals[(3*k + 0)];
          std::cout << std::endl << "  y: ";
          for (size_t k = 0; k < data[i].observations[index].features.size(); ++k)
            std::cout << "  " << std::setw(10) << std::fixed << residuals[(3*k + 1)];
          std::cout << std::endl << "  z: ";
          for (size_t k = 0; k < data[i].observations[index].features.size(); ++k)
            std::cout << "  " << std::setw(10) << std::fixed << residuals[(3*k + 2)];
          std::cout << std::endl << std::endl;
        }

        problem->AddResidualBlock(cost,
                                  NULL,  // squared loss
                                  free_params);
      }
      else if (params.error_blocks[j].type =="camera3d_to_ground")
      {
        std::string camera_name = static_cast<std::string>(params.error_blocks[j].params["camera"]);
        std::string ground_name = static_cast<std::string>(params.error_blocks[j].params["ground"]);

        // Check that this sample has the required features/observations
        if (!hasSensor(data[i], camera_name) || !hasSensor(data[i], ground_name))
          continue;

        // Create the block
        ceres::CostFunction * cost = GroundPlaneError::Create(
          dynamic_cast<Camera3dModel*>(models_[camera_name]),
          z_,
          offsets_.get(), data[i]);

        int index = -1;
        for (size_t k =0; k < data[i].observations.size() ; k++)
        {
          if ( data[i].observations[k].sensor_name == camera_name)
          {
            index = k;
            break;
          }
        }
        
        if(index == -1)
        {
          std::cerr << "Sensor name doesn't exist" << std::endl;
          return 0;
        }

        if (progress_to_stdout)
        {
          double ** params = new double*[1];
          params[0] = free_params;
          double * residuals = new double[data[i].observations[index].features.size()];

          cost->Evaluate(params, residuals, NULL);

          std::cout << std::endl << "  z: ";
          for (size_t k = 0; k < data[i].observations[index].features.size(); ++k)
            std::cout << "  " << std::setw(10) << std::fixed << residuals[(k)];
          std::cout << std::endl << std::endl;
        }

        problem->AddResidualBlock(cost,
                                  NULL /* squared loss */,
                                  free_params);
      }
      else if (params.error_blocks[j].type =="camera3d_to_gripper")
      {
        std::string camera_name = static_cast<std::string>(params.error_blocks[j].params["camera"]);
        std::string gripper_name = static_cast<std::string>(params.error_blocks[j].params["gripper"]);

        // Check that this sample has the required features/observations
        if (!hasSensor(data[i], camera_name) || !hasSensor(data[i], gripper_name))
        { 
          continue;
        }

        // Create the block
        ceres::CostFunction * cost = GripperDepthError::Create(
            dynamic_cast<Camera3dModel*>(models_[camera_name]),
            dynamic_cast<ChainModel*>(models_[gripper_name]),
            offsets_.get(), data[i]);

        int index = -1;
        for (size_t k = 0; k < data[i].observations.size() ; k++)
        {
            if ( data[i].observations[k].sensor_name == camera_name)
            {
             index = k;
             break;
            }
        }

        if(index == -1)
        {
          std::cerr << "Sensor name doesn't exist" << std::endl;
          return 0;
        }

        if (progress_to_stdout)
        {
          double ** params = new double*[1];
          params[0] = free_params;
          double * residuals = new double[data[i].observations[index].features.size() * 3];  // TODO: should check that all features are same length?
          
          cost->Evaluate(params, residuals, NULL);
    /*                  std::cout << "INITIAL COST (" << i << ")" << std::endl << "  x: ";
                      for (size_t k = 0; k < data[i].observations[index].features.size(); ++k)
                      std::cout << "  " << std::setw(10) << std::fixed << residuals[(3*k + 0)];
                      std::cout << std::endl << "  y: ";
                      for (size_t k = 0; k < data[i].observations[index].features.size(); ++k)
                      std::cout << "  " << std::setw(10) << std::fixed << residuals[(3*k + 1)];
                      std::cout << std::endl << "  z: ";
                      for (size_t k = 0; k < data[i].observations[index].features.size(); ++k)
                      std::cout << "  " << std::setw(10) << std::fixed << residuals[(3*k + 2)];
                      std::cout << std::endl << std::endl;
      */     
        }

        problem->AddResidualBlock(cost,
            NULL /* squared loss */,
            free_params);
      }
      else if (params.error_blocks[j].type =="camera3d_to_led")
      {
        //std::cout << "in camera3dtoled" << std::endl;
        std::string camera_name = static_cast<std::string>(params.error_blocks[j].params["camera"]);
        std::string gripper_name = static_cast<std::string>(params.error_blocks[j].params["gripper"]);
 
        //std::cout << camera_name << "\t" << gripper_name << std::endl;
        // Check that this sample has the required features/observations
        if (!hasSensor(data[i], camera_name) || !hasSensor(data[i], gripper_name))
        {
          continue;
        }

        // Create the block
        ceres::CostFunction * cost = GripperColorError::Create(
            dynamic_cast<Camera2dModel*>(models_[camera_name]),
            dynamic_cast<MultiChainModel*>(models_[gripper_name]),
            offsets_.get(), data[i]);

        int index = -1;
        for (size_t k =0; k < data[i].observations.size() ; k++)
        {
          if (data[i].observations[k].sensor_name == camera_name)
            {
             index = k;
             break;
            }

        }

        if(index == -1)
        {
          std::cerr << "Sensor name doesn't exist" << std::endl;
          return 0;
        }
 
        if (progress_to_stdout)
        {
          double ** params = new double*[1];
          params[0] = free_params;
          double * residuals = new double[data[i].observations[index].features.size() * 2];
          
          cost->Evaluate(params, residuals, NULL);
         /*             std::cout << "INITIAL COST (" << i << ")" << std::endl << "  x: ";
                      for (size_t k = 0; k < data[i].observations[index].features.size(); ++k)
                      std::cout << "  " << std::setw(10) << std::fixed << residuals[(2*k + 0)];
                      std::cout << std::endl << "  y: ";
                      for (size_t k = 0; k < data[i].observations[index].features.size(); ++k)
                      std::cout << "  " << std::setw(10) << std::fixed << residuals[(2*k + 1)];
                      //std::cout << std::endl << "  z: ";
                      //for (size_t k = 0; k < data[i].observations[index].features.size(); ++k)
                      //std::cout << "  " << std::setw(10) << std::fixed << residuals[(3*k + 2)];
                      std::cout << std::endl << std::endl;
           */
        }

        problem->AddResidualBlock(cost,
            NULL /* squared loss */,
            free_params);
      }
      else if (params.error_blocks[j].type == "outrageous")
      {
        // Outrageous error block requires no particular sensors, add to every sample
        problem->AddResidualBlock(
            OutrageousError::Create(offsets_.get(),
              params.error_blocks[j].name,
              static_cast<double>(params.error_blocks[j].params["joint_scale"]),
              static_cast<double>(params.error_blocks[j].params["position_scale"]),
              static_cast<double>(params.error_blocks[j].params["rotation_scale"])),
            NULL, // squared loss
            free_params);
      }
      else
      {
        // ERROR unknown
      }
    }
  }

  // Setup the actual optimization
  ceres::Solver::Options options;
  options.use_nonmonotonic_steps = true;
  options.function_tolerance = 1e-10;
  options.linear_solver_type = ceres::DENSE_QR;
  options.max_num_iterations = 1000;
  options.minimizer_progress_to_stdout = progress_to_stdout;
  //  options.use_nonmonotonic_steps = true;

  if (progress_to_stdout)
    std::cout << "\nSolver output:" << std::endl;
  summary_.reset(new ceres::Solver::Summary());
  ceres::Solve(options, problem, summary_.get());
  if (progress_to_stdout)
    std::cout << "\n" << summary_->BriefReport() << std::endl;

  // TODO output stats
  /*if (progress_to_stdout)
    {
    CalibrationOffsetParser no_offsets;
    offsets_->update(free_params);
    for (size_t i = 0; i < data.size(); ++i)
    {
    std::cout << "Sample " << i << std::endl;
    printSimpleDistanceError(arm_model_, camera_model_, &no_offsets, offsets_, data[i]);
    printComparePoints(arm_model_, camera_model_, &no_offsets, offsets_, data[i]);
    }
    }*/

  // Note: the error blocks will be managed by scoped_ptr in cost functor
  //       which takes ownership, and so we do not need to delete them here

  // Done with our free params
  delete[] free_params;
  delete problem;

  return 0;
}

}  // namespace robot_calibration
