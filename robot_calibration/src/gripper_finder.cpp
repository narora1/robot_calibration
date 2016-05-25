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

#include <robot_calibration/capture/gripper_finder.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <math.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <algorithm>
#include <vector>
#include <iostream>
#include <iterator>
#include <opencv2/rgbd/rgbd.hpp>
#include <geometry_msgs/Point32.h>
#include <tf/transform_listener.h>

namespace robot_calibration
{

GripperFinder::GripperFinder(ros::NodeHandle & nh) :
  FeatureFinder(nh),
  waiting_(true)
{
  std::string topic_name;

  subscriber_ = nh.subscribe("/camera/xyz_image",
      1,
      &GripperFinder::cameraCallback,
      this);
  // Get sensor names
  nh.param<std::string>("camera_sensor_name", camera_sensor_name_, "cameradepth");
  nh.param<std::string>("chain_sensor_name", chain_sensor_name_, "gripperdepth");

//  camera_info_sub_ = nh.subscribe<sensor_msgs::CameraInfo>(
//      "/head_camera/depth/camera_info",
//      10, &GripperDepthFinder::cameraInfoCallback, this);
//  publisher_ = nh.advertise<robot_calibration_msgs::Plane>("plane_depth", 10);
  image_pub_cluster_ = nh.advertise<sensor_msgs::Image>("clusters", 1);
  image_pub_gripper_cluster_ = nh.advertise<sensor_msgs::Image>("gripper_cluster", 1);
  if (!depth_camera_manager_.init(nh))
  {
    // Error will be printed in manager
    throw;
  }
}

void GripperFinder::cameraCallback(const sensor_msgs::ImageConstPtr& image)
{
  if (waiting_)
  {
    image_ = image;
    waiting_ = false;
  }
} 

bool GripperFinder::waitForCloud()
{
  ros::Duration(1/10.0).sleep();

  waiting_ = true;
  int count = 250;
  while (--count)
  {
    if (!waiting_)
    {
      // success
      return true;
    }
    ros::Duration(0.01).sleep();
    ros::spinOnce();
  }
  ROS_ERROR("Failed to get cloud");
  return !waiting_;
}

bool GripperFinder::find(robot_calibration_msgs::CalibrationData * msg)
{

  if (!waitForCloud())
  {
    return false;
  }

  cv_bridge::CvImagePtr cv_ptr;

   try
  {
    cv_ptr = cv_bridge::toCvCopy(image_, "16SC3");

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }

    // Clustering

  int total_size = cv_ptr->image.rows * cv_ptr->image.cols;
  std::vector<bool> checked (total_size , false);

  std::vector< std::vector<size_t> >clusters;
  for (size_t i = 1 ; i < cv_ptr->image.rows - 1; i++)
  {
    for(size_t j = 1; j < cv_ptr->image.cols - 1; j++)
    {
      cv::Vec3f coordinates = cv_ptr->image.at<cv::Vec3f>(i,j);
      if(!std::isfinite(coordinates.val[0]) || !std::isfinite(coordinates.val[1]) || !std::isfinite(coordinates.val[0]))//cv_ptr->image.at<float>(i,j) == FLT_MAX)
      {
        continue;
      }         

      std::vector<size_t> seed_q;
      size_t index = i * cv_ptr->image.cols + j;

      //if checked the pixel already then continue
      if(checked[index])
        continue;

      //create a seed queue for clustering
      seed_q.push_back(index);

      size_t seed_index=0;

      checked[index] = true;

      while (seed_index < static_cast<int> (seed_q.size ()))
      {
        // Search for seed queue index
        int m;
        int n;
        m = seed_q[seed_index]/cv_ptr->image.cols;            
        n = seed_q[seed_index] % cv_ptr->image.cols;
        for (int x =- 1; x < 2; x++)
        {  
          for(int y =- 1; y < 2; y++)
          {
            size_t k = (m+x) * cv_ptr->image.cols + (n+y);
            //ensure that region growing doesn't go outside the image
            if ((m+x) < 0 || (n+y) < 0 || (m+x)>=cv_ptr->image.rows ||(n+y)>=cv_ptr->image.cols)
              continue;                     

            //if (mgod->image.at<float>(m+x,n+y) >0.5)
            //  continue;

            if (checked[k])                         
              continue;

            cv::Vec3f current = cv_ptr->image.at<cv::Vec3f>(m,n);
            cv::Vec3f neighbour = cv_ptr->image.at<cv::Vec3f>(m+x,n+y);          
            float distance =  std::sqrt( pow((current.val[0] - neighbour.val[0]),2) + pow((current.val[1] - neighbour.val[1]),2) + pow((current.val[2] - neighbour.val[2]),2));
            
            // if the pixels belong to the same bin then add them to the queue
            if (distance <= 10)//cluster->image.at<float>(m,n) == cluster->image.at<float>(m+x,n+y)   )
            {             
              checked[k] = true;
              seed_q.push_back (k);
            }
          }
        }
        seed_index++; 
      }

      if (seed_q.size () >= 10 && seed_q.size () <= 5000000)
      {
        clusters.push_back(seed_q );
      }
    }
  }


  cv::Mat clustered= cv::Mat::zeros( cv_ptr->image.size(), CV_8UC3);
  cv::Mat individual_clusters = cv::Mat::zeros(cv_ptr->image.size(), CV_32FC4);
 
   cv::RNG rng(12345); 
  for (size_t i = 0; i < clusters.size(); i++)
  {
    cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    for (size_t j = 0; j < clusters[i].size(); j++)
    {
      int  m = clusters[i][j] / cv_ptr->image.cols;
      int  n = clusters[i][j] % cv_ptr->image.cols;

      clustered.at<cv::Vec3b>(m,n)[0] = color[0];
      clustered.at<cv::Vec3b>(m,n)[1] = color[1];
      clustered.at<cv::Vec3b>(m,n)[2] = color[2];
      individual_clusters.at<cv::Vec4f>(m,n)[0] = i;
      individual_clusters.at<cv::Vec4f>(m,n)[1] = cv_ptr->image.at<cv::Vec3f>(m,n)[0];
      individual_clusters.at<cv::Vec4f>(m,n)[2] = cv_ptr->image.at<cv::Vec3f>(m,n)[1];
      individual_clusters.at<cv::Vec4f>(m,n)[3] = cv_ptr->image.at<cv::Vec3f>(m,n)[2];


    }
  }  

  cv::Mat channels[3];
  cv::split(cv_ptr->image, channels);


  cv::Mat centroids;

  // calculate centroids for all the clusters
  for(size_t i = 0 ; i < clusters.size(); i++)
  {
    int count = 0;
    double x = 0;
    double y = 0;
    double z = 0;
    for(size_t j = 0; j < clusters[i].size(); j++)
    {

      int m = clusters[i][j] /cv_ptr->image.cols;
      int n = clusters[i][j] % cv_ptr->image.cols;

      if(cv_ptr->image.at<float>(m,n) == FLT_MAX ||  isnan(channels[0].at<float>(m,n))  || isnan(channels[1].at<float>(m,n)) || isnan(channels[2].at<float>(m,n)))
      {}
      else
      {
        count++ ;
        x += channels[0].at<float>(m,n);
        y += channels[1].at<float>(m,n);
        z += channels[2].at<float>(m,n);
      }
    }

    cv::Vec3f centroid_point(x/count, y/count, z/count);

    centroids.push_back(centroid_point);
  }

  // the expected gripper centroid transformed to the camera frame
  tf::Stamped<tf::Point> gripper_centroid;
  gripper_centroid.setX(0.06);
  gripper_centroid.setY(0.00);
  gripper_centroid.setZ(0.0396);
  gripper_centroid.frame_id_ = "/wrist_roll_link";
  tf::Stamped<tf::Point> gripper_centroid_transformed;
  tf::TransformListener listener;
  ros::Time time = ros::Time(0);
  try
  {

   // std::cout << image_->header.frame_id << std::endl;
    listener.waitForTransform( "/camera_optical_link" /*image_->header.frame_id*/, gripper_centroid.frame_id_,
        time, ros::Duration(3.0));
    listener.transformPoint(  "/camera_optical_link"/*image_->header.frame_id*/,
        time,  gripper_centroid , gripper_centroid.frame_id_, gripper_centroid_transformed);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  cv::Vec3f gripper_centroid_transform;
  gripper_centroid_transform[0] = gripper_centroid_transformed.x();
  gripper_centroid_transform[1] = gripper_centroid_transformed.y();
  gripper_centroid_transform[2] = gripper_centroid_transformed.z();

  cv::Vec3f point_on_plane_1( 0, -0.0625, 0.0396);
  cv::Vec3f point_on_plane_2( 0, 0.0625, 0.0396);
  cv::Vec3f point_on_plane_3( 0.12, 0.0625, 0.0396);
  cv::Vec3f point_on_plane_4( 0.12, -0.0625, 0.0396);

  cv::Mat points_on_plane;
  points_on_plane.push_back(point_on_plane_1);
  points_on_plane.push_back(point_on_plane_2);
  points_on_plane.push_back(point_on_plane_3);
  points_on_plane.push_back(point_on_plane_4);

  int idx_cam = msg->observations.size() + 0;
  int idx_chain = msg->observations.size() + 1;
  msg->observations.resize(msg->observations.size() + 2);
  msg->observations[idx_cam].sensor_name = camera_sensor_name_;
  msg->observations[idx_chain].sensor_name = chain_sensor_name_;
  cv::Mat points_on_plane_transformed;
  tf::StampedTransform transform;

  for(size_t i=0; i< points_on_plane.rows;i++)
  {
    geometry_msgs::PointStamped rgbd_pt;
    rgbd_pt.point.x = points_on_plane.at<cv::Vec3f>(i,0)[0];
    rgbd_pt.point.y = points_on_plane.at<cv::Vec3f>(i,0)[1];
    rgbd_pt.point.z = points_on_plane.at<cv::Vec3f>(i,0)[2];
    rgbd_pt.header.frame_id = "/wrist_roll_link";

    msg->observations[idx_chain].features.push_back(rgbd_pt);

  }

  try
  {
    for (size_t i = 0; i < points_on_plane.rows; i++)
    {
      tf::Stamped<tf::Point> point;
      point.frame_id_ = "/wrist_roll_link";
      point.setX( points_on_plane.at<cv::Vec3f>(i,0)[0]);
      point.setY( points_on_plane.at<cv::Vec3f>(i,0)[1]);
      point.setZ( points_on_plane.at<cv::Vec3f>(i,0)[2]);
      tf::Stamped<tf::Point> point_transformed;
      //    ros::Time time;
      listener.waitForTransform(  "/camera_optical_link"/*image_->header.frame_id*/, point.frame_id_,
          time, ros::Duration(3.0));

      listener.transformPoint(  "/camera_optical_link",
          time,  point , point.frame_id_, point_transformed);

      cv::Vec3f point_transform;
      point_transform[0] = point_transformed.x();
      point_transform[1] = point_transformed.y();
      point_transform[2] = point_transformed.z();

      points_on_plane_transformed.push_back(point_transform);
    }
  }
  catch(tf::TransformException &ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }


  //find equation of the plane in the base frame
  cv::Vec3f V0 = points_on_plane_transformed.at<cv::Vec3f>(0,0);
  cv::Vec3f V1 = points_on_plane_transformed.at<cv::Vec3f>(1,0);
  cv::Vec3f V2 = points_on_plane_transformed.at<cv::Vec3f>(2,0);

  cv::Vec3f normal = (V2-V0).cross(V1-V0);
  cv::Vec4f plane_transformed;
  float distance = sqrt (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
  plane_transformed[0] = normal[0] / distance;
  plane_transformed[1] = normal[1] / distance;
  plane_transformed[2] = normal[2] /distance;
  plane_transformed[3] = - V0.dot(normal/distance);

  std::cout << "gripper coeeficients" << std::endl;
  std::cout << plane_transformed[0] << "\t" << plane_transformed[1] << "\t" << plane_transformed[2] << "\t" << plane_transformed[3] << std::endl;

  // find the closest cluster centroid to the gripper centroid
  float min_distance = 1000;
  int closest_centroid= 1000;

  for( size_t i = 0; i < centroids.rows; i++)
  {
    float distance = pow((gripper_centroid_transform[0]-centroids.at<cv::Vec3f>(i,0)[0]) ,2) +
      pow((gripper_centroid_transform[1]-centroids.at<cv::Vec3f>(i,0)[1]) ,2) +
      pow((gripper_centroid_transform[2]-centroids.at<cv::Vec3f>(i,0)[2]) ,2);

    distance = sqrt(distance);
    if (distance < min_distance)
    {
      min_distance = distance ;
      closest_centroid = i;
    }
  }

  cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
  cv::Mat gripper_cluster = cv::Mat::zeros( cv_ptr->image.size(), CV_8UC3);
  cv::Mat gripper_cluster_ = cv::Mat::zeros( cv_ptr->image.size(), CV_32FC1);

  cv::Mat some_1 = cv::Mat::zeros( cv_ptr->image.size(), CV_32FC1);
  cv::Mat some_2 = cv::Mat::zeros( cv_ptr->image.size(), CV_32FC1);
  cv::Mat some_3 = cv::Mat::zeros( cv_ptr->image.size(), CV_32FC1);

  //display the closest cluster
  for (size_t j = 0; j < clusters[closest_centroid].size(); j++)
  {
    int m = clusters[closest_centroid][j] / cv_ptr->image.cols;
    int n = clusters[closest_centroid][j] % cv_ptr->image.cols;

    if(!isnan(channels[0].at<float>(m,n)) && !isnan(channels[1].at<float>(m,n)) && !isnan(channels[2].at<float>(m,n)))
    {
      geometry_msgs::PointStamped rgbd_pt;
      rgbd_pt.point.x = channels[0].at<float>(m,n);
      rgbd_pt.point.y = channels[1].at<float>(m,n);
      rgbd_pt.point.z = channels[2].at<float>(m,n);
      rgbd_pt.header.frame_id = "/wrist_roll_link";

      msg->observations[idx_cam].features.push_back(rgbd_pt);
      some_1.at<float>(m,n) = channels[0].at<float>(m,n);
      some_2.at<float>(m,n) = channels[1].at<float>(m,n);
      some_3.at<float>(m,n) = channels[2].at<float>(m,n);
    }
    //std::cout << some_3.at<float>(m,n) << std::endl;
    gripper_cluster_.at<float>(m,n) = cv_ptr->image.at<float>(m,n);
    gripper_cluster.at<cv::Vec3b>(m,n)[0] = color[0];
    gripper_cluster.at<cv::Vec3b>(m,n)[1] = color[1];
    gripper_cluster.at<cv::Vec3b>(m,n)[2] = color[2];

  }

ros::Time time_ = ros::Time::now();    

    // convert OpenCV image to ROS message
    cv_bridge::CvImage cvi;     
    cvi.header.stamp = time_;
    cvi.header.frame_id = "camera";
    cvi.encoding = "bgr8";
    cvi.image =  clustered;
    image_pub_cluster_.publish( cvi.toImageMsg());

    cv_bridge::CvImage cvii;
    cvii.header.stamp = time;
    cvii.header.frame_id = "camera";
    cvii.encoding = "bgr8";
    cvii.image =  gripper_cluster;
    image_pub_gripper_cluster_.publish( cvii.toImageMsg());

return true;  
}




} // namespace robot_calibration








