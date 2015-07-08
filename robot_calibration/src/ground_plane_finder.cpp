#include <robot_calibration/capture/ground_plane_finder.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl/filters/filter.h>

namespace robot_calibration
{

GroundPlaneFinder::GroundPlaneFinder(ros::NodeHandle & nh) :
  FeatureFinder(nh),
  waiting_(false)
{
   
  std::string topic_name;
  nh.param<std::string>("topic", topic_name, "/points");
  subscriber_ = nh.subscribe(topic_name,
                            1,
                            &GroundPlaneFinder::cameraCallback,
                            this);

  publisher_ = nh.advertise<sensor_msgs::PointCloud2>("ground_plane_points", 10);
  if (!depth_camera_manager_.init(nh))
 {
    // Error will be printed in manager
    throw;
 }

}

void GroundPlaneFinder::cameraCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{ 
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud , *cloud , indices); 
 

  if (waiting_)
  {
    cloud_ptr_ = cloud;
    waiting_ = false;
  }
}

bool GroundPlaneFinder::waitForCloud()
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
//Write the 'finder''

bool GroundPlaneFinder::find(robot_calibration_msgs::CalibrationData * msg)
{

  geometry_msgs::PointStamped rgbd;
  geometry_msgs::PointStamped world;

 
if(!waitForCloud())
  {
    ROS_ERROR("No point cloud data");
    return false;
  }

 int points_total = 80;
 
 /*

int points_x = 5;
 int points_y = 4;

 size_t step_x = cloud_ptr_->width/points_x;
 size_t step_y = cloud_ptr_->height/points_y;
*/
//std::cout<<"width"<<cloud_ptr_->width<<std::endl;
//std::cout<<"height"<<cloud_ptr_->height<<std::endl; 
  std::vector<cv::Point2f> points;
  //points.resize(points_x * points_y);
  points.resize(1 * points_total);

//for (size_t i=step_x, i< cloud_ptr_->width, i=i+step_x)
 //{
  //for (size_t j=step_y, j< cloud_ptr_->height, j=j+step_y)
  //{
   // Create PointCloud2 to publish
    sensor_msgs::PointCloud2 cloud;
    cloud.width = 0;
    cloud.height = 0;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = cloud_ptr_->header.frame_id;
    sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
    cloud_mod.setPointCloud2FieldsByString(1, "xyz");
    //cloud_mod.resize(points_x * points_y);
    cloud_mod.resize(1* points_total);
    sensor_msgs::PointCloud2Iterator<float> iter_cloud(cloud, "x");

    // Set msg size
    msg->observations.resize(2);
    msg->observations[0].sensor_name = "camera";  
    //msg->observations[0].features.resize(points_x * points_y);
    msg->observations[0].features.resize(1 * points_total);
    msg->observations[1].sensor_name = "ground";     
    //msg->observations[1].features.resize(points_x * points_y);
    msg->observations[1].features.resize(1 * points_total);

    int step = cloud_ptr_->width/points_total ;

    size_t k=0;

    for (size_t i= step ; i<cloud_ptr_->width; i=i+step)
    {
      points[k].x = i;
      k++;
    }
/*
size_t k=0;

for (size_t i=step_x; i< cloud_ptr_->width; i=i+step_x)
 {
  for (size_t j=step_y; j< cloud_ptr_->height; j=j+step_y)
  {
   points[k].x = i;
   points[k].y = j;
   k++;
  }
 }
*/
   
/*for (size_t i=step_x; i< cloud_ptr_->width; i=i+step_x)
 {
  for (size_t j=step_y; j< cloud_ptr_->height; j=j+step_y)
  {
    //for ground plane world can just be zero as we are concerned only with z  
    world.point.x = 0;
    world.point.y = 0;
    world.point.z = 0;

   // Get 3d point
      int index = (int)(points[i].y) * cloud_ptr_->width + (int)(points[i].x);
      rgbd.point.x = cloud_ptr_->points[index].x;
      rgbd.point.y = cloud_ptr_->points[index].y;
      rgbd.point.z = cloud_ptr_->points[index].z;

      msg->observations[0].features[i] = rgbd;
      msg->observations[1].features[i] = world;

      iter_cloud[0] = rgbd.point.x;
      iter_cloud[1] = rgbd.point.y;
      iter_cloud[2] = rgbd.point.z;
      ++iter_cloud;

  }
 }
*/

for (size_t i=0; i<points.size() ; i++)
{
 //for ground plane world can just be zero as we are concerned only with z  
    world.point.x = 0;
    world.point.y = 0;
    world.point.z = 0;
      
    //std::vector<int> indices;
    //pcl::removeNaNFromPointCloud(cloud , cloud , indices);
    //pcl::removeNaNFromPointCloud(cloud_ptr_ , cloud_ptr_ , indices);
   
    // Get 3d point
      //int index = (int)(points[i].y) * cloud_ptr_->width + (int)(points[i].x);
      int index = (int)points[i].x ;
      rgbd.point.x = cloud_ptr_->points[index].x;
      rgbd.point.y = cloud_ptr_->points[index].y;
      rgbd.point.z = cloud_ptr_->points[index].z;
      //std::cout<<"x: "<<rgbd.point.x<<std::endl;
      //std::cout<<"y: "<<rgbd.point.y<<std::endl;
      //std::cout<<"z: "<<rgbd.point.z<<std::endl;
      
     /* if (isnan(rgbd.point.x) ||
          isnan(rgbd.point.y) ||
          isnan(rgbd.point.z))
      {
        ROS_ERROR_STREAM("NAN point on " << i);
        //return false;
        rgbd.point.x = 0;
        rgbd.point.y = 0;
        rgbd.point.z = 0;
      }*/
     

      msg->observations[0].features[i] = rgbd;
      msg->observations[0].ext_camera_info = depth_camera_manager_.getDepthCameraInfo();
      msg->observations[1].features[i] = world;

      iter_cloud[0] = rgbd.point.x;
      iter_cloud[1] = rgbd.point.y;
      iter_cloud[2] = rgbd.point.z;
      ++iter_cloud;
}

//std::cout<< cloud<< std::endl ;
 publisher_.publish(cloud);
return true;

}
}
