// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// perception_class.cpp
//
// Implements a Perception class to handle visualization for operating a robotic manipulator
// utilizing an Intel Realsense D435i sensor
//
// Utilizes code originally created by Victoria Albanese from UMASS Lowell while working 
// at the NERVE Center on the Verizon 5G Challenge Project (Particularly voxel_filter, 
// concatenate_clouds, and move_least_squares functions <- thank you Victoria!)
//
// ********************************************************************************************

#include "perception_class.hpp"

// ********************************************************************************************
// Private Functions
// ********************************************************************************************

// VOXEL FILTER FUNCTION
// downsamples point cloud to make the resulting model cleaner
PointCloud<PointXYZRGB> Perception::voxelgrid_filter(PointCloud<PointXYZRGB>::Ptr cloud)
{
  PointCloud<PointXYZRGB>::Ptr filtered_cloud(new PointCloud<PointXYZRGB>);

  VoxelGrid<PointXYZRGB> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.01, 0.01, 0.01); 
  sor.filter(*filtered_cloud);

  return *filtered_cloud;
}

PointCloud<PointXYZRGB> Perception::sac_segmentation(PointCloud<PointXYZRGB>::Ptr cloud)
{
  ModelCoefficients::Ptr coefficients (new ModelCoefficients);
  PointIndices::Ptr inliers (new PointIndices);
  // Create the segmentation object
  SACSegmentation<PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (SACMODEL_PLANE);
  seg.setMethodType (SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  //seg.setInputCloud (cloud);
  //seg.segment (*inliers, *coefficients);

  // Create the filtering object
  ExtractIndices<PointXYZRGB> extract;

  int i = 0, nr_points = (int) cloud->points.size ();
  // While 30% of the original cloud is still there
  while (cloud->points.size () > 0.6 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud);
    i++;
  }

  return *cloud;
}

// MOVE LEAST SQUARES FUNCTION
// aligns the surface normals to eliminate noise
PointCloud<PointNormal> Perception::move_least_squares(PointCloud<PointXYZRGB>::Ptr cloud)
{
  search::KdTree<PointXYZRGB>::Ptr tree(new search::KdTree<PointXYZRGB>);
  PointCloud<PointNormal> mls_points;
  MovingLeastSquares<PointXYZRGB, PointNormal> mls;
 
  mls.setComputeNormals(true);
  mls.setInputCloud(cloud);
  mls.setPolynomialOrder(4);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.1);
  mls.process(mls_points);

  return mls_points;
}

// WRIST CAMERA CALLBACK FUNCTION
// initialize cloud right with the information from wrist_camera
// converts pointcloud from sensor_msgs::PointCloud2 to pcl::PointXYZRGB
void Perception::wrist_camera_callback(const sensor_msgs::PointCloud2 msg)
{
    fromROSMsg(msg, this->current_cloud);
 
    ros::Time stamp;
    pcl_conversions::toPCL(stamp, this->current_cloud.header.stamp);
    this->transform_listener_ptr->waitForTransform("/world", this->current_cloud.header.frame_id, stamp, ros::Duration(10.0));
    pcl_ros::transformPointCloud("/world", this->current_cloud, this->current_cloud, *this->transform_listener_ptr);
}

// ********************************************************************************************
// Public Functions
// ********************************************************************************************

// INITIALIZE SUBSCRIBER FUNCTION
// Seperate "constructor" for initialization of subscriber due to passing shared pointers as arguments before creating them
void Perception::init_subscriber(ros::NodeHandle nodeHandle)
{
  this->wrist_camera_sub = nodeHandle.subscribe("/pcl_filters/wrist_camera_xyz_filter/output", 1, &Perception::wrist_camera_callback, this);
}

// PERCEPTION CLASS CONSTRUCTOR
// Create instance of Perception class and instantiate publisher for combined cloud 
// and subscriber for pointcloud from wrist camera sensor
Perception::Perception(ros::NodeHandle handle)
{
  this->combined_cloud_pub = handle.advertise<sensor_msgs::PointCloud2>("combined_cloud", 1);
}

// PUBLISH COMBINED CLOUD FUNCTION
// Publish combined (concatenated) point cloud
// Convert combined_cloud (pcl::PointXYZRGB) to sensor_msgs::PointCloud2
// and then publish
void Perception::publish_combined_cloud()
{

  sensor_msgs::PointCloud2 cloud;
  toROSMsg(this->combined_cloud, cloud);

  this->combined_cloud_pub.publish(cloud);
}

