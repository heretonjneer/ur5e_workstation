// ********************************************************************************************
// Author: Brian Flynn;
// Test Engineer - NERVE Center @ UMASS Lowell
// concatenate_cloud.cpp
//
// perception_class function
// ********************************************************************************************

#include "perception_class.hpp"

// CONCATENATE CLOUDS FUNCTION
// concatenates the points of all the cloud members into the combined pointcloud
// then performs downsampling (voxel_filter) and noise reduction (move_least_squares)
// to clean up resulting published pointcloud
void Perception::concatenate_clouds() 
{
  PointCloud<PointXYZRGB>::Ptr temp_cloud(new PointCloud<PointXYZRGB>);
  //PointCloud<PointXYZRGB>::Ptr segment_cloud(new PointCloud<PointXYZRGB>);
    
  *temp_cloud = this->left_cloud;
  *temp_cloud+= this->right_cloud;
  *temp_cloud+= this->top_cloud;
  *temp_cloud+= this->front_cloud;

  *temp_cloud = this->sac_segmentation(temp_cloud);
  *temp_cloud = this->voxelgrid_filter(temp_cloud);

  //*temp_cloud-= *segment_cloud;

  this->combined_cloud = this->move_least_squares(temp_cloud); 
}

