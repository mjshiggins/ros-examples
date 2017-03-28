// ROS Point Cloud DEM Generation

#include <cmath>
#include <vector>
#include <float.h>
#include <stdio.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

#define IMAGE_HEIGHT	701
#define IMAGE_WIDTH	801
#define BIN		0.1

// Global Publishers/Subscribers
ros::Subscriber subPointCloud;
ros::Publisher pubPointCloud;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PassThrough<pcl::PointXYZ> pass;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

double heightArray[IMAGE_HEIGHT][IMAGE_WIDTH];	// for a 700x800 image around the central lidar point [250][400]

// map meters to index
void map(double x, double y, int &row, int &column){

  }

void DEM(const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg)
{
  ROS_DEBUG("Point Cloud Received");

  // Clip
  pass.setInputCloud(cloud);
  pass.setFilterFieldName ("z");

  // Zero out height map
  for(int i = 0; i < (IMAGE_HEIGHT*IMAGE_WIDTH); ++i){
    // 
    heightArray[i] = (-FLT_MAX);
    }

  // Convert from ROS message to PCL point cloud
  pcl::fromROSMsg(*pointCloudMsg, *cloud);

  // Clip point cloud to the range specified by the paper

  // Populate the DEM by looping through every point
  for (size_t j = 0; j < cloud->points.size (); ++j){
    // Check bin
    }
}


int main(int argc, char** argv)
{
  ROS_INFO("Starting LIDAR Node");
  ros::init(argc, argv, "lidar_node");
  ros::NodeHandle nh;

  subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 2, DEM);

  ros::spin();

  return 0;
}
