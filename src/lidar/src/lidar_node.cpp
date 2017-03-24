// ROS Point Cloud Filtering, Segmentation, and Clustering Example node
// MacCallister Higgins, Udacity
// Uses PCL and ROS tutorials
// http://pointclouds.org/documentation/tutorials/
// http://wiki.ros.org/roscpp/

// Standard library includes
#include <cmath>
#include <vector>

// PCL specific includes
// You'll need these to use PCL utilities
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

// ROS specific includes to handle PointCloud2 ROS messages
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

// Global Publishers/Subscribers
ros::Subscriber subPointCloud;
ros::Publisher pubPointCloud;

// Create "heavy" objects here so tat they aren't instantiated on every callback
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointIndicesPtr ground (new pcl::PointIndices);
pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
sensor_msgs::PointCloud2 pointCloudOutMsg;
pcl::VoxelGrid<pcl::PointXYZ> vg;
pcl::SACSegmentation<pcl::PointXYZ> seg;
pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
pcl::ExtractIndices<pcl::PointXYZ> extract;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
std::vector<pcl::PointIndices> cluster_indices;
pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

// Progressive Morphological Filtering Function
// This function is way too slow to run in real time
void removeGround_PMF(const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg)
{
  ROS_DEBUG("Point Cloud Received");

  // Convert from ROS message to PCL point cloud
  pcl::fromROSMsg(*pointCloudMsg, *cloud);

  // Extract the ground indices
  pmf.setInputCloud (cloud);
  pmf.setMaxWindowSize (20);
  pmf.setSlope (1.0f);
  pmf.setInitialDistance (0.5f);
  pmf.setMaxDistance (3.0f);
  pmf.extract (ground->indices);

  // Create the filtering object
  extract.setInputCloud (cloud);
  extract.setIndices (ground);
  extract.setNegative (true);
  extract.filter (*cloud_filtered);

  // Now that you're done using PCL tools, we will need to turn the filtered point cloud back into a compatible ROS message
  pcl::toROSMsg(*cloud_filtered, pointCloudOutMsg);
  pointCloudOutMsg.header.stamp = pointCloudMsg->header.stamp;
  pointCloudOutMsg.header.frame_id = "/velodyne";
  pubPointCloud.publish(pointCloudOutMsg);

}

// Progressive Morphological Filtering Function
void cluster(const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg)
{
  ROS_DEBUG("Point Cloud Received");

  // Convert from ROS message to PCL point cloud
  pcl::fromROSMsg(*pointCloudMsg, *cloud);

  // Create the filtering object: downsample the dataset using a leaf size of 0.5m
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.2f, 0.2f, 0.2f);
  vg.filter (*cloud_filtered);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      ROS_ERROR("Could not estimate a planar model for the given dataset.");
      break;
    }

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    //extract.filter (*cloud_plane);

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Create the KdTree object for the search method of the extraction
  tree->setInputCloud (cloud_filtered);
  ec.setClusterTolerance (1.0); // 1m
  ec.setMinClusterSize (10);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  // Create a single point cloud that has the centroid of each cluster
  // This allows us to combine point clouds from LIDAR and Radar to colocate obstacles!
  int x = 0, y = 0, z = 0, counter = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
    //pcl::CentroidPoint<pcl::PointXYZ> centroid;
    x = 0, y = 0, z = 0, counter = 0;
    // This interior loop iterates through every point in the individual cluster
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
      //centroid.add(cloud_filtered->points[*pit]);
      x += cloud_filtered->points[*pit].x;
      y += cloud_filtered->points[*pit].y;
      z += cloud_filtered->points[*pit].z;
      ++counter;
    }
    // Compute centroid
    pcl::PointXYZ c1;
    //centroid.get(c1);
    c1.x = x / counter;
    c1.y = y / counter;
    c1.z = z / counter;

    // Add it to the accumulated point cloud
    cloud_cluster->points.push_back(c1);
  }

  // Publish cluster centroid point cloud
  pcl::toROSMsg(*cloud_cluster, pointCloudOutMsg);
  pointCloudOutMsg.header.stamp = pointCloudMsg->header.stamp;
  pointCloudOutMsg.header.frame_id = "/velodyne";
  pubPointCloud.publish(pointCloudOutMsg);

  // Clear out point clouds that need to be reset
  cloud_cluster->clear();

}


// Initialize the ROS node and setup your subscribers/publishers here
int main(int argc, char** argv)
{
  // Start the ROS node and call it "lidar_node"
  // You'll see this if you run the command "rosnode list"
  ROS_INFO("Starting LIDAR Node");
  ros::init(argc, argv, "lidar_node");
  ros::NodeHandle nh;

  // Use the node handle to subscrbe to a topic (in this case the points published by the velodyne driver)
  // You can see a list of nodes to subscribe to by running the command "rostopic list"
  subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 2, cluster);

  // Again, use the node handle, but this time you're advertising that you'll be publishing data onto this topic
  pubPointCloud = nh.advertise<sensor_msgs::PointCloud2>("/lidar_centroids", 2);

  // Let ROS do its thing
  ros::spin();

  return 0;
}
