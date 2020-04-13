#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>


ros::Publisher pub;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZ PointT;

void cloud_cb(const PointCloud::ConstPtr& cloud_msg){
  // Container for original & filtered data
  //pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  //pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  // Convert to PCL data type
  //pcl_conversions::toPCL(*cloud_msg, *cloud);
  //pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  

  //passthrough
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (cloud_msg);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-1000, 1000);
  pass.filter (*cloud_filtered);

  //normal estimation
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());  
  pcl::NormalEstimation<PointT, pcl::Normal> ne;  
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  //segmentation process
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
  pcl::ModelCoefficients::Ptr coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);  
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.005);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (0.7);
  seg.setRadiusLimits (0, 0.90);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  seg.segment (*inliers_cylinder, *coefficients_cylinder);

  //extract the points
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);

  pub.publish (cloud_cylinder);


}

int main(int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "segmentCylinders");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("lidar_wamv/points", 1000, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<PointCloud> ("pointSegCyl", 1000);

  // Spin
  ros::spin ();
}