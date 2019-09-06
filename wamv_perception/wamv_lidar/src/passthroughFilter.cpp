#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/point_cloud.h>

ros::Publisher pub;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void callback(const PointCloud::ConstPtr& cloud_msg){
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new PointCloud);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new PointCloud);


  //passthrough filter
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_msg);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.0, 10.0);
  pass.filter(*cloud_filtered);

  //PointCloud* output (new PointCloud);

  pub.publish(cloud_filtered);
}

int main(int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "passthroughFilter");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("lidar_wamv/points", 1, callback);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<PointCloud> ("points2", 1);

  // Spin
  ros::spin ();
}