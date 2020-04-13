#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl_ros/point_cloud.h>


ros::Publisher pub;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void callback(const PointCloud::ConstPtr& cloud_msg){
  pcl::PointXYZ cent;


  //create the centroid object
  //pcl::computeCentroid(*cloud_msg, cent->points)
  //for(int i=0;i<(int)cloud_msg->points.size();i++)
  //    centroid.add(cloud_msg->points[i]);
  
  //centroid.get(&cent);

  float x=0, y=0, z=0;
  for(int i=0;i<(int)cloud_msg->size();i++){
      x+=cloud_msg->at(i).x;
      z+=cloud_msg->at(i).z;
      y+=cloud_msg->at(i).y;
  }

  float a = x / (cloud_msg->size() + 0.0);
  float b = y / (cloud_msg->size() + 0.0);
  float c = z / (cloud_msg->size() + 0.0);
  printf("centroid is %f %f %f\n", a, b, c);
  //pub.publish(cent);
}

int main(int argc, char** argv){
  // Initialize ROS
  ros::init (argc, argv, "centroid");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("points2", 1, callback);

  // Create a ROS publisher for the output point cloud
  //pub = nh.advertise<PointCloud> ("centroids", 1);

  // Spin
  ros::spin ();
}