#ifndef NAV_MAIN_H_
#define NAV_MAIN_H_

#include <iostream>
#include <chrono>
#include <thread>
#include <memory>

#include <ros/ros.h>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "sensor_fusion/ResetIMU.h"

#include "gps_data.h"
#include "imu_data.h"
#include "ekf.h"

namespace navigation
{

class NavigationNode
{
public:
    NavigationNode(const ros::NodeHandlePtr &nh);
    ~NavigationNode();

    void Spin();
    void ProcessCartesianPose();
    void PublishData(ros::Time &current_time);

    void FillPoseMsg(Eigen::Vector3d &position, Eigen::Quaterniond &angle, nav_msgs::Odometry &msg);
    void FillTwistMsg(Eigen::Vector3d &linear_velocity, Eigen::Vector3d &angular_velocity, nav_msgs::Odometry &msg);
    void correct_orientation (Eigen::Quaterniond&);
    bool resetCB (sensor_fusion::ResetIMU::Request& req,
                  sensor_fusion::ResetIMU::Response& res);

private:
    ros::NodeHandlePtr nh_;

    ros::Subscriber gpsTwistSubscriber_;
    ros::Subscriber imuSubscriber_;

    ros::Publisher navigationOdomPublisher_;
    ros::Publisher odom_pub_;

    GPSData gpsData_;
    IMUData imuData_;

    ExtendedKalmanFilter gpsFilter_;
    Eigen::Vector3d poseEstimation_;
    Eigen::Vector3d localPoseEstimation_;

    Eigen::Quaterniond quaternion_;
    Eigen::Vector3d position_;
    Eigen::Vector3d incrementPosition_;
    Eigen::Vector3d velocity_;
    Eigen::Vector3d angularVelocity_;
    Eigen::Vector3d eulerAngel_;

    Eigen::Vector3d imu_offset_;
    Eigen::Vector3d local_position_;
    Eigen::Quaterniond local_orientation_;
    ros::ServiceServer reset_imu_;
};

} // namespace navigation
#endif //NAVIGATION_NODE_H_
