#include "nav_main.h"

namespace navigation
{
NavigationNode::NavigationNode(const ros::NodeHandlePtr &nh) : nh_(nh), quaternion_(0.0, 0.0, 0.0, 0.0)
{
    dvlTwistSubscriber_ = nh_->subscribe("/gps/velocity", 100,
                                         &DvlData::DvlTwistCallback, &dvlData_);
    dvlPressureSubscriber_ = nh_->subscribe("/anahita/pressure", 100,
                                            &DvlData::DvlPressureCallback, &dvlData_);
    imuSubscriber_ = nh_->subscribe("/imu/data", 100,
                                    &IMUData::IMUMsgCallback, &imuData_);

    navigationDepthOffsetServer_ = nh_->advertiseService("/nav/set_depth_offset",
                                                         &navigation::NavigationNode::SetDepthOffsetCallback, this);
    navigationXYOffsetServer_ = nh_->advertiseService("/nav/set_world_x_y_offset",
                                                      &navigation::NavigationNode::SetWorldXYOffsetCallback, this);

    navigationOdomPublisher_ = nh_->advertise<nav_msgs::Odometry>("/anahita/pose_gt/global", 100);
    odom_pub_ = nh_->advertise<nav_msgs::Odometry>("/anahita/pose_gt/relay", 100);

    reset_imu_ = nh_->advertiseService("/anahita/reset_imu", &navigation::NavigationNode::resetCB, this);
    
    position_ = Eigen::Vector3d::Zero();
    local_position_ = Eigen::Vector3d::Zero();
    incrementPosition_ = Eigen::Vector3d::Zero();
    velocity_ = Eigen::Vector3d::Zero();
    angularVelocity_ = Eigen::Vector3d::Zero();
    eulerAngel_ = Eigen::Vector3d::Zero();
    zOffset_ = 0;
    imu_offset_ = Eigen::Vector3d::Zero();

    ROS_INFO("Set up initial constructor successfully");
}

NavigationNode::~NavigationNode()
{
    dvlTwistSubscriber_.shutdown();
    dvlPressureSubscriber_.shutdown();
    imuSubscriber_.shutdown();
    navigationDepthOffsetServer_.shutdown();
    navigationXYOffsetServer_.shutdown();
}

void NavigationNode::Spin()
{
    ros::Rate loop_rate(15); // 100 hz
    while (ros::ok())
    {
        ros::spinOnce();
        ProcessCartesianPose();
        loop_rate.sleep();
    }
}

bool NavigationNode::SetDepthOffsetCallback(
    sensor_fusion::SetDepthOffset::Request &request,
    sensor_fusion::SetDepthOffset::Response &response)
{
    zOffset_ = dvlData_.GetPositionZFromPressure();
    imuData_.SetNewDataReady();
    ROS_INFO("Service completed");
    return true;
}

bool NavigationNode::SetWorldXYOffsetCallback(
    sensor_fusion::SetWorldXYOffset::Request &request,
    sensor_fusion::SetWorldXYOffset::Response &response)
{
    position_.x() = 0.0f;
    position_.y() = 0.0f;
    dvlData_.SetNewDataReady();
    ROS_INFO("Service completed");
    return true;
}

Eigen::Vector3d toEulerAngle(const Eigen::Quaterniond& q)
{
	// roll (x-axis rotation)
    double roll, pitch, yaw;
	double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
	double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
	roll = atan2(sinr_cosp, cosr_cosp);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
	double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());  
	yaw = atan2(siny_cosp, cosy_cosp);

    Eigen::Vector3d euler;
    euler[0] = roll;
    euler[1] = pitch;
    euler[2] = yaw;

    return euler;
}

bool NavigationNode::resetCB (sensor_fusion::ResetIMU::Request& req,
                              sensor_fusion::ResetIMU::Response& res) {
    Eigen::Vector3d euler = toEulerAngle (quaternion_);

    euler[0] = -euler[0];
    euler[1] = -euler[1];
    euler[2] = -euler[2];
    imu_offset_ = euler;

    local_position_.x() = 0.0f;
    local_position_.y() = 0.0f;
    dvlData_.SetNewDataReady();

    res.success = true;
    ROS_INFO("Service completed");
    return true; 
}

void NavigationNode::correct_orientation (Eigen::Quaterniond& q) {
    Eigen::Vector3d euler = toEulerAngle (q);
    euler = euler + imu_offset_;
    if (euler[2] >= M_PI) {
        euler[2] = euler[2] - 2*M_PI;
    }
    else if (euler[2] < -1*M_PI) {
        euler[2] = 2*M_PI - euler[2];
    }
    q = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());

    if (q.w() < 0 && q.z() < 0) {
        q.z() = std::abs(q.z());
        q.w() = std::abs(q.w());
    } else if (q.w() < 0 && q.z() > 0) {
        q.w() = std::abs(q.w());
        q.z() = -1*q.z();
    }
}

void NavigationNode::ProcessCartesianPose()
{
    if (dvlData_.IsNewDataReady() || imuData_.IsNewDataReady())
    {
        dvlData_.SetNewDataUsed();
        imuData_.SetNewDataUsed();

        incrementPosition_ = dvlData_.GetPositionXYZ();
        positionFromDepth_ = dvlData_.GetPositionZFromPressure();
        velocity_ = dvlData_.GetVelocityXYZ();
        angularVelocity_ = imuData_.GetAngularVelocity();
        eulerAngel_ = imuData_.GetOrientation();
        quaternion_ = imuData_.GetQuaternion();
        local_orientation_ = quaternion_;
        correct_orientation (local_orientation_);
        local_orientation_.x() = quaternion_.x();
        local_orientation_.y() = quaternion_.y();
        // ROS_INFO("Value of incrementPosition_: x: %f, y: %f,z: %f", incrementPosition_.x(), incrementPosition_.y(), incrementPosition_.z());
        position_ += quaternion_.toRotationMatrix() * incrementPosition_;
        local_position_ += local_orientation_.toRotationMatrix() * incrementPosition_;
        // ROS_INFO("(Without EKF) Position.x: %f, Position.y: %f, Position.z: %f", position_.x(), position_.y(), position_.z());

        position_.z() = 0;
        local_position_.z() = 0;

        dvlFilter_.Update(position_, poseEstimation_);
        dvlFilter_.Update(local_position_, localPoseEstimation_);

        ros::Time currentTime = ros::Time::now();

        PublishData(currentTime);
        // BroadcastTransform(poseEstimation_, quaternion_, currentTime);
    }
}

void NavigationNode::BroadcastTransform(Eigen::Vector3d &position,
                                        Eigen::Quaterniond &quaternion,
                                        ros::Time &current_time)
{
    // ROS_INFO("Publishing a transform");
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "world";
    odom_trans.child_frame_id = "anahita/base_link";

    odom_trans.transform.translation.x = position.x();
    odom_trans.transform.translation.y = position.y();
    odom_trans.transform.translation.z = position.z();
    odom_trans.transform.rotation.w = quaternion.w();
    odom_trans.transform.rotation.x = quaternion.x();
    odom_trans.transform.rotation.y = quaternion.y();
    odom_trans.transform.rotation.z = quaternion.z();

    // send the transform
    // odom_broadcaster.sendTransform(odom_trans);
    ROS_INFO("Published the transform");
}

void NavigationNode::PublishData(ros::Time &current_time)
{
    nav_msgs::Odometry odometry_msg;
    nav_msgs::Odometry local_msg;

    odometry_msg.header.frame_id = "world";
    odometry_msg.child_frame_id = "anahita/base_link";
    odometry_msg.header.stamp = current_time;

    local_msg.header.frame_id = "world";
    local_msg.child_frame_id = "anahita/base_link";
    local_msg.header.stamp = current_time;

    FillPoseMsg (poseEstimation_, quaternion_, odometry_msg);
    FillTwistMsg (velocity_, angularVelocity_, odometry_msg);

    FillPoseMsg (localPoseEstimation_, local_orientation_, local_msg);
    FillTwistMsg (velocity_, angularVelocity_, local_msg);

    navigationOdomPublisher_.publish(odometry_msg);
    odom_pub_.publish(local_msg);
}

void NavigationNode::FillPoseMsg(Eigen::Vector3d &position,
                                 Eigen::Quaterniond &quaternion,
                                 nav_msgs::Odometry &msg)
{
    msg.pose.pose.position.x = position.x();
    msg.pose.pose.position.y = position.y();
    msg.pose.pose.position.z = position.z();
    msg.pose.pose.orientation.x = quaternion.x();
    msg.pose.pose.orientation.y = quaternion.y();
    msg.pose.pose.orientation.z = quaternion.z();
    msg.pose.pose.orientation.w = quaternion.w();
}

void NavigationNode::FillTwistMsg(Eigen::Vector3d &linear_velocity, Eigen::Vector3d &angular_velocity, nav_msgs::Odometry &msg)
{
    msg.twist.twist.linear.x = linear_velocity.x();
    msg.twist.twist.linear.y = linear_velocity.y();
    msg.twist.twist.linear.z = linear_velocity.z();
    msg.twist.twist.angular.x = angular_velocity.x();
    msg.twist.twist.angular.y = angular_velocity.y();
    msg.twist.twist.angular.z = angular_velocity.z();
}

} // namespace navigation
