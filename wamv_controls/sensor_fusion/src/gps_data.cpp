#include <ros/ros.h>
#include <gps_data.h>

namespace navigation{

    GPSData::GPSData(IntegrationMethodType integrationMethodType) :
            last_timestamp_(ros::Time::now())
    {
        positionIncrement_        = Eigen::Vector3d::Zero();
        historyPositionIncrement_ = Eigen::MatrixXd::Zero(3, 4);

        switch (integrationMethodType)
        {
            case StdMethod :
                integrationMethod_ = &GPSData::StdIntegrationMethod;
                break;
            case RKMethod  :
                integrationMethod_ = &GPSData::RKIntegrationMethod;
                break;
            default :
                integrationMethod_ = &GPSData::StdIntegrationMethod;
                break;
        }
    }

    void GPSData::GPSTwistCallback(geometry_msgs::Vector3 msg)
    {
        gps_twist_ = msg;
        SetNewDataReady();
    }

    Eigen::Vector3d GPSData::GetPositionXYZ()
    {
        ros::Duration dt = ros::Time::now() - last_timestamp_;
        double dt_sec = dt.toSec();

        (this->*integrationMethod_)(dt_sec);

        last_timestamp_ = ros::Time::now();

        return positionIncrement_;
    }

    void GPSData::StdIntegrationMethod(const double &dt_sec)
    {
        positionIncrement_ << gps_twist_.x * dt_sec, gps_twist_.y * dt_sec, gps_twist_.z * dt_sec;
    }

    void GPSData::RKIntegrationMethod(const double &dt_sec)
    {
        positionIncrement_ << gps_twist_.x * dt_sec, gps_twist_.y * dt_sec, gps_twist_.z * dt_sec;
        historyPositionIncrement_.block<3,3>(0, 1) = historyPositionIncrement_.block<3,3>(0, 0);
        historyPositionIncrement_.col(0) = positionIncrement_;
        positionIncrement_ = (1.0 / 6.0) * (historyPositionIncrement_.col(0) + 2 * historyPositionIncrement_.col(1) +
                                2 * historyPositionIncrement_.col(2) + historyPositionIncrement_.col(3));
    }

    Eigen::Vector3d GPSData::GetVelocityXYZ()
    {
        Eigen::Vector3d twist;
        twist << gps_twist_.x, gps_twist_.y, gps_twist_.z;
        return twist;
    }
}
