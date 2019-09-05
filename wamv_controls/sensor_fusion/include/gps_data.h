#pragma once

#include <eigen3/Eigen/Geometry>
#include "navigation_device.h"
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>

namespace navigation{

    class GPSData: public NavigationDevice
    {
        typedef void (GPSData::*IntegrationMethodT) (const double &);

    public:

        enum IntegrationMethodType
        {
            StdMethod  = 0,
            RKMethod,
            DefaultMethod
        };

        GPSData(IntegrationMethodType integrationMethodType = RKMethod);

        void GPSTwistCallback(geometry_msgs::Vector3 msg);

        Eigen::Vector3d GetPositionXYZ();
        Eigen::Vector3d GetVelocityXYZ();

    private:
        void StdIntegrationMethod(const double &dt_sec);
        void RKIntegrationMethod(const double &dt_sec);

        ros::Time last_timestamp_;

        Eigen::Vector3d positionIncrement_;
        Eigen::MatrixXd historyPositionIncrement_;

        geometry_msgs::Vector3 gps_twist_;

        IntegrationMethodT integrationMethod_;
    };
}
