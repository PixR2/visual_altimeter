#include <visual_altimeter/DataFusionAltimeter.h>
#include "visual_altimeter/VisualHeightV3.h"
#include <Eigen/Geometry>
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>

DataFusionAltimeter::DataFusionAltimeter(): VisualAltimeter(false)
{

}

void DataFusionAltimeter::setupResources()
{
    visual_height_pub_ = nh_.advertise<visual_altimeter::VisualHeightV3>("altimeter/height", 10);
}

void DataFusionAltimeter::calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg)
{
    ROS_INFO("Callback...");
}

void DataFusionAltimeter::calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg, const sensor_msgs::ImuConstPtr& imuMsg)
{
    tf::Quaternion q;
    tf::quaternionMsgToTF (imuMsg->orientation, q);

    tf::Vector3 lot(0, 0, 1);
    tf::Vector3 rotated_lot = tf::quatRotate(q, lot);

    ROS_INFO("rotated_lot(%f, %f, %f)", rotated_lot.x(), rotated_lot.y(), rotated_lot.z());

    double u = 525.0*rotated_lot.x()/rotated_lot.z() + 319.5;
    double v = 525.0*rotated_lot.y()/rotated_lot.z() + 239.5;

    ROS_INFO("uv(%f, %f)", u, v);

    //double roll, pitch, yaw;
    //tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    //ROS_INFO("%f, %f, %f", roll, pitch, yaw);
    //ROS_INFO("rosq(%f, %f, %f, %f)", imuMsg->orientation.x, imuMsg->orientation.y, imuMsg->orientation.z, imuMsg->orientation.w);
}
