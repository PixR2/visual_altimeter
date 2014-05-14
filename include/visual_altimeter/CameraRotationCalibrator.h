#ifndef CAMERA_ROTATION_CALIBRATOR_H
#define CAMERA_ROTATION_CALIBRATOR_H

#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Imu.h>

#include <message_filters/subscriber.h>
#include <message_filters/cache.h>

class CameraRotationCalibrator
{
private:
    message_filters::Cache<sensor_msgs::CameraInfo>* cam_info_cache_;
    message_filters::Cache<sensor_msgs::Imu>* imu_cache_;

    message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub_;
    message_filters::Subscriber<sensor_msgs::Imu> imuSub_;
    image_transport::Subscriber imageSub_;

public:
    ~CameraRotationCalibrator();

    void init(ros::NodeHandle nh, ros::NodeHandle pnh);

private:
    void cameraCallback(const sensor_msgs::ImageConstPtr& imageDepthMsg);
};

#endif
