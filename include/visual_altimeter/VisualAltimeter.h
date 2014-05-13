#ifndef VISUAL_ALTIMETER_H
#define VISUAL_ALTIMETER_H

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <sensor_msgs/Imu.h>

#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>

class VisualAltimeter
{
private:
    int subscribtion_mode_;

    image_transport::SubscriberFilter imageDepthSubFilter_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub_;
    message_filters::Subscriber<sensor_msgs::Imu> imuSub_;
    image_transport::Subscriber imageDepthSub_;

    typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::Image,
            sensor_msgs::CameraInfo> CameraSyncPolicy;
    message_filters::Synchronizer<CameraSyncPolicy> *camera_sync_;

    typedef message_filters::sync_policies::ApproximateTime<
            sensor_msgs::Image,
            sensor_msgs::CameraInfo,
            sensor_msgs::Imu> CameraImuSyncPolicy;
    message_filters::Synchronizer<CameraImuSyncPolicy> *camera_imu_sync_;

    message_filters::Cache<sensor_msgs::CameraInfo>* cam_info_cache_;
    message_filters::Cache<sensor_msgs::Imu>* imu_cache_;
protected:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

private:
    void cameraCallback(const sensor_msgs::ImageConstPtr& imageDepthMsg);

    void cameraInfoCallback(const sensor_msgs::ImageConstPtr& imageDepthMsg,
                        const sensor_msgs::CameraInfoConstPtr& camInfoMsg);

    void cameraImuCallback(const sensor_msgs::ImageConstPtr& imageDepthMsg,
                            const sensor_msgs::CameraInfoConstPtr& camInfoMsg,
                            const sensor_msgs::ImuConstPtr& imuMsg);

public:
    VisualAltimeter(int subscibtion_mode);
    ~VisualAltimeter();
    void init(ros::NodeHandle nh, ros::NodeHandle pnh);
    void run();

    virtual void setupResources() = 0;
    virtual void calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg) = 0;
    virtual void calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg, const sensor_msgs::ImuConstPtr& imuMsg) = 0;
};

#endif
