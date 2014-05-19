#ifndef BASIC_VISUAL_ALTIMETER_H
#define BASIC_VISUAL_ALTIMETER_H

#include <visual_altimeter/VisualAltimeter.h>
#include <opencv2/video/tracking.hpp>

class BasicVisualAltimeter: public VisualAltimeter
{
public:
    BasicVisualAltimeter(const int sample_radius = 4, bool use_kalman_filter = true);

    void setupResources();
    void calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg);
    void calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg, const sensor_msgs::ImuConstPtr& imuMsg);

private:
    int sample_radius_;

    ros::Publisher visual_height_pub_;

    cv::KalmanFilter kalman_filter;
    bool use_kalman_filter_;
};

#endif
