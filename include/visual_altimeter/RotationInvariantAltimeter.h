#ifndef ROTATION_INVARIANT_ALTIMETER_H
#define ROTATION_INVARIANT_ALTIMETER_H

#include <visual_altimeter/VisualAltimeter.h>
#include <opencv2/video/tracking.hpp>

class RotationInvariantAltimeter: public VisualAltimeter
{
public:
    RotationInvariantAltimeter();

    void setupResources();
    void calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg);
    void calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg, const sensor_msgs::ImuConstPtr& imuMsg);

private:
    ros::Publisher visual_height_pub_;
};

#endif
