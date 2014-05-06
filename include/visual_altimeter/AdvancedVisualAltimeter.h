#ifndef ADVANCED_VISUAL_ALTIMETER_H
#define ADVANCED_VISUAL_ALTIMETER_H

#include <visual_altimeter/VisualAltimeter.h>

class AdvancedVisualAltimeter: public VisualAltimeter
{
public:
    AdvancedVisualAltimeter();

    void setupResources();
    void calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg);
    void calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg, const sensor_msgs::ImuConstPtr& imuMsg);

private:
    ros::Publisher visual_height_pub_;
};

#endif