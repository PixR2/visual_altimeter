#ifndef DATA_FUSION_ALTIMETER_H
#define DATA_FUSION_ALTIMETER_H

#include <visual_altimeter/VisualAltimeter.h>

class DataFusionAltimeter: public VisualAltimeter
{
public:
    DataFusionAltimeter();

    void setupResources();
    void calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg);
    void calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg, const sensor_msgs::ImuConstPtr& imuMsg);

private:
    ros::Publisher visual_height_pub_;
};

#endif
