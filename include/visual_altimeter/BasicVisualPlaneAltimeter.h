#ifndef BASIC_VISUAL_PLANE_ALTIMETER_H
#define BASIC_VISUAL_PLANE_ALTIMETER_H

#include <visual_altimeter/VisualAltimeter.h>
#include <Eigen/Geometry>

class BasicVisualPlaneAltimeter: public VisualAltimeter
{
public:
    BasicVisualPlaneAltimeter();

    void setupResources();
    void calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg);
    void calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg, const sensor_msgs::ImuConstPtr& imuMsg);

private:
    std_msgs::Header header_;

    ros::Publisher plane_points_pub_;
    ros::Publisher plane_points_transformed_pub_;

    Eigen::ParametrizedLine<double, 3> lot_;
};

#endif
