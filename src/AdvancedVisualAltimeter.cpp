#include <visual_altimeter/AdvancedVisualAltimeter.h>
#include "visual_altimeter/VisualHeightV2.h"

AdvancedVisualAltimeter::AdvancedVisualAltimeter(): VisualAltimeter(false)
{

}

void AdvancedVisualAltimeter::setupResources()
{
    visual_height_pub_ = nh_.advertise<visual_altimeter::VisualHeightV2>("altimeter/height", 10);
}

void AdvancedVisualAltimeter::calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg)
{
    ROS_INFO("Calculateing height...");
    // Mask all invalid (<= 0.5) pixels.
    cv::Mat mask = depth_image > 0.5f;

    // Calculate mean and standart deviation.
    cv::Scalar mean;
    cv::Scalar stddev;
    cv::meanStdDev(depth_image, mean, stddev, mask);

    // Calculate smallest and largest distance.
    double minimum_depth;
    double maximum_depth;
    cv::minMaxLoc(depth_image, &minimum_depth, &maximum_depth, 0, 0, mask);

    // Count valid pixels.
    int nr_valid_pixels = cv::countNonZero(mask);

    // Publish height message.
    visual_altimeter::VisualHeightV2 height_msg;
    height_msg.height = mean[0];
    height_msg.deviation = stddev[0];
    height_msg.min = minimum_depth;
    height_msg.max = maximum_depth;
    height_msg.nr_valid_samples = nr_valid_pixels;

    visual_height_pub_.publish(height_msg);
    ROS_INFO("Calculateing height done.");
}

void AdvancedVisualAltimeter::calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg, const sensor_msgs::ImuConstPtr& imuMsg)
{

}
