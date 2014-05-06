#include <visual_altimeter/BasicVisualAltimeter.h>
#include "visual_altimeter/VisualHeightV1.h"

BasicVisualAltimeter::BasicVisualAltimeter(const int sample_radius): VisualAltimeter(false), sample_radius_(sample_radius)
{

}

void BasicVisualAltimeter::setupResources()
{
    visual_height_pub_ = nh_.advertise<visual_altimeter::VisualHeightV1>("altimeter/height", 10);
}

void BasicVisualAltimeter::calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg)
{
    int c_x = depth_image.cols/2;
    int c_y = depth_image.rows/2;

    /*c_x = (c_x - sample_radius) < 0 ?
            sample_radius : (c_x + sample_radius) >= ptrDepth->image.cols ?
                ptrDepth->image.cols - sample_radius : c_x;


    c_y = (c_y - sample_radius) < 0 ?
            sample_radius : (c_y + sample_radius) >= ptrDepth->image.rows ?
                ptrDepth->image.rows - sample_radius : c_y;*/

    float c_depth = depth_image.at<float>(c_y, c_x);

    float avg_depth = 0.0f;
    float avg_depth_squared = 0.0f;
    float samples_considered = 0.0f;
    for(int y = c_y - sample_radius_; y <= c_y + sample_radius_; y++)
    {
        for(int x = c_x - sample_radius_; x <= c_x + sample_radius_; x++)
        {
            float depth = depth_image.at<float>(y, x);
            if(depth > 0.0f)
            {
                samples_considered += 1.0f;
                avg_depth += depth;
                avg_depth_squared += depth*depth;
            }
        }
    }

    avg_depth /= samples_considered;
    avg_depth_squared /= samples_considered;

    float variance = sqrt(avg_depth_squared - avg_depth*avg_depth);

    visual_altimeter::VisualHeightV1 height_msg;
    height_msg.height = avg_depth;
    height_msg.nr_valid_samples = (int)samples_considered;
    height_msg.variance = variance;
    visual_height_pub_.publish(height_msg);

    //ROS_INFO("Depth: depth(%i, %i) = %f - %f (%f)", c_x, c_y, c_depth, avg_depth, samples_considered);

    /*double min;
    double max;
    cv::minMaxIdx(ptrDepth->image, &min, &max);

    ROS_INFO("[%f, %f]", min, max);

    cv::Mat adjMap;
    ptrDepth->image.convertTo(adjMap,CV_8UC1, 255 / (max-min), -min);

    cv::Mat falseColorsMap;
    cv::applyColorMap(adjMap, falseColorsMap, cv::COLORMAP_BONE);

    cv::imshow("Depth", falseColorsMap);

    cv::waitKey(1);*/
}

void BasicVisualAltimeter::calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg, const sensor_msgs::ImuConstPtr& imuMsg)
{
    return;
}
