#include <visual_altimeter/BasicVisualAltimeter.h>
#include "visual_altimeter/VisualHeightV1.h"

BasicVisualAltimeter::BasicVisualAltimeter(const int sample_radius, bool use_kalman_filter): 
    VisualAltimeter(0), sample_radius_(sample_radius), kalman_filter(3, 1, 0), use_kalman_filter_(use_kalman_filter)
{
    if(use_kalman_filter_ == true)
    {
        kalman_filter.statePre.at<float>(0) = 0.5f;
        kalman_filter.statePre.at<float>(1) = 0.0f;
        kalman_filter.statePre.at<float>(2) = 0.0f;

        kalman_filter.transitionMatrix = *(cv::Mat_<float>(3, 3) << 1,1,0.5, 0,1,1, 0,0,1);

        cv::setIdentity(kalman_filter.measurementMatrix);
        cv::setIdentity(kalman_filter.processNoiseCov, cv::Scalar::all(1e-4));
        cv::setIdentity(kalman_filter.measurementNoiseCov, cv::Scalar::all(1e-1));
        cv::setIdentity(kalman_filter.errorCovPost, cv::Scalar::all(.1));
    }
}

void BasicVisualAltimeter::setupResources()
{
    visual_height_pub_ = nh_.advertise<visual_altimeter::VisualHeightV1>("altimeter/height", 10);
}

void BasicVisualAltimeter::calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg)
{
    int c_x = depth_image.cols/2;
    int c_y = depth_image.rows/2;

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

    float velocity = 0.0f;

    if(use_kalman_filter_ == true)
    {
        cv::Mat prediction = kalman_filter.predict();
        //ROS_INFO("prediction: %f\n", prediction.at<float>(0);
        cv::Mat_<float> measurement(1,1);
        measurement(0) = avg_depth;
        
        cv::Mat estimated = kalman_filter.correct(measurement);
        avg_depth = estimated.at<float>(0);

        velocity = kalman_filter.statePost.at<float>(1);
    }     

    visual_altimeter::VisualHeightV1 height_msg;
    height_msg.height = avg_depth;
    height_msg.z_velocity = velocity;
    height_msg.nr_valid_samples = (int)samples_considered;
    height_msg.variance = variance;
    visual_height_pub_.publish(height_msg);
}

void BasicVisualAltimeter::calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg, const sensor_msgs::ImuConstPtr& imuMsg)
{
    return;
}
