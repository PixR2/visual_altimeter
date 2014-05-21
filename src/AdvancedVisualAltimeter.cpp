#include <visual_altimeter/AdvancedVisualAltimeter.h>
#include "visual_altimeter/VisualHeightV2.h"

AdvancedVisualAltimeter::AdvancedVisualAltimeter(bool use_kalman_filter): 
    VisualAltimeter(0), use_kalman_filter_(use_kalman_filter), kalman_filter(3, 1, 0)
{
    if(use_kalman_filter_ == true)
    {
        kalman_filter.statePre.at<float>(0) = 0.5f;
        kalman_filter.statePre.at<float>(1) = 0.0f;
        kalman_filter.statePre.at<float>(2) = 0.0f;

        kalman_filter.transitionMatrix = *(cv::Mat_<float>(3, 3) << 1,1,0.5, 0,1,1, 0,0,1);
        kalman_filter.measurementMatrix = *(cv::Mat_<float>(1, 3) << 1,1,0.5);

        cv::setIdentity(kalman_filter.processNoiseCov, cv::Scalar::all(1e-4));
        cv::setIdentity(kalman_filter.measurementNoiseCov, cv::Scalar::all(1e-1));
        cv::setIdentity(kalman_filter.errorCovPost, cv::Scalar::all(.1));
    }
}

void AdvancedVisualAltimeter::setupResources()
{
    visual_height_pub_ = nh_.advertise<visual_altimeter::VisualHeightV2>("altimeter/height", 10);
}

void AdvancedVisualAltimeter::calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg)
{
    //ROS_INFO("Calculateing height...");
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

    // Filter the calculated value using kalmanfilter if enabled.
    float height = mean[0];
    float velocity = 0.0f;

    if(use_kalman_filter_ == true)
    {
        cv::Mat prediction = kalman_filter.predict();
        //ROS_INFO("prediction: %f\n", prediction.at<float>(0));
        cv::Mat_<float> measurement(1,1);
        measurement(0) = height;
        
        cv::Mat estimated = kalman_filter.correct(measurement);
        float estimated_depth = estimated.at<float>(0);
        printf("%f, %f\n", height, estimated_depth);
        height = estimated_depth;

        velocity = kalman_filter.statePost.at<float>(1);
    } 

    // Publish height message.
    visual_altimeter::VisualHeightV2 height_msg;
    height_msg.height = height;
    height_msg.z_velocity = velocity;
    height_msg.deviation = stddev[0];
    height_msg.min = minimum_depth;
    height_msg.max = maximum_depth;
    height_msg.nr_valid_samples = nr_valid_pixels;

    visual_height_pub_.publish(height_msg);
    //ROS_INFO("Calculateing height done.");
}

void AdvancedVisualAltimeter::calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg, const sensor_msgs::ImuConstPtr& imuMsg)
{

}
