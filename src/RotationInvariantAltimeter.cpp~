#include <visual_altimeter/RotationInvariantAltimeter.h>
#include "visual_altimeter/VisualHeightV3.h"
#include <Eigen/Geometry>
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <opencv2/highgui/highgui.hpp>

RotationInvariantAltimeter::RotationInvariantAltimeter(): VisualAltimeter(2)
{

}

void RotationInvariantAltimeter::setupResources()
{
    visual_height_pub_ = nh_.advertise<visual_altimeter::VisualHeightV3>("altimeter/height", 10);
}

void RotationInvariantAltimeter::calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg)
{
    ROS_INFO("Callback...");
}

void RotationInvariantAltimeter::calculateHeight(const cv::Mat& depth_image, const sensor_msgs::CameraInfoConstPtr& camInfoMsg, const sensor_msgs::ImuConstPtr& imuMsg)
{
    // Extract center point and focal length.
    cv::Point2f c(camInfoMsg->K[2], camInfoMsg->K[5]);
    cv::Point2f f(camInfoMsg->K[0], camInfoMsg->K[4]);

    // Convert from message to tf::Quaternion.
    tf::Quaternion q;
    tf::quaternionMsgToTF (imuMsg->orientation, q);

    // Create rotation (quaternion) around z-axis to compensate for image not aligning to FlightCtrl. (135 degrees)
    tf::Quaternion cal;
    cal.setRotation(tf::Vector3(0, 0, 1), (3.0*M_PI)/4.0);

    // Rotate lot to compensate for rotation of UAV. (Find the actual lot)
    tf::Vector3 lot(0, 0, 1);
    tf::Vector3 rotated_lot = tf::quatRotate(cal, tf::quatRotate(q, lot));

    ROS_INFO("rotated_lot(%f, %f, %f)", rotated_lot.x(), rotated_lot.y(), rotated_lot.z());

    // Project the found, real, lot onto the camera plane to find the pixel which lies on the line from the camera hole through the lot.
    double u = f.x*rotated_lot.x()/rotated_lot.z() + c.x;
    double v = f.y*rotated_lot.y()/rotated_lot.z() + c.y;

    //TODO: Use Kalmanfilter to filter height and use KF internal state to find rate of height change (i.e. velocity in z-direction.).
    //TODO: Use KF prediction step to predict and publish likely next height. 

    // Publish the height.
    visual_altimeter::VisualHeightV3 height_msg;
    height_msg.height = depth_image.at<float>(v, u);
    visual_height_pub_.publish(height_msg);

    /*ROS_INFO("uv(%f, %f)", u, v);
    cv::Mat tmp = depth_image.clone();
    cv::circle(tmp, cv::Point((int)(u), (int)(v)), 12, cv::Scalar(255), 4);
    cv::circle(tmp, cv::Point((int)(u), (int)(v)), 8, cv::Scalar(0), 4);
    cv::circle(tmp, cv::Point((int)(u), (int)(v)), 4, cv::Scalar(255), 4);
    cv::imshow("lot_image", tmp);
    cv::waitKey(10);*/
    //double roll, pitch, yaw;
    //tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    //ROS_INFO("%f, %f, %f", roll, pitch, yaw);
    //ROS_INFO("rosq(%f, %f, %f, %f)", imuMsg->orientation.x, imuMsg->orientation.y, imuMsg->orientation.z, imuMsg->orientation.w);
}
