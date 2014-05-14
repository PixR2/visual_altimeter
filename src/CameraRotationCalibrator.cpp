#include <visual_altimeter/CameraRotationCalibrator.h>

#include <opencv2/highgui/highgui.hpp>
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>

CameraRotationCalibrator::~CameraRotationCalibrator()
{
    if(cam_info_cache_) delete cam_info_cache_;
    if(imu_cache_) delete imu_cache_;
}

void CameraRotationCalibrator::init(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    std::string camera_ns = nh.resolveName("");
    std::string rgb_topic = ros::names::clean(camera_ns + "/rgb/image_color");
    std::string info_topic = ros::names::clean(camera_ns + "/rgb/camera_info");

    ROS_INFO("rgb topic: %s\ncamera info topic: %s\n", rgb_topic.c_str(), info_topic.c_str());

    image_transport::ImageTransport it(nh);

    std::string imu_topic = "/mikrokopter/imu";
    ROS_INFO("imu topic: %s", imu_topic.c_str());

    try
    {
        cameraInfoSub_.subscribe(nh, info_topic, 10);
        imuSub_.subscribe(nh, imu_topic, 10);
    }
    catch(std::exception& e)
    {
        throw e;
    }

    cam_info_cache_ = new message_filters::Cache<sensor_msgs::CameraInfo>(cameraInfoSub_, 3);
    imu_cache_ = new message_filters::Cache<sensor_msgs::Imu>(imuSub_, 3);
    imageSub_ = it.subscribe(rgb_topic, 1, boost::bind(&CameraRotationCalibrator::cameraCallback, this, _1));

    cv::namedWindow("camera_rotation_calibration");
    cv::createTrackbar("rotation_trackbar", "camera_rotation_calibration", 0, 360000);
}

void CameraRotationCalibrator::cameraCallback(const sensor_msgs::ImageConstPtr& imageMsg)
{
    cv_bridge::CvImageConstPtr ptrImage;

    try
    {
        ptrImage = cv_bridge::toCvCopy(imageMsg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    const sensor_msgs::CameraInfoConstPtr camInfoMsg = cam_info_cache_->getElemBeforeTime(ros::Time::now());
    const sensor_msgs::ImuConstPtr imuMsg = imu_cache_->getElemBeforeTime(ros::Time::now());
    cv::Mat image = ptrImage->image;

    if(camInfoMsg != 0 && imuMsg != 0)
    {
        // TODO: Do stuff...
        cv::Point2f c(camInfoMsg->K[2], camInfoMsg->K[5]);
        cv::Point2f f(camInfoMsg->K[0], camInfoMsg->K[4]);
        ROS_INFO("c: %f, %f f: %f, %f", c.x, c.y, f.x, f.y);


        tf::Quaternion q;
        tf::quaternionMsgToTF (imuMsg->orientation, q);

        double angle =  (double)cv::getTrackbarPos("rotation_trackbar", "camera_rotation_calibration")/1000.0;
        ROS_INFO("angle: %f", angle);

        tf::Quaternion cal;
        cal.setRotation(tf::Vector3(0, 0, 1), angle*0.0174532925);

        tf::Vector3 lot(0, 0, 1);
        tf::Vector3 rotated_lot = tf::quatRotate(cal, tf::quatRotate(q, lot));

        //ROS_INFO("rotated_lot(%f, %f, %f)", rotated_lot.x(), rotated_lot.y(), rotated_lot.z());

        double u = f.x*rotated_lot.x()/rotated_lot.z() + c.x;
        double v = f.y*rotated_lot.y()/rotated_lot.z() + c.y;

        ROS_INFO("uv(%f, %f)", u, v);
        cv::circle(image, cv::Point((int)(u), (int)(v)), 12, cv::Scalar(255), 4);
        cv::circle(image, cv::Point((int)(u), (int)(v)), 8, cv::Scalar(0), 4);
        cv::circle(image, cv::Point((int)(u), (int)(v)), 4, cv::Scalar(255), 4);
        cv::imshow("camera_rotation_calibration", image);
        cv::waitKey(10);
    }
}
