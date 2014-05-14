#include <visual_altimeter/CameraRotationCalibrator.h>

CameraRotationCalibrator::~CameraRotationCalibrator()
{
    if(cam_info_cache_) delete cam_info_cache_;
    if(imu_cache_) delete imu_cache_;
}

void CameraRotationCalibrator::init(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    std::string camera_ns = nh.resolveName("");
    std::string rgb_topic = ros::names::clean(camera_ns + "/rgb/image");
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
}

void CameraRotationCalibrator::cameraCallback(const sensor_msgs::ImageConstPtr& imageDepthMsg)
{
    cv_bridge::CvImageConstPtr ptrDepth;

    try
    {
        ptrDepth = cv_bridge::toCvCopy(imageDepthMsg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    const sensor_msgs::CameraInfoConstPtr cam_info_msg = cam_info_cache_->getElemBeforeTime(ros::Time::now());
    const sensor_msgs::ImuConstPtr imu_msg = imu_cache_->getElemBeforeTime(ros::Time::now());

    if(cam_info_msg != 0 && imu_msg != 0)
    {
        // TODO: Do stuff...
    }
}
