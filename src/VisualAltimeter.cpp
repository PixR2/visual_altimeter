#include <visual_altimeter/VisualAltimeter.h>

void VisualAltimeter::cameraCallback(const sensor_msgs::ImageConstPtr& imageDepthMsg)
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
        calculateHeight(ptrDepth->image, cam_info_msg, imu_msg);
    }
}

void VisualAltimeter::cameraInfoCallback(const sensor_msgs::ImageConstPtr& imageDepthMsg,
                    const sensor_msgs::CameraInfoConstPtr& camInfoMsg)
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

    calculateHeight(ptrDepth->image, camInfoMsg);
}

void VisualAltimeter::cameraImuCallback(const sensor_msgs::ImageConstPtr& imageDepthMsg,
                        const sensor_msgs::CameraInfoConstPtr& camInfoMsg,
                        const sensor_msgs::ImuConstPtr& imuMsg)
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
    ROS_INFO("IMU-Callback...");
    calculateHeight(ptrDepth->image, camInfoMsg, imuMsg);
}

VisualAltimeter::VisualAltimeter(int subscribtion_mode): subscribtion_mode_(subscribtion_mode), cam_info_cache_(0), imu_cache_(0)
{

}

VisualAltimeter::~VisualAltimeter()
{
    if(camera_sync_) delete camera_sync_;
    if(cam_info_cache_) delete cam_info_cache_;
    if(imu_cache_) delete imu_cache_;
}

void VisualAltimeter::init(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;

    std::string camera_ns = nh_.resolveName("");
    std::string depth_topic = ros::names::clean(camera_ns + "/depth/image");
    std::string info_topic = ros::names::clean(camera_ns + "/rgb/camera_info");

    ROS_INFO("depth topic: %s\ncamera info topic: %s\n", depth_topic.c_str(), info_topic.c_str());

    image_transport::ImageTransport it(nh_);

    try
    {
        imageDepthSubFilter_.subscribe(it, depth_topic, 1, image_transport::TransportHints("raw"));
        cameraInfoSub_.subscribe(nh_, info_topic, 10);
    }
    catch(std::exception& e)
    {
        throw e;
    }

    if(subscribtion_mode_ == 0)
    {
        camera_sync_ = new message_filters::Synchronizer<CameraSyncPolicy>(CameraSyncPolicy(10), imageDepthSubFilter_, cameraInfoSub_);
        camera_sync_->registerCallback(boost::bind(&VisualAltimeter::cameraInfoCallback, this, _1, _2));
    }
    else
    {
        //std::string mikrokopter_ns = nh_.resolveName("mikrokopter");
        //std::string imu_topic = ros::names::clean(mikrokopter_ns + "/imu");
        std::string imu_topic = "/mikrokopter/imu";
        ROS_INFO("imu topic: %s", imu_topic.c_str());

        try
        {
            imuSub_.subscribe(nh_, imu_topic, 10);
        }
        catch(std::exception& e)
        {
            throw e;
        }

        if(subscribtion_mode_ == 1)
        {
            camera_imu_sync_ = new message_filters::Synchronizer<CameraImuSyncPolicy>(CameraImuSyncPolicy(10), imageDepthSubFilter_, cameraInfoSub_, imuSub_);
            camera_imu_sync_->registerCallback(boost::bind(&VisualAltimeter::cameraImuCallback, this, _1, _2, _3));
        }
        else
        {
            cam_info_cache_ = new message_filters::Cache<sensor_msgs::CameraInfo>(cameraInfoSub_, 3);
            imu_cache_ = new message_filters::Cache<sensor_msgs::Imu>(imuSub_, 3);
            imageDepthSub_ = it.subscribe(depth_topic, 1, boost::bind(&VisualAltimeter::cameraCallback, this, _1));
        }

    }

    setupResources();
}

void VisualAltimeter::run()
{
    ros::spin();
}
