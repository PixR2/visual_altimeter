#include <visual_altimeter/VisualAltimeter.h>

void VisualAltimeter::cameraCallback(const sensor_msgs::ImageConstPtr& imageDepthMsg,
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

VisualAltimeter::VisualAltimeter(bool needs_imu_data): needs_imu_data_(needs_imu_data)
{

}

VisualAltimeter::~VisualAltimeter()
{
    if(camera_sync_) delete camera_sync_;
}

void VisualAltimeter::init(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;

    std::string camera_ns = nh_.resolveName("camera");
    std::string depth_topic = ros::names::clean(camera_ns + "/depth/image");
    std::string info_topic = ros::names::clean(camera_ns + "/rgb/camera_info");

    image_transport::ImageTransport it(nh_);

    try
    {
        imageDepthSub_.subscribe(it, depth_topic, 1, image_transport::TransportHints("compressedDepth"));
        cameraInfoSub_.subscribe(nh_, info_topic, 10);
    }
    catch(std::exception& e)
    {
        throw e;
    }

    if(!needs_imu_data_)
    {
        camera_sync_ = new message_filters::Synchronizer<CameraSyncPolicy>(CameraSyncPolicy(10), imageDepthSub_, cameraInfoSub_);
        camera_sync_->registerCallback(boost::bind(&VisualAltimeter::cameraCallback, this, _1, _2));
    }
    else
    {
        std::string mikrokopter_ns = nh_.resolveName("mikrokopter");
        std::string imu_topic = ros::names::clean(mikrokopter_ns + "/imu");
        ROS_INFO("imu topic: %s", imu_topic.c_str());

        try
        {
            imuSub_.subscribe(nh_, imu_topic, 10);
        }
        catch(std::exception& e)
        {
            throw e;
        }

        camera_imu_sync_ = new message_filters::Synchronizer<CameraImuSyncPolicy>(CameraImuSyncPolicy(10), imageDepthSub_, cameraInfoSub_, imuSub_);
        camera_imu_sync_->registerCallback(boost::bind(&VisualAltimeter::cameraImuCallback, this, _1, _2, _3));
    }

    setupResources();
}

void VisualAltimeter::run()
{
    ros::spin();
}
