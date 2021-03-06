#include <visual_altimeter/CameraRotationCalibratorNodelet.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(visual_altimeter, CameraRotationCalibratorNodelet, visual_altimeter::CameraRotationCalibratorNodelet, nodelet::Nodelet)

namespace visual_altimeter
{
    CameraRotationCalibratorNodelet::CameraRotationCalibratorNodelet(): calibrator(0)
    {

    }

    CameraRotationCalibratorNodelet::~CameraRotationCalibratorNodelet()
    {
        if(calibrator) delete calibrator;
    }

	void CameraRotationCalibratorNodelet::onInit()
	{
        ROS_INFO("Initializing CameraRotationCalibratorNodelet...");

        if(calibrator) delete calibrator;
        calibrator = new CameraRotationCalibrator();

		try
		{
			calibrator->init(getMTNodeHandle(), getMTPrivateNodeHandle());
		}
		catch(std::exception& e)
		{
		    ROS_ERROR("%s", e.what());
		    return;
		}
		
		ROS_INFO("Initializing CameraRotationCalibratorNodelet done.");
	}
}
