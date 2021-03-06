#ifndef CAMERA_ROTATION_CALIBRATOR_NODELET_H
#define CAMERA_ROTATION_CALIBRATOR_NODELET_H

#include <nodelet/nodelet.h>
#include <visual_altimeter/CameraRotationCalibrator.h>

namespace visual_altimeter
{
	class CameraRotationCalibratorNodelet: public nodelet::Nodelet
	{
	public:
        CameraRotationCalibratorNodelet();
        ~CameraRotationCalibratorNodelet();        

		virtual void onInit();

	private:
		CameraRotationCalibrator* calibrator;
	};
}

#endif
