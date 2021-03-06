#include <visual_altimeter/VisualAltimeterNodelet.h>
#include <pluginlib/class_list_macros.h>

#include <visual_altimeter/BasicVisualAltimeter.h>
#include <visual_altimeter/AdvancedVisualAltimeter.h>
#include <visual_altimeter/BasicVisualPlaneAltimeter.h>
#include <visual_altimeter/RotationInvariantAltimeter.h>

PLUGINLIB_DECLARE_CLASS(visual_altimeter, VisualAltimeterNodelet, visual_altimeter::VisualAltimeterNodelet, nodelet::Nodelet)

namespace visual_altimeter
{
    VisualAltimeterNodelet::VisualAltimeterNodelet(): altimeter(0)
    {

    }

    VisualAltimeterNodelet::~VisualAltimeterNodelet()
    {
        if(altimeter) delete altimeter;
    }

	void VisualAltimeterNodelet::onInit()
	{
        ROS_INFO("Initializing VisualAltimeterNodelet...");

		if(altimeter) delete altimeter;
        	altimeter = new RotationInvariantAltimeter(true); //new BasicVisualAltimeter(1, true);
		try
		{
			altimeter->init(getMTNodeHandle(), getMTPrivateNodeHandle());
		}
		catch(std::exception& e)
		{
		    ROS_ERROR("%s", e.what());
		    return;
		}
		
		ROS_INFO("Initializing VisualAltimeterNodelet done.");
	}
}

