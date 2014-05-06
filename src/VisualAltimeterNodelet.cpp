#include <visual_altimeter/VisualAltimeterNodelet.h>
#include <pluginlib/class_list_macros.h>

#include <visual_altimeter/BasicVisualAltimeter.h>
#include <visual_altimeter/AdvancedVisualAltimeter.h>
#include <visual_altimeter/BasicVisualPlaneAltimeter.h>
#include <visual_altimeter/DataFusionAltimeter.h>

PLUGINLIB_DECLARE_CLASS(visual_altimeter, VisualAltimeterNodelet, visual_altimeter::VisualAltimeterNodelet, nodelet::Nodelet)

namespace visual_altimeter
{
    VisualAltimeterNodelet::~VisualAltimeterNodelet()
    {
        if(altimeter) delete altimeter;
    }

	void VisualAltimeterNodelet::onInit()
	{
		ROS_INFO("Initializing VisualAltimeterNodelet...");

		if(altimeter) delete altimeter;
		altimeter = new AdvancedVisualAltimeter();
		
		try
		{
			altimeter->init(getNodeHandle(), getPrivateNodeHandle());
		}
		catch(std::exception& e)
		{
		    ROS_ERROR("%s", e.what());
		    return;
		}
		
		ROS_INFO("Initializing VisualAltimeterNodelet done.");
	}
}

