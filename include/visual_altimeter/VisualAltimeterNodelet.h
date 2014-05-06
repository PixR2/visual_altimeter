#ifndef VISUAL_ALTIMETER_NODELET_H
#define VISUAL_ALTIMETER_NODELET_H

#include <nodelet/nodelet.h>
#include <visual_altimeter/VisualAltimeter.h>

namespace visual_altimeter
{

	class VisualAltimeterNodelet: public nodelet::Nodelet
	{
	public:
        ~VisualAltimeterNodelet();        

		virtual void onInit();

	private:
		VisualAltimeter* altimeter;
	};
}

#endif
