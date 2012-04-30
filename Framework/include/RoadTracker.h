/*
 *   Jane_RoadTracker.h
 *
 *   Author: ROBOTIS
 *
 */

#ifndef _ROAD_TRACKER_H_
#define _ROAD_TRACKER_H_

#include <string.h>

#include "Point.h"
#include "minIni.h"

namespace Robot
{
	class RoadTracker
	{
	private:
		int NoBallCount;
		static const int NoBallMaxCount = 15;

	public:
        Point2D     ball_position;

		RoadTracker();
		~RoadTracker();

		void Process(Point2D pos);
	};
}

#endif
