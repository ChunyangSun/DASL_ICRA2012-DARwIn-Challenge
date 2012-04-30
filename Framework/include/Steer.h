/*
 *   Jane_Steer.h
 *
 *   Author: Jane
 *
 */

#ifndef _STEER_H_
#define _STEER_H_

#include "Point.h"
#include "RoadTracker.h"


namespace Robot
{
	class Steer
	{
	private:int m_NoBallCount;
		double k;      //added
		double rotation;
		double old_rotation;
		double rotation_diff;
		int width;
		int height;
		int scanLine1;
		int scanLine2;
		int scanLine1Center;
		int scanLine2Center;
		int begp1; 
		int endp1;
		int begp2; 
		int endp2;
		int i, j;
		double m; //filter const for rotation
		int offset;
		int m_left_ini;
		int m_right_ini;
		//int m_rot_diff;
		double p_gain;
		double d_gain;
		double RotationInput;
	protected:

	public:
		bool DEBUG_PRINT;
				
		Image* currentImage;   //added
		Steer();
		~Steer();
		

		void Process();
		void Filter();
		void Initialize();
	};
}

#endif
