/*
 *   RoadTracker.cpp
 *
 *   Author: Jane
 *
 */

#include <math.h>
#include "Head.h"
#include "Camera.h"
#include "ImgProcess.h"
#include "RoadTracker.h"

using namespace Robot;


RoadTracker::RoadTracker() :
        ball_position(Point2D(-1.0, -1.0))
{
	NoBallCount = 0;
}

RoadTracker::~RoadTracker()
{
}

void RoadTracker::Process(Point2D pos)
{
	if(pos.X < 0 || pos.Y < 0)
	{
		ball_position.X = -1;
		ball_position.Y = -1;
		if(NoBallCount < NoBallMaxCount)
		{
			Head::GetInstance()->MoveTracking();
			NoBallCount++;
		}
		else
			Head::GetInstance()->InitTracking();
	}
	else  
	{    // road found 
		NoBallCount = 0;
		Point2D center = Point2D(Camera::WIDTH/2, Camera::HEIGHT/2);
		Point2D offset = pos - center;
		offset *= -1; // Inverse X-axis, Y-axis
		offset.X *= (Camera::VIEW_H_ANGLE / (double)Camera::WIDTH); // pixel per angle
		offset.Y *= (Camera::VIEW_V_ANGLE / (double)Camera::HEIGHT); // pixel per angle
		ball_position = offset;
		Head::GetInstance()->MoveTracking(ball_position);
	}
}
