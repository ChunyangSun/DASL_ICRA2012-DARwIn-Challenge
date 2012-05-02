/*
 *   Jane_Steer.cpp
 *   modify from ballfollower.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <stdio.h>
#include "ImgProcess.h"
#include <stdlib.h>
#include "MX28.h"
#include "Head.h"
#include "Action.h"
#include "Walking.h"
#include "Steer.h"
#include "Camera.h"
#include "MotionStatus.h"
#include "ColorFinder.h"

using namespace Robot;


Steer::Steer() 
{
	DEBUG_PRINT = false;

	k = 1;      //change from 4
	rotation = 0.0;
	old_rotation = 0.0;
	rotation_diff = 0;
	width = Camera::WIDTH;
	height = Camera::HEIGHT;	
	scanLine1 = 80;
	scanLine2 = height - 80;
	scanLine1Center = 0;
	scanLine2Center = 0;
	begp1 = -1; 
	endp1 = -1;
	begp2 = -1; 
	endp2 = -1;
	i = 0;
	j = 0;
	m = 0.1; //filter const for rotation
	offset = width/2;
	m_left_ini = 0;
	m_right_ini = 0;
	p_gain = 2;
	d_gain = 0.22;
	RotationInput = 0;
	//LeftRotation = 0;
	//RightRotation = 0;
	currentImage = NULL;			
}

Steer::~Steer()
{
	//Do not distruct anything
}	

void Steer::Initialize()
{
	rotation = m*rotation +(1-m)*old_rotation;
	old_rotation = rotation;

	m_left_ini = MotionStatus::m_CurrentJoints.GetValue(JointData::ID_R_ELBOW);
	m_right_ini = MotionStatus::m_CurrentJoints.GetValue(JointData::ID_L_ELBOW);
	//CheckLimit();

	//InitTracking();
	//MoveToHome();
}

void Steer::Filter()
{
	double pOffset, dOffset;

	pOffset = rotation * p_gain;
//	pOffset *= pOffset;
//	if(rotation < 0)
//		pOffset = -pOffset;
	dOffset = rotation_diff * d_gain;
//	dOffset *= dOffset;
//	if(rotation_diff < 0)
//		dOffset = -dOffset;
	RotationInput = pOffset + dOffset;
	
}
void Steer::Process()
{
	Steer::Initialize();
	begp1 = -1; 
	endp1 = -1;
	begp2 = -1; 
	endp2 = -1;	

	for( i = 320*scanLine1; i < 320*(scanLine1+1); i++)                       
	{
		if (currentImage->m_ImageData[i*currentImage->m_PixelSize + 1] == 255)
		{
				if(begp1 == -1){
				begp1 = i;
				}
	
				endp1 = i;
				if((begp1 + endp1) != -1){
				scanLine1Center = (begp1 + endp1)/2 - 320*scanLine1;			
				}
		}
		
	}	
	
	for( j = 320*scanLine2; j < 320*(scanLine2+1); j++)                       
	{	
		if (currentImage->m_ImageData[j*currentImage->m_PixelSize + 1] == 255)
		{
				if(begp2 == -1)
				{
					begp2 = j;
				}
	
				endp2 = j;
				
				if((begp2 + endp2)/2 != -1)
				{
					scanLine2Center = (begp2 + endp2)/2 - 320*scanLine2;				
				}
		}
		
	}	
	
	//printf("%d\n", scanLine1Center); //0--320
	//printf("%d\n", scanLine2Center); //0--320

	
	//printf("rotation %f", rotation);//-30~30 with k = 0.2
	
	//If road is not found, don't steer
	if((scanLine1Center+scanLine2Center)<= 0)
	{	
		rotation = old_rotation;
	}
	else
	{		
		rotation = k*((scanLine1Center+scanLine2Center)/2-offset);
	}

	//Lowpass Filter calculated sensor data
	//rotation = m*rotation +(1-m)*old_rotation;
	
	rotation_diff = rotation - old_rotation;
	//old_rotation = rotation;

	
/*	
	if ((rotation - old_rotation)>500)
	{
		rotation = old_rotation + 500;	
	}
	else if((rotation - old_rotation)<-500)
	{
		rotation = old_rotation - 500;
	}
*/	
	Steer::Filter();

	//motor limit
	if (RotationInput>500)
	{
		RotationInput = 500;	
	}
	else if (RotationInput < -500)
	{
		RotationInput = -500;	
	}
	
	RotationInput = -RotationInput;
	
	
	//printf("rotation %f\n", rotation);
	

	printf("rotationinput %f\n", RotationInput); //-600~600
	if(Action::GetInstance()->m_Joint.GetEnable(JointData::ID_R_ELBOW) == true)
	Action::GetInstance()->m_Joint.SetValue(5, 2000 + RotationInput);
	if(Action::GetInstance()->m_Joint.GetEnable(JointData::ID_L_ELBOW) == true)
	Action::GetInstance()->m_Joint.SetValue(6, 2000 + RotationInput);
	
	//usleep(50);
		//fprintf(stderr, "[STEER]");
	
}
