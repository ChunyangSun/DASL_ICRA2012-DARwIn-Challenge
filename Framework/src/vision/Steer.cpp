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
#include "math.h"
#include "RoadTracker.h"
#include "Kinematics.h"
using namespace Robot;
#define PI 3.14159265

Steer::Steer() 
{
	DEBUG_PRINT = false;

	OffsetGain = 0.1;      
	AngleGain = 0.2;//0.3
	rotation = 0.0;
	old_rotation = 0.0;
	rotation_diff = 0;
	width =  Camera::WIDTH;
	height = Camera::HEIGHT;	
	scanLine1 = 80;
	scanLine2 = height - 120;
	scanLine1Center = 0;
	scanLine2Center = 0;
	begp1 = -1; 
	endp1 = -1;
	begp2 = -1; 
	endp2 = -1;
	i = 0;
	j = 0;
	m = 0.15; //filter const for rotation
	offset = width/2;
	m_left_ini = 0;
	m_right_ini = 0;
	p_gain = 2;
	//d_gain = 0.22;
	d_gain = 2;
	RotationInput = 0;
	currentImage = NULL;	
	TurnAngle = 0;	
	delY = 0;	
	delY_Old = 0;
	lspOld = -50;
	lepOld = 25;
	rspOld = 50;
	repOld = -25;		
}

Steer::~Steer()
{
	//Do not distruct anything
}	


void Steer::Initialize()
{
	//rotation = m*rotation +(1-m)*old_rotation


	m_left_ini = MotionStatus::m_CurrentJoints.GetValue(JointData::ID_R_ELBOW);
	m_right_ini = MotionStatus::m_CurrentJoints.GetValue(JointData::ID_L_ELBOW);
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

Point2D Steer::CamPosToGoalPos(double CamX, double CamY)
{
		
		int FOV_X = 60; //degree
		int FOV_Y = 45;
		int H = 450;    //height seat to ground: 210mm cam to seat:240mm
		double Pan = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_PAN);  //horizontal, middle is 0, rightside is "-"
		
		double Tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);//vertical

		printf("Pan %f", Pan); 
		printf("Tilt %f \n\n", Tilt);
		
		double GoalAngleY = (CamY)/(double)height*FOV_Y + Tilt + 15;  //Kinematics.h 
		GoalPos.Y = H*tan(GoalAngleY/180*PI);
		double GoalAngleX = (CamX - (double) Camera::WIDTH/2)/(double)width*FOV_X + (-Pan); //horizontal, middle is 0, rightside is "+"
		GoalPos.X = GoalPos.Y*tan(GoalAngleX/180*PI);		                            //do not use tan2(), seg fault

		//Radius = 2*(GoalPosX*GoalPosX + GoalPosY*GoalPosY)/GoalPosX;   
		//TurnAngle = 180/PI*atan(GoalPosY/GoalPosX); //-pi/2 to pi/2
		//printf("camera width %f", (CamX - Camera::WIDTH/2));
		//printf("GoalAngleX %f", GoalAngleX);
		//printf("DX %lf\n", GoalPos.X);
		//printf("DY %lf\n\n", GoalPos.Y);

		return GoalPos;

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
		if (currentImage->m_ImageData[i*currentImage->m_PixelSize + 0] == 128 && currentImage->m_ImageData[i*currentImage->m_PixelSize + 1] == 255 && currentImage->m_ImageData[i*currentImage->m_PixelSize + 2] == 128)
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
		if (currentImage->m_ImageData[j*currentImage->m_PixelSize + 1] == 255 )
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

	
	Point2D GoalPos2 = CamPosToGoalPos(scanLine2Center, (height - scanLine2));		
	Point2D GoalPos1 = CamPosToGoalPos(scanLine1Center, (height - scanLine1));
	if(scanLine1Center == 0)
	{
		GoalPos1.X = 0; 
		GoalPos1.Y = 0;
	}
	if(scanLine2Center == 0)
	{
		GoalPos2.X = 0;
		GoalPos2.Y = 0;
	}
	if((GoalPos2.Y - GoalPos1.Y) !=0)	
	TurnAngle = 180/PI*atan((GoalPos2.X-GoalPos1.X)/(GoalPos2.Y-GoalPos1.Y));  //cot() = 1/tan(); (scanLine1Center - scanLine2Center)/(scanLine1 - scanLine2);
	else
	TurnAngle = 0;
	
	
	//If road is not found, don't steer
	if((scanLine1Center+scanLine2Center)<= 0)
	{	
		rotation = old_rotation;
	}
	else
	{		
		rotation = OffsetGain*(GoalPos2.X+GoalPos1.X)/2 + AngleGain*TurnAngle;          //k*TurnAngle; //k*((scanLine1Center+scanLine2Center)/2-offset);
	}
	
	rotation_diff = rotation - old_rotation;

	Steer::Filter();

	//motor limit
	float maxRotation = 20;
	if (RotationInput>maxRotation)
	{
		RotationInput = maxRotation;	
	}
	else if (RotationInput < -maxRotation)
	{
		RotationInput = -maxRotation;	
	}
	
	RotationInput = -RotationInput/maxRotation;
	
	
	//printf("rotation %f\n", rotation);
	//printf("scan1x %d", scanLine1Center);   //0--320
	//printf("scan2x %d\n", scanLine2Center); //0--320
	//printf("scanLine1 %d\n", scanLine1);
	//printf("scanLine2 %d\n", scanLine2);
	//printf("rotation %f", rotation); //-30~30 with k = 0.2
	printf("turnangle %f\n", TurnAngle);
	//printf("g1 x%f\n", GoalPos1.X);
	//printf("g2 x%f\n", GoalPos2.X);
	//printf("g1 y%f\n", GoalPos1.Y);
	//printf("g2 y%f\n", GoalPos2.Y);


	printf("rotationinput %f\n", RotationInput); //-600~600
/*
	if(Action::GetInstance()->m_Joint.GetEnable(JointData::ID_R_ELBOW) == true)
	Action::GetInstance()->m_Joint.SetAngle(5, MX28::Angle2Value(-90 + RotationInput));
	if(Action::GetInstance()->m_Joint.GetEnable(JointData::ID_L_ELBOW) == true)
	Action::GetInstance()->m_Joint.SetAngle(6, MX28::Angle2Value(90 + RotationInput);

	if(Action::GetInstance()->m_Joint.GetEnable(JointData::ID_R_SHOULDER_PITCH) == true)
	Action::GetInstance()->m_Joint.SetAngle(1, MX28::Angle2Value(90);
	if(Action::GetInstance()->m_Joint.GetEnable(JointData::ID_L_SHOULDER_PITCH) == true)
	Action::GetInstance()->m_Joint.SetAngle(2, MX28::Angle2Value(-90);

*/	

	
	delY_Old = delY;
	printf("delY_Old: %f,delY: %f \n", delY_Old,delY);		
	delY = 25*RotationInput;
	delY= m*delY +(1-m)*delY_Old;
	lsp = delY*.4453+lspOld;
	lep = delY*1.8*.1799+lepOld;	

	rsp = delY*.4453+rspOld;
	rep = delY*1.8*.1799+repOld;

	Action::GetInstance()->m_Joint.SetAngle(1, rsp);  
	Action::GetInstance()->m_Joint.SetAngle(2, lsp); 
	Action::GetInstance()->m_Joint.SetAngle(5, rep); 
	Action::GetInstance()->m_Joint.SetAngle(6, lep);  
	Action::GetInstance()->m_Joint.SetAngle(3, -40); //right roll
	Action::GetInstance()->m_Joint.SetAngle(4, 40); //cw left shoulder roll-90

	//usleep(50);
		//fprintf(stderr, "[STEER]");
	
}
