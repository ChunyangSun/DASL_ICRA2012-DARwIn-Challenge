/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */
#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <libgen.h>
#include <signal.h>

#include <termios.h>
#include <term.h>
#include <pthread.h>

#include "minIni.h"
#include "mjpg_streamer.h"
#include "LinuxDARwIn.h"

#include "Action.h"
#include "Head.h"

#include "MX28.h"
#include "MotionManager.h"
#include "LinuxMotionTimer.h"
#include "LinuxCM730.h"
#include "LinuxActionScript.h"

#include "RoadTracker.h"
#include "Steer.h"
#include "math.h"

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "../../../Data/config.ini"
#define SCRIPT_FILE_PATH    "script.asc"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

LinuxCM730 linux_cm730(U2D_DEV_NAME0);
CM730 cm730(&linux_cm730);

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
    {
        if(chdir(dirname(exepath)))
            fprintf(stderr, "chdir error!! \n");
    }
}

void sighandler(int sig)
{
    exit(0);
}

int main(void)
{
	int i = 0;
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);
    Image* rgb_output = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->SetCameraSettings(CameraSettings());    // set default
    LinuxCamera::GetInstance()->LoadINISettings(ini);                   // load from ini

    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    ColorFinder* road_finder = new ColorFinder(180, 180, 0, 20, 60, 100, 0.07, 30); //look for road in black, 
    //road_finder->LoadINISettings(ini);
    httpd::road_finder = road_finder;
    
    RoadTracker roadtracker = RoadTracker();
    Steer steer;			         //Steer Class, steer object
    steer.currentImage = rgb_output;             //Steer class member
    ImgProcess imgprocess = ImgProcess();

    //httpd::ini = ini;

    //////////////////// Framework Initialize ////////////////////////////
    if(MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        linux_cm730.SetPortName(U2D_DEV_NAME1);
        if(MotionManager::GetInstance()->Initialize(&cm730) == false)
        {
            printf("Fail to initialize Motion Manager!\n");
            return 0;
        }
    }

    Walking::GetInstance()->LoadINISettings(ini);

    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());

    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();
    /////////////////////////////////////////////////////////////////////
    
    MotionManager::GetInstance()->LoadINISettings(ini);

    int firm_ver = 0;
    if(cm730.ReadByte(JointData::ID_HEAD_PAN, MX28::P_VERSION, &firm_ver, 0)  != CM730::SUCCESS)
    {
        fprintf(stderr, "Can't read firmware version from Dynamixel ID %d!! \n\n", JointData::ID_HEAD_PAN);
        exit(0);
    }

    if(0 < firm_ver && firm_ver < 27)
    {
#ifdef MX28_1024
        Action::GetInstance()->LoadFile(MOTION_FILE_PATH);
#else
        fprintf(stderr, "MX-28's firmware is not support 4096 resolution!! \n");
        fprintf(stderr, "Upgrade MX-28's firmware to version 27(0x1B) or higher.\n\n");
        exit(0);
#endif
    }
    else if(27 <= firm_ver)
    {
#ifdef MX28_1024
        fprintf(stderr, "MX-28's firmware is not support 1024 resolution!! \n");
        fprintf(stderr, "Remove '#define MX28_1024' from 'MX28.h' file and rebuild.\n\n");
        exit(0);
#else
        Action::GetInstance()->LoadFile((char*)MOTION_FILE_PATH);
#endif
    }
    else
        exit(0);

    Action::GetInstance()->m_Joint.SetEnableBody(true, true);
    MotionManager::GetInstance()->SetEnable(true);
 
    while(Action::GetInstance()->IsRunning()) usleep(8*1000);

    while(1)
    {	
	LinuxCamera::GetInstance()->CaptureFrame();
        memcpy(rgb_output->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
	
        //ColorFinder* greyscale = new ColorFinder(0, 15, 0, grey, 0.3, 50.0);
	
	Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);

	//find color using only saturation in HSV, 	
	roadtracker.Process(road_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame));
	    for(int i = 0; i < rgb_output->m_NumberOfPixels; i++)
            {
		if(road_finder->m_result->m_ImageData[i] == 1)
		{
				rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 128;
                   		rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 255;
                   		rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 128;
		}  
	   }
		
	  //steer.Process();
	  streamer->send_image(rgb_output);

	//printf("right shoulder %f", Action::GetInstance()->m_Joint.GetAngle(1));  //right shoulder
	//printf("left shoulder %f", Action::GetInstance()->m_Joint.GetAngle(2));
	float delY = 0;
	float count = 0;	
	float lsp,lep,rsp,rep;
	float lspOld = -50;
	float lepOld = 25;
	float rspOld = 50;
	float repOld = -25;

	while (1) {

		delY = 25*sin(count);
		lsp = delY*.4453+lspOld;
		lep = delY*1.8*.1799+lepOld;	//.1799

		rsp = delY*.4453+rspOld;
		rep = delY*1.8*.1799+repOld;
		//Action::GetInstance()->m_Joint.SetAngle(1, 50);  //ccw 90
		Action::GetInstance()->m_Joint.SetAngle(1, rsp);  //ccw 90
		//Action::GetInstance()->m_Joint.SetAngle(2, -50); //cw left shoulder -90
		Action::GetInstance()->m_Joint.SetAngle(2, lsp); //cw left shoulder -90
		//Action::GetInstance()->m_Joint.SetAngle(5, -25); //cw right elbow -90
		Action::GetInstance()->m_Joint.SetAngle(5, rep); //cw right elbow -90
		//Action::GetInstance()->m_Joint.SetAngle(6, 25);  //ccw 90 left elbow
		Action::GetInstance()->m_Joint.SetAngle(6, lep);  //ccw 90 left elbow
		Action::GetInstance()->m_Joint.SetAngle(3, -40); //right roll
		Action::GetInstance()->m_Joint.SetAngle(4, 40); //cw left shoulder roll-90

		count += .0000002*3.141592654;
	}
	/*int j = 0;
       if ( j < 30)
	{	for (j = 0; j<30; j++)
		{
			
		}
	}
*/
   } 

	return 0;
}
