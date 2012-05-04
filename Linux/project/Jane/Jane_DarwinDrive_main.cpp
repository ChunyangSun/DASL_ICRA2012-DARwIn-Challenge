/*
 * Jane_darwinDrive_main.cpp
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
#include "Jane_StatusCheck.h"
//#include "Jane_DriveVision.h"

#include "Steer.h"
#include "RoadTracker.h"
#include "ImgProcess.h"

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "../../../Data/config.ini"
#define SCRIPT_FILE_PATH    "script.asc"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

int startpoint, begp; //begp = endp =-1;
int endpoint, endp;
int roadwidth;
int threshHold;
int scanLine1;
int scanLine2;
int width;
int height;
int steer = 0;
float g = 0;
float rotation = 0, out = 0;
int count;
int to, from;
int scanline = 60;


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

int _getch()
{
    struct termios oldt, newt;
    int ch;
    tcgetattr( STDIN_FILENO, &oldt );
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr( STDIN_FILENO, TCSANOW, &newt );
    ch = getchar();
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
    return ch;
}

void* pedal_thread(void* ptr)
{
	while(1){
       	int ch = _getch();
        if(ch == 0x20) {
          		if(Action::GetInstance()->m_Joint.GetValue(15) >= 1800 && Action::GetInstance()->m_Joint.GetValue(15) <= 2000) {
           			Action::GetInstance()->m_Joint.SetValue(15, 2200);
          		}
          		else {
            			Action::GetInstance()->m_Joint.SetValue(15, 1900);
        	   		}
      			}
			}
	return NULL;
}

int main(void)
{
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);
    Image* rgb_output = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);
    Image* grey_output = new Image(Camera::WIDTH, Camera::HEIGHT, Image::GREY_PIXEL_SIZE);;

    LinuxCamera::GetInstance()->Initialize(0);
    LinuxCamera::GetInstance()->SetCameraSettings(CameraSettings());    // set default
    LinuxCamera::GetInstance()->LoadINISettings(ini);                   // load from ini
    
    
    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    ColorFinder* ball_finder = new ColorFinder();
    ball_finder->LoadINISettings(ini);
    httpd::ball_finder = ball_finder;
 
    BallTracker tracker = BallTracker();
    BallFollower follower = BallFollower();

//ColorFinder(hue, hue_tol, min_saturation, min_value, min_per, max_per)
    ColorFinder* red_finder = new ColorFinder(0, 15, 45, 0, 0.3, 50.0); 

    red_finder->LoadINISettings(ini, "RED");
    httpd::red_finder = red_finder;

    ColorFinder* yellow_finder = new ColorFinder(60, 15, 45, 0, 0.3, 50.0);
    yellow_finder->LoadINISettings(ini, "YELLOW");
    httpd::yellow_finder = yellow_finder;

    ColorFinder* blue_finder = new ColorFinder(225, 15, 45, 0, 0.3, 50.0);
    blue_finder->LoadINISettings(ini, "BLUE");
    httpd::blue_finder = blue_finder;
	 
    ColorFinder* road_finder = new ColorFinder(15, 15, 0, 0, 0.3, 50.0); //look for road in black
    road_finder->LoadINISettings(ini);
    httpd::road_finder = road_finder;

    RoadTracker roadtracker = RoadTracker();
    Steer steer;
    ImgProcess imgprocess = ImgProcess();
    //steer.currentImage = rgb_output;
    //steer.currentImage = grey_output;
    httpd::ini = ini;

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

   // Walking::GetInstance()->LoadINISettings(ini);

    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());
   

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
    MotionManager::GetInstance()->SetEnable(true);//False: motor turns off

    cm730.WriteByte(CM730::P_LED_PANNEL, 0x01|0x02|0x04, NULL); //head LED turns yellow, camera on

    Action::GetInstance()->Start(1); //standing up
    while(Action::GetInstance()->IsRunning()) usleep(8*1000);


	pthread_t thread_t;
    pthread_create(&thread_t, NULL, pedal_thread, NULL);

    while(1)
    {
        StatusCheck::Check(cm730);

        Point2D ball_pos, red_pos, yellow_pos, blue_pos, road_pos;


		///////////add vision here///////////////////////////////////////////////////////////////////
        LinuxCamera::GetInstance()->CaptureFrame(); 
        memcpy(rgb_output->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, 		   		LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);
	//ImgProcess::RGBtoGREY(LinuxCamera::GetInstance()->fbuffer); //called in image.cpp, so we dont need to call here.
	//grey_output = LinuxCamera::GetInstance()->fbuffer->m_GREYFrame;
  memcpy(grey_output->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_GREYFrame->m_ImageData, 		   		LinuxCamera::GetInstance()->fbuffer->m_GREYFrame->m_ImageSize);
	
	
	if(StatusCheck::m_cur_mode == DRIVE)
	{ 
	    //imgprocess.Erosion(rgb_output);
	    //ImgProcess::Dilation(rgb_output);
	    //roadtracker.Process(road_finder->GetPosition(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame));

	  //  for(int i = 0; i < rgb_output->m_NumberOfPixels; i++)
            //{
		//if(road_finder->m_result->m_ImageData[i] == 1)
		//{
				//rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 0] = 128;
                   		//rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 1] = 255;
                   		//rgb_output->m_ImageData[i*rgb_output->m_PixelSize + 2] = 128;
		 //}  
	   //}
	   //steer.Process();
          
			//rgb_output->m_ImageData[(int) Steer().center*rgb_output->m_PixelSize + 0] = 255;
                   	//rgb_output->m_ImageData[(int) Steer().center*rgb_output->m_PixelSize + 1] = g;
                   	//rgb_output->m_ImageData[(int) Steer().center*rgb_output->m_PixelSize + 2] = b;
        } //else if: DRIVE
      

        streamer->send_image(grey_output);
        //streamer->send_image(rgb_output);


        if(StatusCheck::m_is_started == 0)
            continue;

        switch(StatusCheck::m_cur_mode)
        {
        case READY:
            break;
       
	case DRIVE:
	     {		
		
		//printf("ID7 %d", Action::GetInstance()->m_Joint.GetCCWSlope(7));
		//printf("HERE");
		//MotionManager::GetInstance()->SetEnable(false);
	  		
		//MotionManager::GetInstance()->SetEnable(true);
   				 

		//	Action::GetInstance()->m_Joint.SetValue(1,2500);
		//	Action::GetInstance()->m_Joint.SetValue(2,1500);
               		

	   
	                //Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);
			
	    }

	    break;
	
           }
       }
    
    return 0;
}
