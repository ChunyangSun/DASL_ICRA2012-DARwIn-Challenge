/*
 * Jane_darwinDrive_main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <libgen.h>
#include <signal.h>

#include <termios.h>
#include <term.h>
#include <pthread.h>


//opencv headers
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <sys/un.h>

#include <fcntl.h>
#include <unistd.h>
#include <math.h>

#include "minIni.h"
#include "LinuxDARwIn.h"
#include "Steer.h"
#include "Action.h"

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"
#endif

#define INI_FILE_PATH       "../../../Data/config.ini"

#define U2D_DEV_NAME0       "/dev/ttyUSB0"
#define U2D_DEV_NAME1       "/dev/ttyUSB1"

using namespace cv;

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
    //opencv camera initialization
	CvCapture* capture = cvCaptureFromCAM( 0 );
	if( !capture ) {
		fprintf( stderr, "ERROR: capture is NULL \n" );
		getchar();
		return -1;
	}
   
   IplImage image;

   image.width = 320;
   image.depth = 240;
 
// Create a window in which the captured images will be presented
   NamedWindow( "mywindow", CV_WINDOW_AUTOSIZE );




   signal(SIGABRT, &sighandler);
   signal(SIGTERM, &sighandler);
   signal(SIGQUIT, &sighandler);
   signal(SIGINT, &sighandler);

//???
    change_current_dir();
//???
    minIni* ini = new minIni(INI_FILE_PATH);




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


   int p;

  


    Action::GetInstance()->m_Joint.SetEnableBody(true, true);			
    MotionManager::GetInstance()->SetEnable(true);//False: motor turns off

    Action::GetInstance()->Start(1); //standing up
    while(Action::GetInstance()->IsRunning()) usleep(8*1000);


    pthread_t thread_t;
    pthread_create(&thread_t, NULL, pedal_thread, NULL);
     
    //steer.Process();
          


 	int i = 0;
    while(1){
		// Get one frame
		IplImage* frame = cvQueryFrame( capture );
		if( !frame ) {
		      fprintf( stderr, "ERROR: frame is null...\n" );
		      getchar();
		      break;
		}
		cvShowImage( "mywindow", frame );
		cvReleaseCapture( &capture );
		


		printf("HERE");
		//MotionManager::GetInstance()->SetEnable(false);
  		int iniPos = Action::GetInstance()->m_Joint.GetValue(3);
		
		if (i<50){
				Action::GetInstance()->m_Joint.SetValue(1,iniPos+ 4*i);
				//Action::GetInstance()->m_Joint.SetValue(2,2000);
				i++;
		}	            
			
     	     }   
    
    return 0;
}
