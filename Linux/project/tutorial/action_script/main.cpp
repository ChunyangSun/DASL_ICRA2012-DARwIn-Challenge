/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <unistd.h>
#include <string.h>
#include <libgen.h>

#include "Camera.h"
#include "Point.h"
#include "mjpg_streamer.h"
#include "minIni.h"
#include "LinuxCamera.h"
#include "ColorFinder.h"

#include "Action.h"
#include "Head.h"
#include "Walking.h"
#include "MX28.h"
#include "MotionManager.h"
#include "LinuxMotionTimer.h"
#include "LinuxCM730.h"
#include "LinuxActionScript.h"
#define INI_FILE_PATH       "../../../Data/config.ini"
#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../../Data/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../../Data/motion_4096.bin"
#endif
int leftarm, rightarm;
void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

int main(void)
{
    printf( "\n===== Action script Tutorial for DARwIn =====\n\n");

    change_current_dir();

    Action::GetInstance()->LoadFile(MOTION_FILE_PATH);

    //////////////////// Framework Initialize ////////////////////////////
    LinuxCM730 linux_cm730("/dev/ttyUSB0");
    CM730 cm730(&linux_cm730);
    if(MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        printf("Fail to initialize Motion Manager!\n");
            return 0;
    }
    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();
    /////////////////////////////////////////////////////////////////////

    	//MotionManager::GetInstance()->SetEnable(true);
 	//Action::GetInstance()->m_Joint.SetEnableBody(true, true);// this opposes to the torque disable
	//Action* Action::m_UniqueInstance = new Action();
   	//Action::GetInstance()->Start(1);
    	MotionManager::GetInstance()->SetEnable(true);

 
   /* Init(stand up) pose */
   	Action::GetInstance()->Start(1);
   	Action::GetInstance()->Start(5);
   	Action::GetInstance()->Start(6);
   	Action::GetInstance()->Start(7);
   	Action::GetInstance()->Start(8);
   while(LinuxActionScript::m_is_running == 1) sleep(10);
   	Action::GetInstance()->Start(9);
   while(LinuxActionScript::m_is_running == 1) sleep(10);
   	Action::GetInstance()->Start(10);
    while(Action::GetInstance()->IsRunning()) usleep(8*1000);

    printf("Press the ENTER key to begin!\n");
    getchar();
      //LinuxActionScript::m_is_running = 0;
		//Action::GetInstance()->m_Joint.SetAngle(6, MotionStatus::m_CurrentJoints.GetValue(6));
		//leftarm = JointData::GetAngle(int ID_R_SHOULDER_PITCH);  
		//Action::GetInstance()->m_Joint.SetAngle(5, MotionStatus::m_CurrentJoints.GetValue(5));
		//leftarm = MotionStatus::m_CurrentJoints.GetAngle(6);
		//rightarm = MotionStatus::m_CurrentJoints.GetAngle(5);
   // cm730.WriteWord(JointData::ID_R_SHOULDER_PITCH, MX28::P_TORQUE_ENABLE, 0, 0);
  //  cm730.WriteWord(JointData::ID_R_SHOULDER_ROLL,  MX28::P_TORQUE_ENABLE, 0, 0);
  //  cm730.WriteWord(JointData::ID_R_ELBOW,          MX28::P_TORQUE_ENABLE, 0, 0);
		//Action::GetInstance()->m_Joint.SetValue(1, 2000);
 		//Action::GetInstance()->m_Joint.SetValue(2, 2000);
		
		//while(Action::GetInstance()->IsRunning()) usleep(8000);
		
		//Action::GetInstance()->m_Joint.SetAngle(6, -20);
		//Action::GetInstance()->m_Joint.SetAngle(5, 20);

   	Action::GetInstance()->Start(5);
   	Action::GetInstance()->Start(6);
   	Action::GetInstance()->Start(7);
   	Action::GetInstance()->Start(8);
   	Action::GetInstance()->Start(9);
   	Action::GetInstance()->Start(10);
   // LinuxActionScript::ScriptStart("script.asc");
   while(LinuxActionScript::m_is_running == 1) sleep(10);

    return 0;
}
