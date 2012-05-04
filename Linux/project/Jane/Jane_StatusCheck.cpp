/*
 * StatusCheck.cpp
 *
 *  Created on: 2011. 1. 21.
 *      Author: zerom
 */

#include <stdio.h>
#include <unistd.h>

#include "Jane_StatusCheck.h"
#include "Head.h"
#include "Action.h"
#include "MotionStatus.h"
#include "MotionManager.h"
#include "LinuxActionScript.h"
#include "LinuxDARwIn.h"
using namespace Robot;

int StatusCheck::m_cur_mode     = READY;
int StatusCheck::m_old_btn      = 0;
int StatusCheck::m_is_started   = 0;

void StatusCheck::Check(CM730 &cm730)
{
   
    if(m_old_btn == MotionStatus::BUTTON)
        return;

    m_old_btn = MotionStatus::BUTTON;

    if(m_old_btn & BTN_MODE)
    {
        fprintf(stderr, "Mode button pressed.. \n");

        if(m_is_started == 1)
        {
            m_is_started    = 0;
            m_cur_mode      = READY;
            LinuxActionScript::m_stop = 1;

            //Walking::GetInstance()->Stop();
            //Action::GetInstance()->m_Joint.SetEnableBody(true, true);

            while(Action::GetInstance()->Start(15) == false) usleep(8000);
            while(Action::GetInstance()->IsRunning() == true) usleep(8000);
        }
        else
        {
            m_cur_mode++;
            if(m_cur_mode >= MAX_MODE) m_cur_mode = READY;
        }

        MotionManager::GetInstance()->SetEnable(false);
        usleep(10000);

        if(m_cur_mode == READY)
        {
            cm730.WriteByte(CM730::P_LED_PANNEL, 0x01|0x02|0x04, NULL);
            LinuxActionScript::PlayMP3("../../../Data/mp3/Demonstration ready mode.mp3");
        }
       
	else if(m_cur_mode == DRIVE)
        {
            cm730.WriteByte(CM730::P_LED_PANNEL, 0x04, NULL);
	    printf("Driving Mode!\n");
          //  LinuxActionScript::PlayMP3("../../../Data/mp3/Jane_Hotter Than That.mp3");
        }
    }

    if(m_old_btn & BTN_START)
    {
        if(m_is_started == 0)
        {
            fprintf(stderr, "Start button pressed.. & started is false.. \n");

         }
  	   else if(m_cur_mode == DRIVE)
            {
                MotionManager::GetInstance()->Reinitialize();
                MotionManager::GetInstance()->SetEnable(true);//GET INTO MAIN:CASES 
                m_is_started = 1; //zero: can't control the joints
                //LinuxActionScript::PlayMP3("../../../Data/mp3/Jane_Hotter Than That.mp3");
		
                // Joint Enable...
       		Action::GetInstance()->m_Joint.SetEnableBody(true, true);//(enable, exclusive)

		//Action::GetInstance()->m_Joint.JointData::SetEnableRightArmOnly(true,true);
		//Action::GetInstance()->m_Joint.JointData::SetEnableLeftArmOnly(true,true);
		//Action::GetInstance()->m_Joint.JointData::SetEnableLowerBody(true,true);
		//Action::GetInstance()->Start(1);
		
		Action::GetInstance()->Start(7); 
		 while(Action::GetInstance()->IsRunning() == true) usleep(1000);		
		Action::GetInstance()->Start(14); //go to pedal
		 while(Action::GetInstance()->IsRunning() == true) usleep(1000);	
		Action::GetInstance()->Start(20); //go to steer
		printf("Press the SPACE key to start/stop driving \n\n");
		/*
    		cm730.WriteWord(7, MX28::P_TORQUE_ENABLE, 0, 0);
		cm730.WriteWord(8, MX28::P_TORQUE_ENABLE, 0, 0);
    		cm730.WriteWord(9, MX28::P_TORQUE_ENABLE, 0, 0);
    		cm730.WriteWord(10, MX28::P_TORQUE_ENABLE, 0, 0);
		cm730.WriteWord(11, MX28::P_TORQUE_ENABLE, 0, 0);
    		cm730.WriteWord(12, MX28::P_TORQUE_ENABLE, 0, 0);
    		cm730.WriteWord(13, MX28::P_TORQUE_ENABLE, 0, 0);
    		cm730.WriteWord(14, MX28::P_TORQUE_ENABLE, 0, 0);
		cm730.WriteWord(15, MX28::P_TORQUE_ENABLE, 0, 0);
    		cm730.WriteWord(16, MX28::P_TORQUE_ENABLE, 0, 0);
    		cm730.WriteWord(17, MX28::P_TORQUE_ENABLE, 0, 0);
    		cm730.WriteWord(18, MX28::P_TORQUE_ENABLE, 0, 0);
    		//cm730.WriteWord(19, MX28::P_TORQUE_ENABLE, 0, 0);
  		
		
   		cm730.WriteByte(7, MX28::P_P_GAIN, 8, 0);
		cm730.WriteByte(8, MX28::P_P_GAIN, 8, 0);
		cm730.WriteByte(9, MX28::P_P_GAIN, 8, 0);
		cm730.WriteByte(10, MX28::P_P_GAIN, 8, 0);
		cm730.WriteByte(11, MX28::P_P_GAIN, 8, 0);
		cm730.WriteByte(12, MX28::P_P_GAIN, 8, 0);
		cm730.WriteByte(13, MX28::P_P_GAIN, 8, 0);
		cm730.WriteByte(14, MX28::P_P_GAIN, 8, 0);
		cm730.WriteByte(15, MX28::P_P_GAIN, 8, 0);
		cm730.WriteByte(16, MX28::P_P_GAIN, 8, 0);
		cm730.WriteByte(17, MX28::P_P_GAIN, 8, 0);
		//cm730.WriteByte(18, MX28::P_P_GAIN, 8, 0);
		*/
                while(Action::GetInstance()->IsRunning() == true) usleep(8000);
            }
	}
        
        else
        {
            fprintf(stderr, "Start button pressed.. & started is true.. \n");
        }
    }

