/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_remote.c
 *  Description  : This file contains Remote control function
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:44:37
 *  LastEditTime : 2024-01-09 14:59:08
 */


#include "app_remote.h"
#include "sys_const.h"
#include "alg_filter.h"
#include "periph_remote.h"
#include "module_platform.h"
#include "module_communicate.h"
#include "app_gimbal.h"
#include "module_chassis.h"

Remote_RemoteControlTypeDef Remote_remoteControlData;

/**
  * @brief          Remote task
  * @param          NULL
  * @retval         NULL
  */
void Remote_Task(void const * argument) {
  for(;;) {
    Remote_ControlCom();

    osDelay(2);
  }
}


/**
  * @brief      Remote Control Init
  * @param      NULL
  * @retval     NULL
  */
void Remote_RemotrControlInit() {
    Remote_RemoteControlTypeDef *control_data = Remote_GetControlDataPtr();
}


/**
  * @brief      Gets the pointer to the remote control data object
  * @param      NULL
  * @retval     Pointer to remote control data object
  */
Remote_RemoteControlTypeDef* Remote_GetControlDataPtr() {
    return &Remote_remoteControlData;
}


/**
* @brief      Remote control command
* @param      NULL
* @retval     NULL
*/
void Remote_ControlCom() {
    Remote_RemoteControlTypeDef *control_data = Remote_GetControlDataPtr();
    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
 
    control_data->pending = 1;

    switch (data->remote.s[0]) {
        case Remote_SWITCH_UP: {       //jiefa
          Remote_RemoteProcess();          
          break;
        }
        case Remote_SWITCH_MIDDLE: {
          Remote_RemoteProcessPlus();			//chuanqiu  
          break;
        }
        case Remote_SWITCH_DOWN: {    //dianqiu
          Remote_NucProcess();   
          break;
        }
        default:
          break;
    }
    control_data->pending = 0;
}


/**
* @brief      Remote control process
* @param      NULL
* @retval     NULL
*/

void Remote_RemoteProcess() {
	
  Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
  Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
	Gimbal_DataTypeDef *gimbal = Gimbal_GetDataPtr();
	
  switch (data->remote.s[1]) {
		// 完全交由上位机控制
    case Remote_SWITCH_UP: {
      Platform_Set_ControlMode(Platform_Jiefa);	
			Chassis_SetControlMode(Chassis_PC);
      Platform_Hitting_Pose_Update();
      Platform_Set_Hitting_Pose(10.0f, 0.0f, 0.0f, 2);
      // Platform_Set_Hitting_Pose(10, 0, 0, 3);
			break;
    }
		// 完全交由遥控器控制
    case Remote_SWITCH_MIDDLE: {
      Platform_Set_ControlMode(Platform_Initpose);
      Chassis_SetControlMode(Chassis_Remote);   
      Chassis_Set_Speed(Chassis_Run, data->remote.ch[2]/660.0f * 3.0f,data->remote.ch[3]/660.0f * 3.0f,data->remote.ch[0]/660.0f * 3.0f);  							
	    // Chassis_Set_Speed( data->remote.ch[2]/660.0f * 2.0f,data->remote.ch[3]/660.0f * 2.0f,data->remote.ch[0]/660.0f * 3.0f);
      break;
    }
		// 紧急制动全部停止
    case Remote_SWITCH_DOWN: {
	    Platform_Set_ControlMode(Platform_Jiefa);	
			Chassis_SetControlMode(Chassis_Remote);  
      Chassis_Set_Speed(Chassis_Run, data->remote.ch[2]/660.0f * 3.0f,data->remote.ch[3]/660.0f * 3.0f,data->remote.ch[0]/660.0f * 3.0f);  							
			//Chassis_Set_Speed(Chassis_Stop,0,0,0); 						
      break;
    }
    default:
      break;
  }
}


/**
* @brief      Remote control process
* @param      NULL
* @retval     NULL
*/
void Remote_RemoteProcessPlus() {

  Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
  Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
  
  switch (data->remote.s[1]) {
    case Remote_SWITCH_UP: {
      Platform_Set_ControlMode(Platform_Chuanqiu);
      Platform_Set_Target_Pos(0, 0, data->remote.ch[1] * 0.15 / 660.0f + 0.12f, data->remote.ch[2] * 30.0f / 660.0f, 0, data->remote.ch[3] * 60.0f / 660.0f);
      Chassis_Set_Speed(Chassis_Lock, 0, 0, 0);
      break;
    }
    case Remote_SWITCH_MIDDLE: {    
      Platform_Set_ControlMode(Platform_Initpose);		
      Chassis_SetControlMode(Chassis_Remote);   
      Chassis_Set_Speed(Chassis_Run, data->remote.ch[2]/660.0f * 3.0f,data->remote.ch[3]/660.0f * 3.0f,data->remote.ch[0]/660.0f * 3.0f); 
      break;
    }
    case Remote_SWITCH_DOWN: {
      Platform_Set_ControlMode(Platform_Stop);
      Chassis_SetControlMode(Chassis_Remote);
      Chassis_Set_Speed(Chassis_Stop, 0, 0, 0);
      Chassis_SetControlMode(Chassis_Remote);
      Chassis_Set_Speed(Chassis_Run, data->remote.ch[2]/660.0f * 3.0f,data->remote.ch[3]/660.0f * 3.0f,data->remote.ch[0]/660.0f * 3.0f); 

      Platform_Set_ControlMode(Platform_Dianqiu);
      break;
    }
    default:
      break;
  }
}

// 遥控器控制小车和平台颠球
void Remote_NucProcess() {
	
  Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr(); // 遥控器控制指针
  Platform_DataTypeDef *Platform = Platform_GetPlatformPtr(); // 平台控制指针
    
  switch (data->remote.s[1]) {
    case Remote_SWITCH_UP: {
			
      //Chassis_SetControlMode(Chassis_PC);		
      Chassis_SetControlMode(Chassis_Remote);   
      Chassis_Set_Speed(Chassis_Run, data->remote.ch[2]/660.0f * 3.0f,data->remote.ch[3]/660.0f * 3.0f,data->remote.ch[0]/660.0f * 3.0f); 
      Platform_Set_ControlMode(Platform_Dianqiu);		
      break;
    }
    case Remote_SWITCH_MIDDLE: {
	 	  Chassis_SetControlMode(Chassis_Remote);  					
      Chassis_Set_Speed(Chassis_Run, data->remote.ch[2]/660.0f * 3.0f,data->remote.ch[3]/660.0f * 3.0f,data->remote.ch[0]/660.0f * 3.0f); 		
      Platform_Set_ControlMode(Platform_Dianqiu);	
		  //Platform_Set_Target_Pos(0,0,data->remote.ch[1]*0.25/660.0f+0.15f,data->remote.ch[2]*30.0f/660.0f,0,data->remote.ch[3]*30.0f/660.0f);		
      break;
    }
    case Remote_SWITCH_DOWN: {     
      //Chassis_SetControlMode(Chassis_Location);   
		  Chassis_SetControlMode(Chassis_Remote);  
      Chassis_Set_Speed(Chassis_Run, data->remote.ch[2]/660.0f * 3.0f,data->remote.ch[3]/660.0f * 3.0f,data->remote.ch[0]/660.0f * 3.0f); 
		  Platform_Set_ControlMode(Platform_Dianqiu);				
      break;
    }
    default:
      break;
  }
}
