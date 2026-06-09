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
#include "comm_receive.h"

extern float keep_state_flag;

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

/*
遥控器键位
UU	平台自动接发	云台自动 	底盘自动				imu
UM 	平台自动接发 	云台自动 	底盘自动 	给上位机发1	imu
UD	平台自动接发 	云台自动 	底盘遥控				imu

MU	平台自动垫球 模式1 	云台自动 	底盘自动				imu
MM	平台自动垫球 模式2 	云台自动 	底盘自动 	给上位机发1	imu
MD	平台自动垫球 模式3 	云台自动 	底盘遥控				imu

DU	平台遥控 	无云台   	底盘停止
DM	平台停止 	云台遥控 	底盘停止
DD	全关
*/


/*新遥控器键位 s[0] s[3]
UU 平台自动接发 云台自动
UM 平台自动接发 云台遥控

DD 平台击打一次 
DM 平台初始位姿
DU 平台停止 
*/
/**
* @brief      Remote control command
* @param      NULL
* @retval     NULL
*/
void Remote_ControlCom() {
    Remote_RemoteControlTypeDef *control_data = Remote_GetControlDataPtr();
    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
		Remote_DataTypeDef *rc = Remote_GetRemotePtr(); 

    control_data->pending = 1;

    switch (rc->s[0]) {
        case Lever_up: {
          Remote_RemoteProcess();          
          break;
        }
        case Lever_middle: {
          Remote_RemoteProcessPlus();
          break;
        }
        case Lever_down: {
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
	Remote_DataTypeDef *rc = Remote_GetRemotePtr(); 

	Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
	Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
	Gimbal_TypeDef *gimbal = Gimbal_GetPtr();
	
	switch (rc->s[3]) {
		case Lever_down: {
			Platform_Set_ControlMode(Platform_Jiefa);	
			keep_state_flag = 0.0f;
			Chassis_SetControlMode(Chassis_PC);
			Gimbal_StateSet(Gimbal_auto);
			break;
		}
		case Lever_middle: {
			Platform_Set_ControlMode(Platform_Initpose);
			Chassis_SetControlMode(Chassis_PC); 
			Gimbal_StateSet(Gimbal_remote);  							
			break;
		}
		case Lever_up: {
			Platform_Set_ControlMode(Platform_Stop);	
			Chassis_SetControlMode(Chassis_Remote); 
			Chassis_Set_Speed(Chassis_Run, data->remote.ch[2]/660.0f * 3.0f,data->remote.ch[3]/660.0f * 3.0f,data->remote.ch[0]/660.0f * 3.0f); 
			Gimbal_StateSet(Gimbal_auto);
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
  Remote_DataTypeDef *rc = Remote_GetRemotePtr();

    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
    Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
    
    switch (rc->s[3]) {
        case Lever_down: {
			Platform_Set_ControlMode(Platform_Dianqiu);	
      Platform_Dianqiu_Cal(1.0f, Remote_SWITCH_DOWN);
			// Chassis_SetControlMode(Chassis_PC);   
			Gimbal_StateSet(Gimbal_auto);
			break;
        }
        case Lever_middle: {
			Platform_Set_ControlMode(Platform_Initpose);	
			// Chassis_SetControlMode(Chassis_PC);   
			Gimbal_StateSet(Gimbal_auto);
			break;
        }
        case Lever_up: {
			Platform_Set_ControlMode(Platform_Stop);
			// Chassis_SetControlMode(Chassis_Remote);
			Chassis_Set_Speed(Chassis_Stop,0,0,0);
			Gimbal_StateSet(Gimbal_auto);
			break;
        }
        default:
			break;
    }
}

void Remote_NucProcess() {
		Remote_DataTypeDef *rc = Remote_GetRemotePtr(); 

    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
    Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
    
    switch (rc->s[3]) {
        case Lever_down // 手动击球
					: {
            // 击球模式：执行快速上升-回落动作
            Platform_Set_ControlMode(Platform_Jiefa);
						keep_state_flag = 1; // 保持状态标志，确保在击球动作完成前不被其他输入打断
            Platform->flag = 1.0f; // ch[3]推上=轻击0.7 拉下=重击1.0
						Gimbal_StateSet(Gimbal_auto);		
            break;
        }
        case Lever_middle: {
            // Platform_Set_Target_Pos(0,0,Initial_Z_Position,0,0,0);
            Platform_Set_ControlMode(Platform_Initpose);  
						Gimbal_StateSet(Gimbal_auto);
						break;
        }
        case Lever_up: {
            Platform_Set_ControlMode(Platform_Jiefa);
            keep_state_flag = 1;
            Platform->flag = 0.7f;
            // Chassis_Set_Speed(Chassis_Stop,0,0,0);
            Gimbal_StateSet(Gimbal_Off);
            break;
        }
        default:
            break;
    }
}
