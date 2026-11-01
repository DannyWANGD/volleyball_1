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
UU	平台零力矩
UM 	平台初始化 Initpose
UD	平台手动击打（小力）

MU	平台自动击打
MM	平台初始化 Initpose
MD	平台手动击打（大力）

DU	平台 Dianqiu 击打
DM	平台初始化 Initpose
DD	平台手动击打（大力）
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
		case Lever_up: { // 零力矩
			Platform_Set_ControlMode(Platform_Stop);	
			break;
		}
		case Lever_middle: { // 初始化
			Platform_Set_ControlMode(Platform_Initpose); 							
			break;
		}
		case Lever_down: { // 手动 大力击打
			Platform_Set_ControlMode(Platform_Jiefa);
			keep_state_flag = 1.0f;
			Platform->flag = 1.0f;
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
			
			case Lever_up: { // 手动 前场颠球
				Platform_Set_ControlMode(Platform_Dianqiu);	
				Platform_Dianqiu_Cal(1.0f, Remote_SWITCH_DOWN);	
				break;
			}
			case Lever_middle: { // 自动 大力击打
				Platform_Set_ControlMode(Platform_Jiefa);	
				keep_state_flag = 0.0f;
				Chassis_SetControlMode(Chassis_PC);	
				break;
			}
			case Lever_down: { // 手动 大力击打
				Platform_Set_ControlMode(Platform_Jiefa);
				keep_state_flag = 1.0f;
				Platform->flag = 1.0f;
//				Platform_Set_ControlMode(Platform_Jiefa);	
//				keep_state_flag = 0.0f;
//				Chassis_SetControlMode(Chassis_PC);	
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
			case Lever_up: { // 手动 前场颠球
					Platform_Set_ControlMode(Platform_Dianqiu);	
					Platform_Dianqiu_Cal(1.0f, Remote_SWITCH_DOWN);		
					break;
			}
			case Lever_middle: { // 初始化
					Platform_Set_ControlMode(Platform_Initpose);  
					break;
			}
			case Lever_down: { // 手动 大力击打
					Platform_Set_ControlMode(Platform_Jiefa);
					keep_state_flag = 1.0f;
					Platform->flag = 1.0f;
					break;
			}
			default:
					break;
    }
}
