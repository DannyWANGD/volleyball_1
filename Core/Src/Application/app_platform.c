/*
 *  Project      :DeltaPlatform
 * 
 *  file         : app_platform.c
 *  Description  : This file contains DeltaPlatform task function
 *  LastEditors  : lkn
 *  Date         : 2024-7-2
 *  LastEditTime : 2024-7-3
 */
 
#include "module_platform.h"
#include "app_platform.h"
#include "sys_dwt.h"
 /**
  * @brief          Platform task
  * @param          NULL
  * @retval         NULL
  */

void Platform_Task(void const * argument) {

  Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
  osDelay(500);

  for(;;) {

    // 获取最新的传感器反馈数据
    Platform_Update_Fdb();
    // 进行安全监控
    // Platform_Check();

    /*if(Platform->error_code == 0){
      Platform_Control();
    } else {
      Platform_Set_Torque_Output(0,0,0,0,0,0);
    }*/

    // 输出控制指令到电机驱动器
    Platform_Control();
		Platform_Output();
    
    osDelay(2);
  }
}
