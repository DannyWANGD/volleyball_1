/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_remote.c
 *  Description  : This file contains Remote control function
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:44:37
 *  LastEditTime : 2024-01-09 14:59:08
 */

#include "app_communicate.h"
#include "module_communicate.h"
#include "module_platform.h"
#include "cmsis_os.h"
#include "util_uart.h"
#include "FreeRTOS.h"
#include "sys_const.h"
#include "string.h"
#include "stdio.h"
#include "lib_str.h"
#include "semphr.h"
#include "task.h"
#include "sys_dwt.h"
#include "util_uart.h"
#include "comm_common.h"

void Communicate_Task(void const * argument) {
	 
   osDelay(500);
	 Comm_DataTypeDef *buscomm = Comm_GetBusDataPtr();
      for(;;) {
      buscomm->tx_dt = DWT_GetDeltaT(&buscomm->last_tx_tick);
      Comm_SendCommData();
      osDelay(3);
   }
}


