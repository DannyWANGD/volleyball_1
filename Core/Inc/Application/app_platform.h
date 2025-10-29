/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_remote.h
 *  Description  : This file contains Remote control function
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:44:57
 *  LastEditTime : 2023-09-23 21:35:07
 */
#ifndef APP_PLATFORM_H
#define APP_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "cmsis_os.h"
#include "FreeRTOS.h"

void Platform_Task(void const * argument);




#endif

#ifdef __cplusplus
}
#endif
