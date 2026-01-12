/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : app_init.c
 *  Description  : All initialization threads
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:42:52
 *  LastEditTime : 2023-10-28 18:15:38
 */
 
 #include "app_init.h"
 #include "app_remote.h"
 #include "sys_dwt.h"
 #include "util_can.h"
 #include "module_platform.h"
 #include "module_communicate.h"
 #include "periph_motor.h"
 #include "FreeRTOS.h"
 #include "cmsis_os.h"
 #include "periph_remote.h"
 #include "periph_dt35.h"
 #include "comm_common.h"
 #include "app_serve.h"
 #include "app_gimbal.h"
 #include "module_chassis.h"
 
void Init_InitAll() {

	// system init
	DWT_Init(168);

	// uart init
	Remote_InitRemote();
	Comm_InitComm();
	// util init
	Can_InitFilterAndStart(&hcan1);
	Can_InitFilterAndStart(&hcan2);
	// periph init
	DT35_Init();
	Motor_InitAllMotors();
	HAL_Delay(500);
	Motor_DM_Basic_Ctrl(Motor_Enable);  
	// Motor_DM_Basic_Ctrl(Motor_SaveInitpos);
	// App init
	Platform_Init();
	Remote_RemotrControlInit();
	Chassis_Init();
	Gimbal_Init();
} 



void Init_Task(){

	for(;;){
		osDelay(1000);
	}

}
