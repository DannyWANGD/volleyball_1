/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : sys_const.c
 *  Description  : This file include all required constants
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:20:29
 *  LastEditTime : 2024-01-09 14:57:59
 */


#include "sys_const.h"

const float root3 = 1.73205;
const float Initial_Z_Position = 0.08f;

const  float Const_DMmotor_P_MIN  = -12.5  ;
const  float Const_DMmotor_P_MAX   =12.5   ;
const  float Const_DMmotor_V_MIN   =-45    ;
const  float Const_DMmotor_V_MAX  = 45     ;
const  float Const_DMmotor_KP_MIN = 0     ; 
const  float Const_DMmotor_KP_MAX  =500    ;
const  float Const_DMmotor_KD_MIN  =0     ; 
const  float Const_DMmotor_KD_MAX  =5     ; 
const  float Const_DMmotor_T_MIN  = -30   ; 
const  float Const_DMmotor_T_MAX   =30  ; 


const float Const_Chassis_Location_Distance_Param[4][5] = 
{{0.04f, 0.0f, 0.0f, 1, 3}, {0.1f, -1}, {0, 0}, {-1, -1}};

const float Const_Chassis_Location_Anglew_Param[4][5] = 
{{0.06f, 0.0f, 0.0f, 1, 3}, {0.1f, -1}, {0, 0}, {-1, -1}};
			
const float Const_Platform_Pitch_Ang_Param[3][4][5] = {    
  {{1.2f, 0.01f, 15.0f, 120.0f, 40.0f},{0.1f, -1}, {0, 0}, {-1, -1}},                   //0fast 1middle 2slow
  {{0.9f, 0.01f, 10.0f, 120.0f, 14.0f},{0.1f, -1}, {0, 0}, {-1, -1}},
  {{0.8f, 0.01f, 8.0f , 8.0f , 10.0f },{0.1f, -1}, {0, 0}, {-1, -1}}
};

const float Const_Platform_Pitch_Spd_Param[3][4][5] = {
	{{0.65f, 0.01f, 0.0f, 18.0f, 24.0f},{0.1f, -1}, {0, 0}, {-1, -1}},
	{{0.8f, 0.01f, 0.0f, 100.0f, 24.0f},{0.1f, -1}, {0, 0}, {-1, -1}},
	{{0.6f , 0.01f, 0.0f, 15.0f, 20.0f},{0.1f, -1}, {0, 0}, {-1, -1}},
};

const float Const_Platform_Yaw_Ang_Param[3][4][5] = {
	{{2.0f, 0.03f, 0.5f, 50.0f, 70.0f},{0.1f, -1}, {0, 0}, {-1, -1}},
    {{2.0f, 0.03f, 0.5f, 50.0f, 70.0f},{0.1f, -1}, {0, 0}, {-1, -1}},
	{{1.8f, 0.03f, 0.5f, 50.0f, 70.0f},{0.1f, -1}, {0, 0}, {-1, -1}},
 };
 
 const float Const_Platform_Yaw_Spd_Param[3][4][5] = {
	{{1.3f, 0.003f, 0.0f, 20.0f, 27.0f},{0.1f, -1}, {0, 0}, {-1, -1}},
    {{1.3f, 0.003f, 0.0f, 20.0f, 27.0f},{0.1f, -1}, {0, 0}, {-1, -1}},
	{{1.1f, 0.002f, 0.0f, 20.0f, 27.0f},{0.1f, -1}, {0, 0}, {-1, -1}},
 };


const float Const_Platform_Big_Arm_Length = 0.25 ;
const float Const_Platform_Small_Arm_Length = 0.2 ;
const float Const_Platform_Static_Plat_Radius = 0.065 ;
const float Const_Platform_Dynamic_Plat_Radius = 0.25 ;



const float Curr2ForceConstant_3508 = 0.3;
const float Roll2SpeedConstant_3508 = 24.48;
const float ReductionRate = 3591 / 187;
const float CtrlCurrent_3508 = 16384;
const float FactCurrent_3508 = 20;
const float Fact2Ctrl_3508 = 16384 / 20;

UART_HandleTypeDef* Const_Remote_UART_HANDLER       = &huart1;
UART_HandleTypeDef* Const_Communicate_UART_HANDLER  = &huart7;
UART_HandleTypeDef* Const_Motor_UART_HANDLER        = &huart6;
UART_HandleTypeDef* Const_DT35_UART_HANDLER         = &huart8;
const float Const_Remote_REMOTE_OFFLINE_TIME        = 1.0f;
const float Const_Comm_OFFLINE_TIME                 = 0.5f;
