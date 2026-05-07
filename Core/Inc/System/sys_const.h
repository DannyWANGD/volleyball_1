/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : sys_const.h
 *  Description  : This file include all required constants
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:21:16
 *  LastEditTime : 2024-01-09 14:59:58
 */


#ifndef SYS_CONST_H
#define SYS_CONST_H

#ifdef __cplusplus
extern "C" {
#endif 
#include "util_uart.h"

extern const float root3 ;
extern const float Initial_Z_Position ;

extern const  float Const_FRONT_PITCH_MOTOR_INIT_OFFSET  ;
extern const  float Const_LEFT_PITCH_MOTOR_INIT_OFFSET   ;
extern const  float Const_RIGHT_PITCH_MOTOR_INIT_OFFSET  ;
extern const  float Const_FRONT_YAW_MOTOR_INIT_OFFSET    ;
extern const  float Const_LEFT_YAW_MOTOR_INIT_OFFSET     ;
extern const  float Const_RIGHT_YAW_MOTOR_INIT_OFFSET    ;

extern const float Const_A1_Pitch_Ang_Param[4][5];
extern const float Const_A1_Pitch_Spd_Param[4][5];
extern const float Const_A1_Init_Position[3];

extern const float Const_LeftPosMotorParam[4][5];
extern const float Const_LeftSpdMotorParam[4][5];
extern const float Const_RightPosMotorParam[4][5];
extern const float Const_RightSpdMotorParam[4][5];

extern const float Const_YawPosMotorParam[4][5];
extern const float Const_YawSpdMotorParam[4][5];
extern const float Const_PitchPosMotorParam[4][5];
extern const float Const_PitchSpdMotorParam[4][5];

extern const  float Const_DMmotor_P_MIN    ;
extern const  float Const_DMmotor_P_MAX    ;
extern const  float Const_DMmotor_V_MIN    ;
extern const  float Const_DMmotor_V_MAX    ;
extern const  float Const_DMmotor_KP_MIN  ; 
extern const  float Const_DMmotor_KP_MAX   ;
extern const  float Const_DMmotor_KD_MIN  ; 
extern const  float Const_DMmotor_KD_MAX  ; 
extern const  float Const_DMmotor_T_MIN   ; 
extern const  float Const_DMmotor_T_MAX   ; 
extern const  float Const_Platform_Dynamic_R;
extern const  float Const_Platform_Static_R;
extern const  float Const_Platform_Static_L;
extern const  float Const_Platform_Dynamic_L;
extern const float Const_Platform_Big_Arm_Length  ;
extern const float Const_Platform_Small_Arm_Length  ;
extern const float Const_Platform_Static_Plat_Radius  ;
extern const float Const_Platform_Dynamic_Plat_Radius ;

extern const float Const_Chassis_Location_Distance_Param[4][5];
extern const float Const_Chassis_Location_Anglew_Param[4][5];
extern const float Const_Platform_Pitch_Spd_Param[3][4][5];
extern const float Const_Platform_Pitch_Ang_Param[3][4][5];
extern const float Const_Platform_Yaw_Spd_Param[3][4][5];
extern const float Const_Platform_Yaw_Ang_Param[3][4][5];


extern const  float Const_Shoulder_Motor_Offset;
extern const  float Const_Elbow_Motor_Offset;
extern const  float Const_Wrist_Motor_Offset;


extern const float Const_L1 ;
extern const float Const_L2 ;
extern const float Const_L3 ;

extern const float Const_Init_Osang ;
extern const float Const_Init_L ;
extern const float Const_Init_Theta ;
 
 extern const float Curr2ForceConstant_3508;
extern const float Roll2SpeedConstant_3508;
extern const float ReductionRate;
extern const float Fact2Ctrl_3508;

extern const float CtrlCurrent_3508;
extern const float FactCurrent_3508;

extern UART_HandleTypeDef* Const_Remote_UART_HANDLER;
extern UART_HandleTypeDef* Const_Communicate_UART_HANDLER;
extern UART_HandleTypeDef* Const_Motor_UART_HANDLER;
extern UART_HandleTypeDef* Const_DT35_UART_HANDLER;
extern const float Const_Remote_REMOTE_OFFLINE_TIME;
extern const float Const_Comm_OFFLINE_TIME;     

extern const float Const_Gimbal_Yaw_OFFSET;
extern const float Const_Gimbal_Pitch_OFFSET;

extern const uint16_t Remote_RockerMin;
extern const uint16_t Remote_RockerMiddle;
extern const uint16_t Remote_RockerMax;
#endif

#ifdef __cplusplus
}
#endif
