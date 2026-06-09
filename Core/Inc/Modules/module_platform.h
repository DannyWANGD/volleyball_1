/*  Project      : Polaris
 * 
 *  file         : module_platform.h
 *  Description  : This file contains Platform control function
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-10-04 17:53:57
 */

#ifndef MODULE_PLATFORM_H
#define MODULE_PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "periph_motor.h"
#include "alg_pid.h"

// 输出状态枚举
typedef enum
{
	Platform_slow = 1,
	Platform_fast,
	Platform_middle,
	Platform_stop
}Platform_Output_StateEnum;

// 控制模式枚举
typedef enum
{
	Platform_Jiefa = 1,
	Platform_Chuanqiu =2,
	Platform_Dianqiu =3,
  Platform_Initpose =4,	
	Platform_Test =5,
  Platform_Stop =6,
}Platform_Ctrl_ModeEnum;

// 击球板数据反馈结构体
typedef struct
{
  float front_pitch_angle, front_yaw_angle, left_pitch_angle,left_yaw_angle, right_pitch_angle,right_yaw_angle;

  float plat_x;
  float plat_y;
  float plat_z;
  float plat_pitch;
  float plat_yaw;
  float plat_roll;
}Platform_FeedbackTypeDef;

// 击球板目标结构体
typedef struct
{
  float plat_x,plat_y,plat_z;
  float plat_pitch,plat_yaw,plat_roll;
  float front_pitch_torque,left_pitch_torque,right_pitch_torque;
  float front_yaw_torque,left_yaw_torque,right_yaw_torque;

}Platform_TargetTypeDef;

// PID参数结构体
typedef struct
{
  PID_PIDTypeDef Pitch1_Ang_PID;
  PID_PIDTypeDef Pitch2_Ang_PID;
  PID_PIDTypeDef Pitch3_Ang_PID;
  PID_PIDParamTypeDef Pitch_Ang_Fast_PIDParam;
	PID_PIDParamTypeDef Pitch_Ang_Middle_PIDParam;
	PID_PIDParamTypeDef Pitch_Ang_Slow_PIDParam;
	
  PID_PIDTypeDef Pitch1_Spd_PID;
  PID_PIDTypeDef Pitch2_Spd_PID;
  PID_PIDTypeDef Pitch3_Spd_PID;
  PID_PIDParamTypeDef Pitch_Spd_Fast_PIDParam;
	PID_PIDParamTypeDef Pitch_Spd_Middle_PIDParam;
	PID_PIDParamTypeDef Pitch_Spd_Slow_PIDParam;

  PID_PIDTypeDef Yaw1_Ang_PID;
  PID_PIDTypeDef Yaw2_Ang_PID;
  PID_PIDTypeDef Yaw3_Ang_PID;
  PID_PIDParamTypeDef Yaw_Ang_Fast_PIDParam;
  PID_PIDParamTypeDef Yaw_Ang_Middle_PIDParam;
	PID_PIDParamTypeDef Yaw_Ang_Slow_PIDParam;
	
  PID_PIDTypeDef Yaw1_Spd_PID;
  PID_PIDTypeDef Yaw2_Spd_PID;
  PID_PIDTypeDef Yaw3_Spd_PID;
  PID_PIDParamTypeDef Yaw_Spd_Fast_PIDParam;
  PID_PIDParamTypeDef Yaw_Spd_Middle_PIDParam;
	PID_PIDParamTypeDef Yaw_Spd_Slow_PIDParam;
	
}Platform_PIDTypeDef;

// 平台数据结构体
typedef struct {

  Platform_Output_StateEnum  output_state; //输出状态
  Platform_Ctrl_ModeEnum ctrl_mode; //控制模式
  Platform_FeedbackTypeDef fdb; //反馈数据
  Platform_TargetTypeDef tar; //目标数据
  Platform_PIDTypeDef pid; //PID参数

	float flag;
  float update_dt; //更新周期
  uint32_t last_update_tick; //上次更新时刻
  uint8_t error_code; //错误代码

} Platform_DataTypeDef;

typedef enum { STATE_IDLE = 0, STATE_RISING = 1, STATE_BRAKING = 2, STATE_RETURNING = 3, STATE_COOLDOWN = 4 } HitState_t;

extern Platform_DataTypeDef Platform_Data;
extern HitState_t hit_state;
extern float keep_state_flag;
Platform_DataTypeDef* Platform_GetPlatformPtr(void);
void Platform_Init();
void Platform_Update_Fdb();
void Platform_Set_Angle_Output(float ang1,float ang2 ,float ang3 ,float ang4,float ang5,float ang6);
void Platform_Set_ControlMode(uint8_t mode) ;
void Platform_Set_OutputState(uint8_t state) ;
void Platform_Set_Target_Pos(float x,float y,float z,float pitch,float yaw,float roll) ;
// 遥控/人机接口：丝滑轨迹版目标位姿设置（速度/加速度受限）
// void Platform_Set_Target_Pos_Smooth(float x,float y,float z,float pitch,float yaw,float roll) ;
// 封装：激进/平稳两套轨迹接口（内部切换 z_profile）
void Platform_Set_Target_Pos_Smooth_Aggressive(float x,float y,float z,float pitch,float yaw,float roll);
void Platform_Set_Target_Pos_Smooth_Stable(float x,float y,float z,float pitch,float yaw,float roll);
void Platform_Control(void);
void Platform_Output(void);
void Platform_Cal_3Degree_IK_Output() ;
void Platform_Set_Torque_Output(float torque1,float torque2,float torque3,float torque4,float torque5,float torque6);
void Platform_Dianqiu_Cal(float hit,uint8_t distance);
void Platform_Jiefa_Cal(float hit);
void Platform_Check();

// 开环加速模式接口（用于调参和控制）
void Platform_Set_Boost_Params(float pid_switch_dist, float max_torque);
void Platform_Enable_OpenLoop_Boost(uint8_t enable);
void Platform_Reset_OpenLoop_Boost(void);

// 击球位姿控制接口
void Platform_Set_Hit_Pose(float pitch, float roll);

// 简化接口：姿态设置请调用 Platform_Set_Target_Pos，击球触发调用 Platform_Jiefa_Cal(1.0f)
 // Deprecated external interface: now internally used by Platform_Jiefa_Cal
 // Keep for reference, but do not expose externally.
 // int Platform_Update_Z_TrajWithUserSpeed(float goal_z, float dt, float v_set, float a_limit);

void A1_Init(void);
void A1_Position_Control(void);

uint8_t Platform_Get_Hitting_State(void);
void Platform_Reset_Hitting_State(void);
#endif

#ifdef __cplusplus
}
#endif
