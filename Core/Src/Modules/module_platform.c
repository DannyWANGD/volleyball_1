/*
 *  Project      :DeltaPlatform
 * 
 *  file         : module_platform.c
 *  Description  : This file contains DeltaPlatform control function
 *  LastEditors  : lkn
 *  Date         : 2024-7-2
 *  LastEditTime : 2024-7-3
 */
#include "sys_dwt.h"
#include <math.h>
 #include "module_communicate.h"
 #include "module_platform.h"
 #include "periph_motor.h"
 #include "sys_const.h"
 #include "alg_math.h"
 #include "cmsis_os.h"
 #include "app_remote.h"
 #include "periph_remote.h"
 
Platform_DataTypeDef Platform_Data;


  /**
  * @brief      Get the pointer of Platform control object
  * @param      NULL
  * @retval     Pointer to Platform control object
  */
Platform_DataTypeDef* Platform_GetPlatformPtr() {
    return &Platform_Data;
}

void Platform_Init() {
	
  	Platform_DataTypeDef *Platform = &Platform_Data;  
	
	Platform->output_state = Platform_slow;
	Platform->ctrl_mode = Platform_Initpose;
	Platform->fdb.front_pitch_angle = 0;
	Platform->fdb.front_yaw_angle = 0;
	Platform->fdb.left_pitch_angle = 0;
	Platform->fdb.left_yaw_angle = 0;
	Platform->fdb.right_pitch_angle = 0;
	Platform->fdb.right_yaw_angle = 0;
	
	Platform->fdb.plat_x = 0;
	Platform->fdb.plat_y = 0;
	Platform->fdb.plat_z = 0;
	Platform->fdb.plat_pitch = 0;
	Platform->fdb.plat_yaw = 0;
	Platform->fdb.plat_roll = 0;
	Platform->tar.plat_x = 0;
	Platform->tar.plat_y = 0;
	Platform->tar.plat_z = 0.12f;
	Platform->tar.plat_pitch = 0;
	Platform->tar.plat_yaw = 0;
	Platform->tar.plat_roll = 0;


	PID_InitPIDParam(&Platform->pid.Pitch_Ang_Fast_PIDParam,Const_Platform_Pitch_Ang_Param[0][0][0],Const_Platform_Pitch_Ang_Param[0][0][1],Const_Platform_Pitch_Ang_Param[0][0][2],Const_Platform_Pitch_Ang_Param[0][0][3],Const_Platform_Pitch_Ang_Param[0][0][4],
                                                     		Const_Platform_Pitch_Ang_Param[0][1][0],Const_Platform_Pitch_Ang_Param[0][1][1],
                                                    		Const_Platform_Pitch_Ang_Param[0][2][0],Const_Platform_Pitch_Ang_Param[0][2][1],
                                                     		Const_Platform_Pitch_Ang_Param[0][3][0],Const_Platform_Pitch_Ang_Param[0][3][1],PID_POSITION);
  	PID_InitPIDParam(&Platform->pid.Pitch_Spd_Fast_PIDParam,Const_Platform_Pitch_Spd_Param[0][0][0],Const_Platform_Pitch_Spd_Param[0][0][1],Const_Platform_Pitch_Spd_Param[0][0][2],Const_Platform_Pitch_Spd_Param[0][0][3],Const_Platform_Pitch_Spd_Param[0][0][4],
                                                     		Const_Platform_Pitch_Spd_Param[0][1][0],Const_Platform_Pitch_Spd_Param[0][1][1],
                                                     		Const_Platform_Pitch_Spd_Param[0][2][0],Const_Platform_Pitch_Spd_Param[0][2][1],
                                                     		Const_Platform_Pitch_Spd_Param[0][3][0],Const_Platform_Pitch_Spd_Param[0][3][1],PID_POSITION);
  	PID_InitPIDParam(&Platform->pid.Yaw_Ang_Fast_PIDParam,Const_Platform_Yaw_Ang_Param[0][0][0],Const_Platform_Yaw_Ang_Param[0][0][1],Const_Platform_Yaw_Ang_Param[0][0][2],Const_Platform_Yaw_Ang_Param[0][0][3],Const_Platform_Yaw_Ang_Param[0][0][4],
														  	Const_Platform_Yaw_Ang_Param[0][1][0],Const_Platform_Yaw_Ang_Param[0][1][1],
															Const_Platform_Yaw_Ang_Param[0][2][0],Const_Platform_Yaw_Ang_Param[0][2][1],
															Const_Platform_Yaw_Ang_Param[0][3][0],Const_Platform_Yaw_Ang_Param[0][3][1],PID_POSITION);
  	PID_InitPIDParam(&Platform->pid.Yaw_Spd_Fast_PIDParam,Const_Platform_Yaw_Spd_Param[0][0][0],Const_Platform_Yaw_Spd_Param[0][0][1],Const_Platform_Yaw_Spd_Param[0][0][2],Const_Platform_Yaw_Spd_Param[0][0][3],Const_Platform_Yaw_Spd_Param[0][0][4],
															Const_Platform_Yaw_Spd_Param[0][1][0],Const_Platform_Yaw_Spd_Param[0][1][1],
															Const_Platform_Yaw_Spd_Param[0][2][0],Const_Platform_Yaw_Spd_Param[0][2][1],
                                                     		Const_Platform_Yaw_Spd_Param[0][3][0],Const_Platform_Yaw_Spd_Param[0][3][1],PID_POSITION);
																
	PID_InitPIDParam(&Platform->pid.Pitch_Ang_Middle_PIDParam,Const_Platform_Pitch_Ang_Param[1][0][0],Const_Platform_Pitch_Ang_Param[1][0][1],Const_Platform_Pitch_Ang_Param[1][0][2],Const_Platform_Pitch_Ang_Param[1][0][3],Const_Platform_Pitch_Ang_Param[1][0][4],
															Const_Platform_Pitch_Ang_Param[1][1][0],Const_Platform_Pitch_Ang_Param[1][1][1],
															Const_Platform_Pitch_Ang_Param[1][2][0],Const_Platform_Pitch_Ang_Param[1][2][1],
															Const_Platform_Pitch_Ang_Param[1][3][0],Const_Platform_Pitch_Ang_Param[1][3][1],PID_POSITION);
  	PID_InitPIDParam(&Platform->pid.Pitch_Spd_Middle_PIDParam,Const_Platform_Pitch_Spd_Param[1][0][0],Const_Platform_Pitch_Spd_Param[1][0][1],Const_Platform_Pitch_Spd_Param[1][0][2],Const_Platform_Pitch_Spd_Param[1][0][3],Const_Platform_Pitch_Spd_Param[1][0][4],
                                                    		Const_Platform_Pitch_Spd_Param[1][1][0],Const_Platform_Pitch_Spd_Param[1][1][1],
                                                     		Const_Platform_Pitch_Spd_Param[1][2][0],Const_Platform_Pitch_Spd_Param[1][2][1],
                                                    		Const_Platform_Pitch_Spd_Param[1][3][0],Const_Platform_Pitch_Spd_Param[1][3][1],PID_POSITION);
  	PID_InitPIDParam(&Platform->pid.Yaw_Ang_Middle_PIDParam,Const_Platform_Yaw_Ang_Param[1][0][0],Const_Platform_Yaw_Ang_Param[1][0][1],Const_Platform_Yaw_Ang_Param[1][0][2],Const_Platform_Yaw_Ang_Param[1][0][3],Const_Platform_Yaw_Ang_Param[1][0][4],
                                                     	  	Const_Platform_Yaw_Ang_Param[1][1][0],Const_Platform_Yaw_Ang_Param[1][1][1],
                                                          	Const_Platform_Yaw_Ang_Param[1][2][0],Const_Platform_Yaw_Ang_Param[1][2][1],
                                                     	  	Const_Platform_Yaw_Ang_Param[1][3][0],Const_Platform_Yaw_Ang_Param[1][3][1],PID_POSITION);
 	PID_InitPIDParam(&Platform->pid.Yaw_Spd_Middle_PIDParam,Const_Platform_Yaw_Spd_Param[1][0][0],Const_Platform_Yaw_Spd_Param[1][0][1],Const_Platform_Yaw_Spd_Param[1][0][2],Const_Platform_Yaw_Spd_Param[1][0][3],Const_Platform_Yaw_Spd_Param[1][0][4],
                                                    		Const_Platform_Yaw_Spd_Param[1][1][0],Const_Platform_Yaw_Spd_Param[1][1][1],
                                                     		Const_Platform_Yaw_Spd_Param[1][2][0],Const_Platform_Yaw_Spd_Param[1][2][1],
                                                     		Const_Platform_Yaw_Spd_Param[1][3][0],Const_Platform_Yaw_Spd_Param[1][3][1],PID_POSITION);

  	PID_InitPIDParam(&Platform->pid.Pitch_Ang_Slow_PIDParam,Const_Platform_Pitch_Ang_Param[2][0][0],Const_Platform_Pitch_Ang_Param[2][0][1],Const_Platform_Pitch_Ang_Param[2][0][2],Const_Platform_Pitch_Ang_Param[2][0][3],Const_Platform_Pitch_Ang_Param[2][0][4],
                                                     		Const_Platform_Pitch_Ang_Param[2][1][0],Const_Platform_Pitch_Ang_Param[2][1][1],
                                                     		Const_Platform_Pitch_Ang_Param[2][2][0],Const_Platform_Pitch_Ang_Param[2][2][1],
                                                     		Const_Platform_Pitch_Ang_Param[2][3][0],Const_Platform_Pitch_Ang_Param[2][3][1],PID_POSITION);
  	PID_InitPIDParam(&Platform->pid.Pitch_Spd_Slow_PIDParam,Const_Platform_Pitch_Spd_Param[2][0][0],Const_Platform_Pitch_Spd_Param[2][0][1],Const_Platform_Pitch_Spd_Param[2][0][2],Const_Platform_Pitch_Spd_Param[2][0][3],Const_Platform_Pitch_Spd_Param[2][0][4],
                                                    		Const_Platform_Pitch_Spd_Param[2][1][0],Const_Platform_Pitch_Spd_Param[2][1][1],
                                                     		Const_Platform_Pitch_Spd_Param[2][2][0],Const_Platform_Pitch_Spd_Param[2][2][1],
                                                     		Const_Platform_Pitch_Spd_Param[2][3][0],Const_Platform_Pitch_Spd_Param[2][3][1],PID_POSITION);
  	PID_InitPIDParam(&Platform->pid.Yaw_Ang_Slow_PIDParam,Const_Platform_Yaw_Ang_Param[2][0][0],Const_Platform_Yaw_Ang_Param[2][0][1],Const_Platform_Yaw_Ang_Param[2][0][2],Const_Platform_Yaw_Ang_Param[2][0][3],Const_Platform_Yaw_Ang_Param[2][0][4],
                                                     		Const_Platform_Yaw_Ang_Param[2][1][0],Const_Platform_Yaw_Ang_Param[2][1][1],
                                                     		Const_Platform_Yaw_Ang_Param[2][2][0],Const_Platform_Yaw_Ang_Param[2][2][1],
                                                     		Const_Platform_Yaw_Ang_Param[2][3][0],Const_Platform_Yaw_Ang_Param[2][3][1],PID_POSITION);
  	PID_InitPIDParam(&Platform->pid.Yaw_Spd_Slow_PIDParam,Const_Platform_Yaw_Spd_Param[2][0][0],Const_Platform_Yaw_Spd_Param[2][0][1],Const_Platform_Yaw_Spd_Param[2][0][2],Const_Platform_Yaw_Spd_Param[2][0][3],Const_Platform_Yaw_Spd_Param[2][0][4],
                                                     		Const_Platform_Yaw_Spd_Param[2][1][0],Const_Platform_Yaw_Spd_Param[2][1][1],
                                                     		Const_Platform_Yaw_Spd_Param[2][2][0],Const_Platform_Yaw_Spd_Param[2][2][1],
                                                     		Const_Platform_Yaw_Spd_Param[2][3][0],Const_Platform_Yaw_Spd_Param[2][3][1],PID_POSITION);
																										 
	Platform->update_dt = 0;
	Platform->last_update_tick = DWT_GetTimeline_us();
	Platform->error_code = 0;


}

uint16_t watchdog2;

float start_hit_time = 0.0f;
float start_back_time = 0.0f;
uint8_t finishhit = 1; //击球动作的状态标志，0为击球中，1为击打已完成
float l_tar = 0.12f; //目标高度初始化为初始高度


// 新增：击球位姿控制相关变量
static uint8_t hitting_state = 0; // 0-空闲, 1-击球中, 2-回位中

uint8_t output_state;
uint64_t last_output_time;

// 从底层获取传感器数据，更新到主数据结构中
void Platform_Update_Fdb() {
	 
  	Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
	
	// 进行坐标系标定，处理零点偏移量
	Platform->fdb.front_pitch_angle = 29.1f-Motor_Pitch_Front_Motor.encoder.angle;
	Platform->fdb.left_pitch_angle  = 23.2f-Motor_Pitch_Left_Motor.encoder.angle;
  	Platform->fdb.right_pitch_angle = Motor_Pitch_Right_Motor.encoder.angle-48.5f;
  	Platform->fdb.front_yaw_angle   = Motor_Yaw_Front_Motor.encoder.angle ;
  	Platform->fdb.left_yaw_angle    = Motor_Yaw_Left_Motor.encoder.angle ;
  	Platform->fdb.right_yaw_angle   = Motor_Yaw_Right_Motor.encoder.angle+360.0f ;

	// 递增看门狗计数器。
	// 在其他地方（通常是CAN接收中断中），当收到对应电机的数据时，这个计数器会被清零。
	// 如果长时间未收到数据，计数值会持续累加，最终在 Platform_Check 中触发离线错误。
	Motor_Pitch_Front_Motor.watchdog += 1; 
	Motor_Pitch_Left_Motor.watchdog += 1; 
	Motor_Pitch_Right_Motor.watchdog += 1; 
	
	// 计算自上次调用以来经过的精确时间，为PID的微分项（D）提供准确的时间基准
  	Platform->update_dt = DWT_GetDeltaT(&Platform->last_update_tick);

}


// 安全监控函数，用于检测系统运行中的异常情况
// 在其他地方（通常是CAN接收中断中），当收到对应电机的数据时，这个计数器会被清零。
// 如果长时间未收到数据，计数值会持续累加，最终在 Platform_Check 中触发离线错误。
void Platform_Check() {
	
	Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
	if(Motor_Pitch_Front_Motor.watchdog>20)
		{Platform->error_code = 1;}
		if(Motor_Pitch_Left_Motor.watchdog>20)
			{Platform->error_code = 2;}	
		if(Motor_Pitch_Right_Motor.watchdog>20)
			{Platform->error_code = 3;}		
		if(fabsf(Motor_Pitch_Left_Motor.encoder.torque + Motor_Pitch_Right_Motor.encoder.torque)>20.0f)
		{watchdog2++;}
		else{watchdog2=0;}
	if(watchdog2>20){
			Platform->error_code = 4;
		}
	
// if(Platform->state==Platform_3degree && Motor_Pitch_Left_Motor.encoder.torque==0  && Motor_Pitch_Left_Motor.output!=0)
// {//Platform->error_code = 5;
// 	}
// if(Platform->state==Platform_3degree && Motor_Pitch_Right_Motor.encoder.torque==0 && Motor_Pitch_Right_Motor.output!=0)
// {//Platform->error_code = 6;
}

// 底层力矩输出接口
// 可调输出力矩上限（提升加速度能力，谨慎调高）
#define PITCH_TORQUE_LIMIT   26.0f
#define YAW_TORQUE_LIMIT     27.0f

void Platform_Set_Torque_Output(float torque1,float torque2,float torque3,float torque4,float torque5,float torque6){

    // 输出力矩限幅
    LimitMaxMin(torque1,PITCH_TORQUE_LIMIT,-PITCH_TORQUE_LIMIT);
    LimitMaxMin(torque2,PITCH_TORQUE_LIMIT,-PITCH_TORQUE_LIMIT);
    LimitMaxMin(torque3,PITCH_TORQUE_LIMIT,-PITCH_TORQUE_LIMIT);
    LimitMaxMin(torque4,YAW_TORQUE_LIMIT,-YAW_TORQUE_LIMIT);
    LimitMaxMin(torque5,YAW_TORQUE_LIMIT,-YAW_TORQUE_LIMIT);
    LimitMaxMin(torque6,YAW_TORQUE_LIMIT,-YAW_TORQUE_LIMIT);
	
	// 将力矩值写入电机对象的数据结构中，等待 Platform_Output 函数将其发送出去。
  	Motor_SetMotorOutput(&Motor_Pitch_Front_Motor,torque1) ;  
	Motor_SetMotorOutput(&Motor_Pitch_Left_Motor ,torque2) ; 
	Motor_SetMotorOutput(&Motor_Pitch_Right_Motor,torque3) ; 
	 
  	Motor_SetMotorOutput(&Motor_Yaw_Front_Motor,torque4) ;
  	Motor_SetMotorOutput(&Motor_Yaw_Left_Motor ,torque5) ;
  	Motor_SetMotorOutput(&Motor_Yaw_Right_Motor,torque6) ;

 }

// 实现位置-速度串级双环 PID 控制器，是整个闭环控制的核心算法
// 内部消除重复：使用宏封装串级计算，保持接口与行为不变
#define CASCADE_PITCH(ANG_PID, SPD_PID, ANG_PARAM, SPD_PARAM, SETPOINT, FDB_ANG, ENC_SPD, SIGN, BIAS, OUT_TORQUE) \
  do { \
    PID_SetPIDRef(&(ANG_PID), (SETPOINT)); \
    PID_SetPIDFdb(&(ANG_PID), (FDB_ANG)); \
    PID_CalcPID(&(ANG_PID), &(ANG_PARAM)); \
    PID_SetPIDRef(&(SPD_PID), PID_GetPIDOutput(&(ANG_PID))); \
    PID_SetPIDFdb(&(SPD_PID), (SIGN) * (ENC_SPD)); \
    PID_CalcPID(&(SPD_PID), &(SPD_PARAM)); \
    (OUT_TORQUE) = (SIGN) * PID_GetPIDOutput(&(SPD_PID)) + (BIAS); \
  } while (0)

#define CASCADE_YAW(ANG_PID, SPD_PID, ANG_PARAM, SPD_PARAM, SETPOINT, FDB_ANG, ENC_SPD, BIAS, OUT_TORQUE) \
  do { \
    PID_SetPIDRef(&(ANG_PID), (SETPOINT)); \
    PID_SetPIDFdb(&(ANG_PID), (FDB_ANG)); \
    PID_CalcPID(&(ANG_PID), &(ANG_PARAM)); \
    PID_SetPIDRef(&(SPD_PID), PID_GetPIDOutput(&(ANG_PID))); \
    PID_SetPIDFdb(&(SPD_PID), (ENC_SPD)); \
    PID_CalcPID(&(SPD_PID), &(SPD_PARAM)); \
    (OUT_TORQUE) = PID_GetPIDOutput(&(SPD_PID)) + (BIAS); \
  } while (0)

void Platform_Set_Angle_Output(float ang1,float ang2 ,float ang3,float ang4,float ang5,float ang6) {
	
  	Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();

	float torque1,torque2,torque3,torque4,torque5,torque6;
	LimitMaxMin(ang1,75.0f,-30.0f);
	LimitMaxMin(ang2,75.0f,-30.0f);
	LimitMaxMin(ang3,75.0f,-30.0f);
	LimitMaxMin(ang4,30.0f,-30.0f);
	LimitMaxMin(ang5,150.0f,90.0f);
	LimitMaxMin(ang6,270.0f,210.0f);
  
  switch (Platform->output_state) {
    case Platform_slow:
      CASCADE_PITCH(Platform->pid.Pitch1_Ang_PID, Platform->pid.Pitch1_Spd_PID, Platform->pid.Pitch_Ang_Slow_PIDParam, Platform->pid.Pitch_Spd_Slow_PIDParam,
                    ang1, Platform->fdb.front_pitch_angle, Motor_Pitch_Front_Motor.encoder.speed, -1.0f, -1.5f, torque1);
      CASCADE_PITCH(Platform->pid.Pitch2_Ang_PID, Platform->pid.Pitch2_Spd_PID, Platform->pid.Pitch_Ang_Slow_PIDParam, Platform->pid.Pitch_Spd_Slow_PIDParam,
                    ang2, Platform->fdb.left_pitch_angle, Motor_Pitch_Left_Motor.encoder.speed, -1.0f, -1.5f, torque2);
      CASCADE_PITCH(Platform->pid.Pitch3_Ang_PID, Platform->pid.Pitch3_Spd_PID, Platform->pid.Pitch_Ang_Slow_PIDParam, Platform->pid.Pitch_Spd_Slow_PIDParam,
                    ang3, Platform->fdb.right_pitch_angle, Motor_Pitch_Right_Motor.encoder.speed, 1.0f, 1.5f, torque3);

      CASCADE_YAW(Platform->pid.Yaw1_Ang_PID, Platform->pid.Yaw1_Spd_PID, Platform->pid.Yaw_Ang_Slow_PIDParam, Platform->pid.Yaw_Spd_Slow_PIDParam,
                  ang4, Platform->fdb.front_yaw_angle, Motor_Yaw_Front_Motor.encoder.speed, 0.0f, torque4);
      CASCADE_YAW(Platform->pid.Yaw2_Ang_PID, Platform->pid.Yaw2_Spd_PID, Platform->pid.Yaw_Ang_Slow_PIDParam, Platform->pid.Yaw_Spd_Slow_PIDParam,
                  ang5, Platform->fdb.left_yaw_angle, Motor_Yaw_Left_Motor.encoder.speed, 0.5f, torque5);
      CASCADE_YAW(Platform->pid.Yaw3_Ang_PID, Platform->pid.Yaw3_Spd_PID, Platform->pid.Yaw_Ang_Slow_PIDParam, Platform->pid.Yaw_Spd_Slow_PIDParam,
                  ang6, Platform->fdb.right_yaw_angle, Motor_Yaw_Right_Motor.encoder.speed, -0.5f, torque6);
      break;

    case Platform_middle:
      CASCADE_PITCH(Platform->pid.Pitch1_Ang_PID, Platform->pid.Pitch1_Spd_PID, Platform->pid.Pitch_Ang_Middle_PIDParam, Platform->pid.Pitch_Spd_Middle_PIDParam,
                    ang1, Platform->fdb.front_pitch_angle, Motor_Pitch_Front_Motor.encoder.speed, -1.0f, -1.5f, torque1);
      CASCADE_PITCH(Platform->pid.Pitch2_Ang_PID, Platform->pid.Pitch2_Spd_PID, Platform->pid.Pitch_Ang_Middle_PIDParam, Platform->pid.Pitch_Spd_Middle_PIDParam,
                    ang2, Platform->fdb.left_pitch_angle, Motor_Pitch_Left_Motor.encoder.speed, -1.0f, -1.5f, torque2);
      CASCADE_PITCH(Platform->pid.Pitch3_Ang_PID, Platform->pid.Pitch3_Spd_PID, Platform->pid.Pitch_Ang_Middle_PIDParam, Platform->pid.Pitch_Spd_Middle_PIDParam,
                    ang3, Platform->fdb.right_pitch_angle, Motor_Pitch_Right_Motor.encoder.speed, 1.0f, 1.5f, torque3);

      CASCADE_YAW(Platform->pid.Yaw1_Ang_PID, Platform->pid.Yaw1_Spd_PID, Platform->pid.Yaw_Ang_Middle_PIDParam, Platform->pid.Yaw_Spd_Middle_PIDParam,
                  ang4, Platform->fdb.front_yaw_angle, Motor_Yaw_Front_Motor.encoder.speed, 0.0f, torque4);
      CASCADE_YAW(Platform->pid.Yaw2_Ang_PID, Platform->pid.Yaw2_Spd_PID, Platform->pid.Yaw_Ang_Middle_PIDParam, Platform->pid.Yaw_Spd_Middle_PIDParam,
                  ang5, Platform->fdb.left_yaw_angle, Motor_Yaw_Left_Motor.encoder.speed, 0.5f, torque5);
      CASCADE_YAW(Platform->pid.Yaw3_Ang_PID, Platform->pid.Yaw3_Spd_PID, Platform->pid.Yaw_Ang_Middle_PIDParam, Platform->pid.Yaw_Spd_Middle_PIDParam,
                  ang6, Platform->fdb.right_yaw_angle, Motor_Yaw_Right_Motor.encoder.speed, -0.5f, torque6);
      break;

    case Platform_fast:
      CASCADE_PITCH(Platform->pid.Pitch1_Ang_PID, Platform->pid.Pitch1_Spd_PID, Platform->pid.Pitch_Ang_Fast_PIDParam, Platform->pid.Pitch_Spd_Fast_PIDParam,
                    ang1, Platform->fdb.front_pitch_angle, Motor_Pitch_Front_Motor.encoder.speed, -1.0f, -1.5f, torque1);
      CASCADE_PITCH(Platform->pid.Pitch2_Ang_PID, Platform->pid.Pitch2_Spd_PID, Platform->pid.Pitch_Ang_Fast_PIDParam, Platform->pid.Pitch_Spd_Fast_PIDParam,
                    ang2, Platform->fdb.left_pitch_angle, Motor_Pitch_Left_Motor.encoder.speed, -1.0f, -1.5f, torque2);
      CASCADE_PITCH(Platform->pid.Pitch3_Ang_PID, Platform->pid.Pitch3_Spd_PID, Platform->pid.Pitch_Ang_Fast_PIDParam, Platform->pid.Pitch_Spd_Fast_PIDParam,
                    ang3, Platform->fdb.right_pitch_angle, Motor_Pitch_Right_Motor.encoder.speed, 1.0f, 1.5f, torque3);

      CASCADE_YAW(Platform->pid.Yaw1_Ang_PID, Platform->pid.Yaw1_Spd_PID, Platform->pid.Yaw_Ang_Fast_PIDParam, Platform->pid.Yaw_Spd_Fast_PIDParam,
                  ang4, Platform->fdb.front_yaw_angle, Motor_Yaw_Front_Motor.encoder.speed, 0.0f, torque4);
      CASCADE_YAW(Platform->pid.Yaw2_Ang_PID, Platform->pid.Yaw2_Spd_PID, Platform->pid.Yaw_Ang_Fast_PIDParam, Platform->pid.Yaw_Spd_Fast_PIDParam,
                  ang5, Platform->fdb.left_yaw_angle, Motor_Yaw_Left_Motor.encoder.speed, 0.5f, torque5);
      CASCADE_YAW(Platform->pid.Yaw3_Ang_PID, Platform->pid.Yaw3_Spd_PID, Platform->pid.Yaw_Ang_Fast_PIDParam, Platform->pid.Yaw_Spd_Fast_PIDParam,
                  ang6, Platform->fdb.right_yaw_angle, Motor_Yaw_Right_Motor.encoder.speed, -0.5f, torque6);
      break;

    default:
      break;
  }
	// 将前面计算出的三个力矩值 torque1, torque2, torque3 发送给底层的电机驱动器，驱动器再将这些数值转换成对应的电流，施加到电机上，从而驱动平台运动。
  	Platform_Set_Torque_Output(torque1,torque2,torque3,torque4,torque5,torque6);

 }

// 上层应用向下层控制系统下达指令的标准化接口
void Platform_Set_Target_Pos(float x,float y,float z,float pitch,float yaw,float roll) {
	
	Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
 
	Platform->tar.plat_x = x;
	Platform->tar.plat_y = y;
	Platform->tar.plat_z = z;
	Platform->tar.plat_pitch = pitch;
	Platform->tar.plat_yaw = yaw;
	Platform->tar.plat_roll = roll;
	
	// 坐标目标值限幅
	LimitMaxMin(Platform->tar.plat_x,0.1f,-0.1f);
	LimitMaxMin(Platform->tar.plat_y,0.1f,-0.1f);
	LimitMaxMin(Platform->tar.plat_z,0.42f,0.10f);
	LimitMaxMin(Platform->tar.plat_pitch,10,-30);
	LimitMaxMin(Platform->tar.plat_yaw,30,-30);
	LimitMaxMin(Platform->tar.plat_roll,20,-20);

}

// 逆运动学解算【数学核心】
void Platform_Cal_3Degree_IK_Output() {
	
	Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();

	float z = Platform->tar.plat_z;
	float pi = Platform->tar.plat_pitch;
	float ro = Platform->tar.plat_roll;
		
	float L = Const_Platform_Big_Arm_Length;
	float l = Const_Platform_Small_Arm_Length;
	float R = Const_Platform_Static_Plat_Radius;
	float r = Const_Platform_Dynamic_Plat_Radius;
	float theta1 ,theta2,theta3;
	float sin_ro, cos_ro, sin_pi, cos_pi; 
	 
	arm_sin_cos_f32(ro, &sin_ro, &cos_ro); 
  	arm_sin_cos_f32(pi, &sin_pi, &cos_pi); 
	
	// 三自由度的解算代码，直接给底下的电机一个固定的转角值
  	theta1 = - acosf(((z * z * sin_ro * sin_ro + powf(r *cos_pi - R + z *cos_ro *sin_pi, 2) + L * L - l * l + powf(r *sin_pi - z *cos_pi *cos_ro, 2)) / (2 * L * sqrtf(z * z * sin_ro * sin_ro + powf(r *cos_pi - R + z *cos_ro *sin_pi, 2) + powf(r *sin_pi - z *cos_pi *cos_ro, 2))))) 
            - atanf((r *sin_pi - z *cos_pi *cos_ro) / sqrtf(z * z * sin_ro * sin_ro + powf(r *cos_pi - R + z *cos_ro *sin_pi, 2)));

  	theta2 = atanf(((r *sin_pi) / 2 + z *cos_pi *cos_ro + (sqrtf(3) * r *cos_pi * sin_ro) / 2) / sqrtf(powf((sqrtf(3) * R) / 2 + z * sin_ro - (sqrtf(3) * r *cos_ro) / 2, 2) + powf((R / 2 - (r *cos_pi) / 2 + z *cos_ro *sin_pi + (sqrtf(3) * r *sin_pi * sin_ro) / 2), 2))) 
			- acosf((powf((sqrtf(3) * R) / 2 + z * sin_ro - (sqrtf(3) * r *cos_ro) / 2, 2) + powf(R / 2 - (r *cos_pi) / 2 + z *cos_ro *sin_pi +(sqrtf(3) * r *sin_pi * sin_ro) / 2, 2) +powf((r *sin_pi) / 2 + z *cos_pi *cos_ro +(sqrtf(3) * r *cos_pi * sin_ro) / 2, 2) +L * L - l * l) /(2 * L * sqrtf(powf((sqrtf(3) * R) / 2 + z * sin_ro -(sqrtf(3) * r *cos_ro) / 2, 2) +powf(R / 2 - (r *cos_pi) / 2 + z *cos_ro *sin_pi +(sqrtf(3) * r *sin_pi * sin_ro) / 2, 2) +powf((r *sin_pi) / 2 + z *cos_pi *cos_ro +(sqrtf(3) * r *cos_pi * sin_ro) / 2, 2))));

  	theta3 = atanf(((r *sin_pi) / 2 + z *cos_pi *cos_ro - (sqrtf(3) * r *cos_pi * sin_ro) / 2) /sqrtf(powf(z * sin_ro - (sqrtf(3) * R) / 2 + (sqrtf(3) * r *cos_ro) / 2, 2) +      powf(R / 2 - (r *cos_pi) / 2 + z *cos_ro *sin_pi - (sqrtf(3) * r *sin_pi * sin_ro) / 2, 2))) 
           - acosf((powf(z * sin_ro - (sqrtf(3) * R) / 2 +(sqrtf(3) * r *cos_ro) / 2, 2) + powf(R / 2 - (r *cos_pi) / 2 + z *cos_ro *sin_pi -(sqrtf(3) * r *sin_pi * sin_ro) / 2, 2) +powf((r *sin_pi) / 2 + z *cos_pi *cos_ro -(sqrtf(3) * r *cos_pi * sin_ro) / 2, 2) +L * L - l * l) /(2 * L * sqrtf(powf(z * sin_ro - (sqrtf(3) * R) / 2 +(sqrtf(3) * r *cos_ro) / 2, 2) +powf(R / 2 - (r *cos_pi) / 2 + z *cos_ro *sin_pi -(sqrtf(3) * r *sin_pi * sin_ro) / 2, 2) +powf((r *sin_pi) / 2 + z *cos_pi *cos_ro -(sqrtf(3) * r *cos_pi * sin_ro) / 2, 2))));
	
	// 解算出的角度会直接赋给 Platform_Set_Angle_Output() 函数并启动闭环控制
  	Platform_Set_Angle_Output(rad2deg(theta1),rad2deg(theta2),rad2deg(theta3),0,120,240);

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
【注意】
这里是后期可以主要更改的地方，在此我们只是传入了一个 hit，即是否击球的标志位，
后面将会传入击球的期望空间位姿 pitch、yaw、roll 和 z 等参数，从而实现更复杂的击球策略。
【疑问】
我该如何解决当位姿一定时击球的空间速度呢？（或者说我该如何控制击球的力度呢？）
*/
// 轨迹平滑控制（Z 高度）——参数与状态
// 为最大化击球速度，提高全局轨迹限制 - 推至机械极限
static float l_traj_v = 0.0f;            // 高度指令的速度状态
static float l_traj_v_up_max = 10.0f;    // 上升最大速度（m/s）- 最大化至10m/s
static float l_traj_a_up_max = 15.0f;    // 上升最大加速度（m/s^2）- 高加速度
static float l_traj_v_down_max = 0.50f;  // 回落最大速度（m/s）- 适中回落速度
static float l_traj_a_down_max = 2.00f;  // 回落最大加速度（m/s^2）- 适中加速度
static float l_tar_goal = 0.08f;         // 目标高度（与 l_tar 同单位）
static const float l_tar_tol = 1e-3f;    // 目标收敛阈值

// 平滑轨迹更新：使 l_tar 以速度/加速度受限的方式逼近 goal_z
// 返回值：1-到达目标并锁定；0-尚未到达，已平滑前进
static int Platform_Update_Z_TrajSmooth(float goal_z, float dt, int is_rising)
{
    float a_max = is_rising ? l_traj_a_up_max   : l_traj_a_down_max;
    float v_max = is_rising ? l_traj_v_up_max   : l_traj_v_down_max;
    float e     = goal_z - l_tar;

    if (fabsf(e) > l_tar_tol) {
        // 防止到目标附近过冲：基于当前速度的制动距离判断
        float stop_dist = (l_traj_v * l_traj_v) / (2.0f * a_max);
        if (fabsf(e) <= stop_dist) {
            // 线性减速至零速
            if (l_traj_v > 0.0f) {
                l_traj_v -= a_max * dt;
                if (l_traj_v < 0.0f) l_traj_v = 0.0f;
            } else if (l_traj_v < 0.0f) {
                l_traj_v += a_max * dt;
                if (l_traj_v > 0.0f) l_traj_v = 0.0f;
            }
        } else {
            // 按误差方向加速（加速度限幅）
            float dir = (e > 0.0f) ? 1.0f : -1.0f;
            l_traj_v += dir * a_max * dt;
        }

        // 速度限幅
        if (l_traj_v >  v_max) l_traj_v =  v_max;
        if (l_traj_v < -v_max) l_traj_v = -v_max;

        // 位置积分
        l_tar += l_traj_v * dt;

        // 安全限幅（保持系统约束）：最低 Initial_Z_Position，最高 0.30f
        if (l_tar < Initial_Z_Position) l_tar = Initial_Z_Position;
        if (l_tar > 0.30f)              l_tar = 0.30f;

        return 0; // 尚未到达目标
    } else {
        l_tar    = goal_z;
        l_traj_v = 0.0f;
        return 1; // 已到达目标
    }
}

/**
 * @brief      Z 轴平滑轨迹（用户速度控制版）
 * @param      goal_z    目标高度 (m)
 * @param      dt        时间步长 (s)
 * @param      v_set     期望速度 (m/s)，取绝对值作为速度上限，自动按方向设置符号
 * @param      a_limit   最大加速度 (m/s^2)，用于限制速度变化与刹车
 * @retval     1 已到达目标并锁定，0 尚未到达（已按给定速度推进）
 * @note       与 Platform_Update_Z_TrajSmooth 相似，但允许外部直接指定速度与加速度，
 *             仍包含到目标附近的自动刹车与安全限幅，避免机械冲击。
 */
// Z 轴平滑轨迹的增强：更激进的加速与更晚刹车，放宽高度上限
#define PLATFORM_Z_MAX         0.37f
#define Z_BRAKE_GAIN           0.60f   // 刹车判据系数，值越小越晚刹车（更激进）
#define Z_ACCEL_BOOST_ERR      0.02f   // 误差阈值（m），超过则提升加速度（更早触发）
#define Z_ACCEL_BOOST_FACTOR   3.00f   // 加速度提升因子
#define Z_ACCEL_ABS_MAX        30.0f   // 加速度绝对上限（m/s^2）保护（匹配更高 a_limit）
int Platform_Update_Z_TrajWithUserSpeed(float goal_z, float dt, float v_set, float a_limit)
{
    if (dt <= 0.0f) {
        return 0;
    }
    // 保护输入参数
    float v_cap = fabsf(v_set);
    float a_cap = (a_limit > 1e-6f) ? a_limit : 1e-3f;

    float e = goal_z - l_tar;
    float abs_e = fabsf(e);

    // 远离目标时，动态提升加速度以更快贴近目标
    if (abs_e > Z_ACCEL_BOOST_ERR) {
        a_cap *= Z_ACCEL_BOOST_FACTOR;
        if (a_cap > Z_ACCEL_ABS_MAX) a_cap = Z_ACCEL_ABS_MAX;
    }

    if (fabsf(e) > l_tar_tol) {
        // 目标方向上的速度指令
        float v_cmd = (e > 0.0f) ? v_cap : -v_cap;

        // 根据当前速度的制动距离，接近目标时把速度指令改为 0 以线性刹停
        float stop_dist = (l_traj_v * l_traj_v) / (2.0f * a_cap);
        if (fabsf(e) <= stop_dist * Z_BRAKE_GAIN) {
            v_cmd = 0.0f; // 进入刹车段
        }

        // 以加速度上限将 l_traj_v 追向 v_cmd
        float dv = v_cmd - l_traj_v;
        float dv_max = a_cap * dt;
        if (dv >  dv_max) dv =  dv_max;
        if (dv < -dv_max) dv = -dv_max;
        l_traj_v += dv;

        // 速度上限：不超过用户设置
        if (l_traj_v >  v_cap) l_traj_v =  v_cap;
        if (l_traj_v < -v_cap) l_traj_v = -v_cap;

        // 位置积分
        l_tar += l_traj_v * dt;

        // 安全限幅（保持系统约束）：最低 Initial_Z_Position，最高 PLATFORM_Z_MAX
        if (l_tar < Initial_Z_Position) l_tar = Initial_Z_Position;
        if (l_tar > PLATFORM_Z_MAX)     l_tar = PLATFORM_Z_MAX;

        return 0; // 尚未到达目标
    } else {
        l_tar    = goal_z;
        l_traj_v = 0.0f;
        return 1; // 已到达目标
    }
}

// ========= 遥控器“丝滑”轨迹接口（角度与高度） =========
// 为遥控模式提供角度/高度的速度-加速度受限平滑控制，降低噪音与卡顿
// 速度/加速度上限（可根据机械能力调整）
// 提升响应：在安全前提下增大速度/加速度上限
#define REMOTE_Z_V_MAX      1.20f   // m/s   (原 0.80)
#define REMOTE_Z_A_MAX      6.00f   // m/s^2 (原 3.00)
#define REMOTE_PITCH_V_MAX  180.0f  // deg/s (原 120.0)
#define REMOTE_PITCH_A_MAX  900.0f  // deg/s^2 (原 600.0)
#define REMOTE_ROLL_V_MAX   240.0f  // deg/s (原 180.0)
#define REMOTE_ROLL_A_MAX   1100.0f // deg/s^2 (原 800.0)
#define REMOTE_YAW_V_MAX    200.0f  // deg/s (原 150.0)
#define REMOTE_YAW_A_MAX    900.0f  // deg/s^2 (原 600.0)

// 角度平滑更新：与 Z 的用户速度版同思路，支持任意角度（度）
static int Platform_Update_Angle_TrajWithUserSpeed(float goal_ang, float dt, float v_set, float a_limit,
                                                   float *ang_state, float *ang_traj_v,
                                                   float ang_min, float ang_max)
{
    if (dt <= 0.0f || ang_state == NULL || ang_traj_v == NULL) {
        return 0;
    }

    float v_cap = fabsf(v_set);
    float a_cap = (a_limit > 1e-6f) ? a_limit : 1e-3f;
    float e = goal_ang - *ang_state;
    const float tol = 1e-3f;

    if (fabsf(e) > tol) {
        // 目标方向速度指令
        float v_cmd = (e > 0.0f) ? v_cap : -v_cap;
        // 到目标附近的制动距离
        float stop_dist = ((*ang_traj_v) * (*ang_traj_v)) / (2.0f * a_cap);
        if (fabsf(e) <= stop_dist) {
            v_cmd = 0.0f;
        }
        // 以加速度上限追向 v_cmd
        float dv = v_cmd - (*ang_traj_v);
        float dv_max = a_cap * dt;
        if (dv >  dv_max) dv =  dv_max;
        if (dv < -dv_max) dv = -dv_max;
        *ang_traj_v += dv;

        // 速度限幅
        if (*ang_traj_v >  v_cap) *ang_traj_v =  v_cap;
        if (*ang_traj_v < -v_cap) *ang_traj_v = -v_cap;

        // 位置积分
        *ang_state += (*ang_traj_v) * dt;

        // 安全限幅
        if (*ang_state > ang_max) *ang_state = ang_max;
        if (*ang_state < ang_min) *ang_state = ang_min;

        return 0;
    } else {
        *ang_state  = goal_ang;
        *ang_traj_v = 0.0f;
        return 1;
    }
}

// 遥控角度轨迹速度状态
static float pitch_traj_v = 0.0f;
static float roll_traj_v  = 0.0f;
static float yaw_traj_v   = 0.0f;

// 对外接口：将目标位姿（来自遥控映射）转为丝滑的轨迹目标
void Platform_Set_Target_Pos_Smooth(float x, float y, float z_goal,
                                    float pitch_goal, float yaw_goal, float roll_goal)
{
    Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
    float dt = Platform->update_dt;
    if (dt <= 0.0f || dt > 0.05f) {
        dt = 0.005f; // 5ms 安全步长
    }

    // 高度：沿用 Z 轴用户速度版，保证平滑与安全限幅
    (void)Platform_Update_Z_TrajWithUserSpeed(z_goal, dt, REMOTE_Z_V_MAX, REMOTE_Z_A_MAX);

    // 角度：按速度/加速度上限做平滑推进（单位：度）
    (void)Platform_Update_Angle_TrajWithUserSpeed(pitch_goal, dt, REMOTE_PITCH_V_MAX, REMOTE_PITCH_A_MAX,
                                                  &Platform->tar.plat_pitch, &pitch_traj_v, -30.0f, 30.0f);
    // (void)Platform_Update_Angle_TrajWithUserSpeed(yaw_goal,   dt, REMOTE_YAW_V_MAX,   REMOTE_YAW_A_MAX,
    //                                               &Platform->tar.plat_yaw,   &yaw_traj_v,   -30.0f, 30.0f);
    (void)Platform_Update_Angle_TrajWithUserSpeed(roll_goal,  dt, REMOTE_ROLL_V_MAX,  REMOTE_ROLL_A_MAX,
                                                  &Platform->tar.plat_roll,  &roll_traj_v,  -45.0f, 45.0f);

    // 更新目标位姿（X/Y 保持原接口习惯）
    Platform->tar.plat_x = x;
    Platform->tar.plat_y = y;
    Platform->tar.plat_z = l_tar; // 已由 Z 平滑函数更新
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 接发球调度函数
void Platform_Jiefa_Cal(float hit) {
  
    Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
    // 使用系统已计算的精确 dt（秒），若异常则给一个保守默认值
    float dt = Platform->update_dt;
    if (dt <= 0.0f || dt > 0.05f) {
        dt = 0.005f; // 5ms 安全步长
    }

    // 内嵌接口：使用用户速度/加速度版本进行平滑控制
    // 说明：为了将 "Platform_Update_Z_TrajWithUserSpeed" 的接口整合到本函数中，
    // 原始对外声明已在头文件中注释；此处作为内部调用使用。
    // 最大化上升速度配置 - 推至机械极限但保持安全
    const float v_set_up   = 20.0f; // 上升阶段速度（m/s）- 最大化速度
    const float a_limit_up = 30.0f; // 上升阶段加速度（m/s^2）- 高加速度快速响应
    const float v_set_down = 0.30f; // 回落阶段速度（m/s）- 稍快回落
    const float a_limit_dn = 1.50f; // 回落阶段加速度（m/s^2）- 适中加速度

    if (finishhit == 0 ){

        if(DWT_GetTimeline_s() - start_hit_time  <= 1.0f) { // 击球阶段，缩短至0.8秒，更快响应
            // 上升阶段：以用户速度/加速度进行平滑逼近上限
            l_tar_goal = 0.30f; // 最大行程
			Platform_Set_OutputState(Platform_fast);
            (void)Platform_Update_Z_TrajWithUserSpeed(l_tar_goal, dt, v_set_up, a_limit_up);
            // 使用当前目标姿态
            Platform_Set_Target_Pos(0,0,l_tar, Platform->tar.plat_pitch, Platform->tar.plat_yaw, Platform->tar.plat_roll);
            // 击球上升默认快速档
            
            hitting_state = 1;
        } else {
            // 回落阶段：采用更慢的速度与加速度做平滑下落
            Platform_Set_OutputState(Platform_slow);
            l_tar_goal = Initial_Z_Position;
            int reached = Platform_Update_Z_TrajWithUserSpeed(l_tar_goal, dt, v_set_down, a_limit_dn);
            if (reached) {
                // 到达目标：锁定到初始高度，并进入冷却计时
                start_back_time = DWT_GetTimeline_s();
                finishhit = 1;
                hitting_state = 2;
            } else {
                Platform_Set_Target_Pos(0,0,l_tar, Platform->tar.plat_pitch, Platform->tar.plat_yaw, Platform->tar.plat_roll);
                hitting_state = 2;
            }
        }

    } else if (finishhit == 1) {

		if(DWT_GetTimeline_s() - start_back_time <= 2.0f){ // 从返回到现在是否未超过2.0秒，保持“冷却状态”
			Platform_Set_OutputState(Platform_slow);
            hitting_state = 2;
        } else {
            if((int)hit == 1) {
                // 确保平台在初始位置
                Platform_Set_Target_Pos(0,0,Initial_Z_Position,0,0,0);
                Platform_Set_OutputState(Platform_middle);
                start_hit_time = DWT_GetTimeline_s();
                // 击球开始时，轨迹速度清零，目标设定为上限，避免初始抖动
                l_traj_v = 0.0f;  // 从零开始加速，确保最大加速度
                l_tar_goal = 0.37f;  // 直接设定为最大允许高度
                finishhit = 0;
                hitting_state = 1;
            } else if ((int)hit != 1) {
                Platform_Set_Target_Pos(0,0,Initial_Z_Position,0,0,0);
                Platform_Set_OutputState(Platform_middle);
                hitting_state = 0;
            }
	  	}
	}
};

// 颠球调度函数（传入distance，且时间参数如 0.8s 和轨迹参数如 +=0.05 不同）
void Platform_Dianqiu_Cal(float hit, uint8_t distance) {
	
	Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();

	if (finishhit == 0 ){
		
		if(DWT_GetTimeline_s() - start_hit_time  <= 0.8f) {
			
			switch (distance){
				
				case Remote_SWITCH_UP://jin
					l_tar +=0.05;
					LimitMax (l_tar,0.3f);
					Platform_Set_Target_Pos(0,0,l_tar,-15.0f,0,0);
					Platform_Set_OutputState(Platform_middle); 
					break ;
			    case Remote_SWITCH_MIDDLE:
					l_tar +=0.05;
					LimitMax (l_tar,0.3f);
					Platform_Set_Target_Pos(0,0,l_tar,0,0,0);
					Platform_Set_OutputState(Platform_middle); 
				  	break ;
			  	case Remote_SWITCH_DOWN://yuan
					l_tar +=0.05;
					LimitMax (l_tar,0.3f);
					Platform_Set_Target_Pos(0,0,l_tar,0,0,0);
					Platform_Set_OutputState(Platform_fast); 
				  	break;
		 	}
		} else {
			Platform_Set_OutputState(Platform_slow);	
			Platform_Set_Target_Pos(0,0,0.13f,0,0,0);
		  	start_back_time = DWT_GetTimeline_s();	
		  	finishhit = 1;	
	  }
 	} else if (finishhit == 1) {
	 
		if(DWT_GetTimeline_s() - start_back_time <= 1.0f) { // 从返回到现在是否未超过1.5秒，保持“冷却状态”
			Platform_Set_OutputState(Platform_slow);
		} else {
		  	if((int)hit == 1) {
		  		// 确保平台在初始位置
			  	Platform_Set_Target_Pos(0,0,0.13f,0,0,0);
		    	Platform_Set_OutputState(Platform_middle);
				start_hit_time = DWT_GetTimeline_s();
				finishhit = 0;
			} else if((int)hit != 1) {
				Platform_Set_Target_Pos(0,0,0.13f,0,0,0);
			  	Platform_Set_OutputState(Platform_middle);
			}
	  	}
	}	
};

// 设置控制模式 mode
void Platform_Set_ControlMode(uint8_t mode) {
  	Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
  	Platform->ctrl_mode = mode;
}

// 设置输出状态 state
void Platform_Set_OutputState(uint8_t state) {
  	Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
  	Platform->output_state = state;
}

// 顶层状态机
void Platform_Control() {
	
  	Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
  	switch (Platform->ctrl_mode) {
		case Platform_Test:
			break;
		case Platform_Dianqiu:
		  	Platform_Cal_3Degree_IK_Output();
      		break;
		case Platform_Jiefa:
			Platform_Cal_3Degree_IK_Output(); 
			break;				
		case Platform_Chuanqiu:
			Platform_Set_OutputState(Platform_fast); 
			Platform_Cal_3Degree_IK_Output(); 
			break;				
		case Platform_Initpose: // 复位模式
			start_hit_time = 0.0f;
			start_back_time = 0.0f;
			finishhit = 1;
			l_tar = 0.12f;	
			Platform_Set_OutputState(Platform_slow);  
			Platform_Set_Target_Pos(0,0,0.12f,0,0,0);  
			Platform_Cal_3Degree_IK_Output(); 
			break;
		case Platform_Stop:
			Platform_Set_OutputState(Platform_stop);
			Platform_Set_Torque_Output(0,0,0,0,0,0);
			break;				
		default:
			break;
	}
}

// 输出分组节流配置：用于在发送不同电机组数据之间留出时间间隔，避免总线/电源瞬时拥塞。
// 为最大化击球响应速度，最小化延时 - 设置为50us并启用阻塞模式以确保命令顺序执行
#ifndef PLATFORM_OUTPUT_GAP_US
#define PLATFORM_OUTPUT_GAP_US 80U  // 最小化延时，提高响应速度
#endif
#ifndef PLATFORM_OUTPUT_BLOCKING
#define PLATFORM_OUTPUT_BLOCKING 1  // 保持阻塞以确保命令顺序
#endif

// 为三个电极发送数据
void Platform_Output() {
    Platform_DataTypeDef *Platform = Platform_GetPlatformPtr(); 
        
//while( (output_state !=1) && (DWT_GetTimeline_us()-last_output_time < 150)){}
    Motor_SendMotorGroupOutput(Motor_groupHandle[3]) ; 
    Motor_SendMotorGroupOutput(Motor_groupHandle[0]) ;
    // last_output_time = DWT_GetTimeline_us();
    #if PLATFORM_OUTPUT_BLOCKING
    DWT_Delayus(PLATFORM_OUTPUT_GAP_US);
    #endif
    
//while( (output_state !=2) && (DWT_GetTimeline_us()-last_output_time < 150)){}
    Motor_SendMotorGroupOutput(Motor_groupHandle[4]) ;
    Motor_SendMotorGroupOutput(Motor_groupHandle[1]) ;
    // last_output_time = DWT_GetTimeline_us();
    #if PLATFORM_OUTPUT_BLOCKING
    DWT_Delayus(PLATFORM_OUTPUT_GAP_US);
    #endif

//while( (output_state !=3) && (DWT_GetTimeline_us()-last_output_time < 150)){}
    Motor_SendMotorGroupOutput(Motor_groupHandle[5]) ;
    Motor_SendMotorGroupOutput(Motor_groupHandle[2]) ;
    // last_output_time = DWT_GetTimeline_us();
    
  }



/**
 * @brief      击球位姿控制状态机（需要在主循环中调用）
 * @param      None
 * @retval     None
 * @note       管理击球动作的完整流程：击球 -> 保持 -> 回位 -> 冷却
 */
// 已简化：更新循环统一到 Platform_Jiefa_Cal

/**
 * @brief      获取当前击球状态
 * @param      None
 * @retval     击球状态 (0-空闲, 1-击球中, 2-回位中)
 */
uint8_t Platform_Get_Hitting_State(void) {
    return hitting_state;
}

/**
 * @brief      重置击球状态到空闲
 * @param      None
 * @retval     None
 */
void Platform_Reset_Hitting_State(void) {
    hitting_state = 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
