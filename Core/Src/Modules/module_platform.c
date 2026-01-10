/*
 *  Project      :DeltaPlatform
 * 
 *  file         : module_platform.c
 *  Description  : This file contains DeltaPlatform control function
 *  LastEditors  : wd
 *  Date         : 2025-11-24
 *  LastEditTime : 2025-11-24
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

// 前置声明（在初始化与更新入口中使用）
 static inline void Z_ApplyProfileAggressive(void);
 static inline void Z_ApplyProfileSafe(void);
// Z 轴高度安全上限（保留）
 float PLATFORM_Z_MAX = 0.28f;

// 轨迹平滑控制（Z 高度）——参数与状态（简化版：统一配置 + 兼容宏）
typedef struct {
    float v_up_max;       // 上升最大速度
    float a_up_max;       // 上升最大加速度
    float ret_ratio;      // 下降阶段占比（用于速度和加速度派生）
    float brake_gain;     // 刹车判据系数
    float boost_err;      // 触发加速增强的误差阈值（m）
    float boost_gain;     // 加速增强因子
    float accel_abs_max;  // 加速度绝对上限（保护）
} ZTrajProfile;

// 不在这里初始化赋值，由 Z_ApplyProfileAggressive/Z_ApplyProfileSafe 在运行时统一赋值
ZTrajProfile z_profile = {
    0.0f,   // v_up_max
    0.0f,   // a_up_max
    0.0f,   // ret_ratio
    0.0f,   // brake_gain
    0.0f,  // boost_err
    0.0f,   // boost_gain
    0.0f   // accel_abs_max
};

//20251205 激进版本
// ZTrajProfile z_profile = {
//     20.0f,   // v_up_max
//     20.0f,   // a_up_max
//     0.01f,   // ret_ratio
//     0.10f,   // brake_gain
//     0.05f,  // boost_err
//     4.5f,   // boost_gain
//     35.0f   // accel_abs_max
// };

// ========= 开环加速模式配置（Open-Loop Boost Mode）=========
// 原理：初始阶段直接输出最大扭矩加速，接近顶点时切换PID减速
typedef struct {
    uint8_t  enabled;           // 模式使能
    float    pid_switch_dist;   // 切换到PID的距离阈值（距顶点，单位m）
    float    max_torque;        // 开环最大扭矩（Nm）
    uint8_t  in_openloop;       // 当前是否处于开环加速状态
} OpenLoopBoostConfig;

static OpenLoopBoostConfig boost_cfg = {
    .enabled         = 1,       // 默认启用
    .pid_switch_dist = 0.05f,   // 距顶点5cm时切换到PID减速
    .max_torque      = 15.0f,   // 最大扭矩
    .in_openloop     = 0
};

// 20251118 阶段性参数调整
// ZTrajProfile z_profile = {
//     35.0f,   // v_up_max
//     20.0f,   // a_up_max
//     0.01f,   // ret_ratio
//     0.10f,   // brake_gain
//     0.05f,  // boost_err
//     4.8f,   // boost_gain
//     47.0f   // accel_abs_max
// };

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
	Platform->tar.plat_z = 0.08f;
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

    Z_ApplyProfileSafe();
    Platform_Reset_OpenLoop_Boost(); // 初始化开环加速状态


}

uint16_t watchdog2;

float start_hit_time = 0.0f;
float start_back_time = 0.0f;
uint8_t finishhit = 1; //击球动作的状态标志，0为击球中，1为击打已完成
float l_tar = 0.12f; //目标高度初始化为初始高度


// 新增：击球位姿控制相关变量
typedef enum { STATE_IDLE = 0, STATE_RISING = 1, STATE_RETURNING = 2, STATE_COOLDOWN = 3 } HitState_t;
static HitState_t hit_state = STATE_IDLE;

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
#define PITCH_TORQUE_LIMIT   27.0f
#define YAW_TORQUE_LIMIT     27.0f

/**
 * @brief      开环加速模式：直接输出最大扭矩（绕过PID）
 * @param      direction  加速方向（1=向上推，-1=向下）
 * @retval     None
 * @note       用于初始阶段快速加速，充分利用电机峰值扭矩
 */
void Platform_OpenLoop_Boost_Output(int8_t direction) {
    float boost_torque = boost_cfg.max_torque * direction;
    // Pitch电机全力输出（考虑电机安装方向）
    // 根据坐标系：front和left是负向，right是正向
    Motor_SetMotorOutput(&Motor_Pitch_Front_Motor, -boost_torque - 1.5f);
    Motor_SetMotorOutput(&Motor_Pitch_Left_Motor,  -boost_torque - 1.5f);
    Motor_SetMotorOutput(&Motor_Pitch_Right_Motor,  boost_torque + 1.5f);
    // Yaw电机保持位置（小扭矩）
    Motor_SetMotorOutput(&Motor_Yaw_Front_Motor, 0.0f);
    Motor_SetMotorOutput(&Motor_Yaw_Left_Motor,  0.5f);
    Motor_SetMotorOutput(&Motor_Yaw_Right_Motor, -0.5f);
}

/**
 * @brief      检查是否应使用开环加速（距顶点较远时开环，接近时切PID）
 * @param      current_z  当前Z高度
 * @param      target_z   目标Z高度（顶点）
 * @retval     1=开环加速中，0=使用PID控制
 */
uint8_t Platform_Check_OpenLoop_Boost(float current_z, float target_z) {
    if (!boost_cfg.enabled) return 0;
    
    float dist_to_top = target_z - current_z;
    
    // 距离顶点大于阈值 → 开环加速
    if (dist_to_top > boost_cfg.pid_switch_dist) {
        boost_cfg.in_openloop = 1;
        Platform_OpenLoop_Boost_Output(1); // 向上加速
        return 1;
    } else {
        // 接近顶点 → 切换到PID减速
        boost_cfg.in_openloop = 0;
        return 0;
    }
}

/** @brief 重置开环加速状态（用于新的击球周期） */
void Platform_Reset_OpenLoop_Boost(void) {
    boost_cfg.in_openloop = 0;
}

/** @brief 设置开环加速参数（供外部调参） */
void Platform_Set_Boost_Params(float pid_switch_dist, float max_torque) {
    boost_cfg.pid_switch_dist = pid_switch_dist;
    boost_cfg.max_torque = max_torque;
}

/** @brief 启用/禁用开环加速模式 */
void Platform_Enable_OpenLoop_Boost(uint8_t enable) {
    boost_cfg.enabled = enable;
}

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

float torque1,torque2,torque3,torque4,torque5,torque6;

void Platform_Set_Angle_Output(float ang1,float ang2 ,float ang3,float ang4,float ang5,float ang6) {
	
  	Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
	
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
	LimitMaxMin(Platform->tar.plat_z,PLATFORM_Z_MAX,Initial_Z_Position);
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


// 保留速度状态和目标
static float l_traj_v   = 0.0f;   // 高度指令的速度状态
static float l_tar_goal = 0.12f;  // 目标高度（与 l_tar 同单位）
static const float l_tar_tol = 1e-3f;    // 目标收敛阈值
static float hit_pitch_goal = 0.0f;
static float hit_roll_goal  = 0.0f;

// 已移除 legacy 参数，直接使用 z_profile 与派生函数

// 便捷接口：一键切换“激进/保守”档
// static inline void Z_ApplyProfileAggressive(void) {
//     z_profile.v_up_max      = 10.0f;
//     z_profile.a_up_max      = 5.0f;
//     z_profile.ret_ratio     = 0.30f;
//     z_profile.brake_gain    = 0.6f;  // 略晚刹车
//     z_profile.boost_err     = 0.10;  // 更早触发增强
//     z_profile.boost_gain    = 2.0f;
//     z_profile.accel_abs_max = 25.0f;  // 若电机/结构允许
// }
static inline void Z_ApplyProfileSafe(void) {
    z_profile.v_up_max      = 20.0f;
    z_profile.a_up_max      = 40.0f;
    z_profile.ret_ratio     = 0.01f;
    z_profile.brake_gain    = 0.10f;
    z_profile.boost_err     = 0.05f;
    z_profile.boost_gain    = 1.00f;
    z_profile.accel_abs_max = 20.0f;
}

/**
 * @brief      通用平滑轨迹更新器（位置/角度）
 * @param      goal      目标值（高度 m 或角度 deg）
 * @param      dt        时间步长（秒）
 * @param      v_set     期望最大速度（绝对值为上限，符号由误差方向决定）
 * @param      a_limit   期望最大加速度
 * @param      state     待更新的状态指针（Z 高度或角度）
 * @param      traj_v    对应的轨迹速度状态指针
 * @param      min/max   状态安全范围（限幅）
 * @param      tol       收敛阈值（误差小于该值视为到达）
 * @param      is_z      是否为 Z 轴：启用上升加速增强与下降提前刹车
 * @retval     1 到达目标（锁定并清零轨迹速度），0 未到达（按限速推进）
 * @note       统一实现：加速度受限追速、接近目标线性刹停、速度与状态限幅。
 *             当 is_z=1 时，远离目标的上升方向会提升加速度（受绝对上限保护），
 *             下降方向会加大刹车增益，避免机械下砸。
 */
static inline int Traj_Update(float goal, float dt, float v_set, float a_limit,
                              float *state, float *traj_v,
                              float min, float max, float tol, int is_z)
{
    // 输入参数检查
    if (dt <= 0.0f || state == NULL || traj_v == NULL) return 0;
    // 期望最大速度限幅，确保非零
    float v_cap = fabsf(v_set);
    if (v_cap < 1e-6f) v_cap = 1e-3f;
    // 期望最大加速度限幅，确保非零
    float a_cap = (a_limit > 1e-6f) ? a_limit : 1e-3f;
    // 计算当前误差（目标 - 当前状态）
    float e = goal - (*state);
    float abs_e = fabsf(e);

    // Z 轴仅在“偏差较大且为上升方向”时提升加速度，加快贴近目标
    if (is_z && abs_e > z_profile.boost_err && e > 0.0f) {
        a_cap *= z_profile.boost_gain;
        if (a_cap > z_profile.accel_abs_max) a_cap = z_profile.accel_abs_max;
    }
    if (is_z && goal > (*state)) {
        if (a_cap < z_profile.accel_abs_max) a_cap = z_profile.accel_abs_max;
    }
    
    // 处于未到达阶段：以加速度受限将速度追向目标方向
    if (fabsf(e) > tol) {
        float v_cmd = (e > 0.0f) ? v_cap : -v_cap;
        // 接近目标时的线性刹停（根据当前速度的制动距离）
        float stop_dist = ((*traj_v) * (*traj_v)) / (2.0f * a_cap);
        float brake_k = is_z ? z_profile.brake_gain : 1.0f;

        // Z 轴下降提前进入刹车段，防止“下砸”
        if (is_z && e < 0.0f) brake_k *= 2.0f;
        // 当偏差小于刹车距离时，以最大刹车加速度 dt 步长停止
        if (fabsf(e) <= stop_dist * brake_k) v_cmd = 0.0f;

        // 以最大加速度 dt 步长逼近期望速度
        float dv = v_cmd - (*traj_v);

        // 限制最大加速度 dt 步长，避免超速
        float dv_max = a_cap * dt;
        if (dv >  dv_max) dv =  dv_max;
        if (dv < -dv_max) dv = -dv_max;

        *traj_v += dv; // 速度以最大加速度逐步逼近期望

        // 速度限幅，保证不超过期望速度上限
        if (*traj_v >  v_cap) *traj_v =  v_cap;
        if (*traj_v < -v_cap) *traj_v = -v_cap;

        // 位置/角度积分，更新当前状态
        *state += (*traj_v) * dt;

        // 安全范围限幅，避免越界
        if (*state < min) *state = min;
        if (*state > max) *state = max;
        return 0;
    } else {
        // 进入到达阶段：锁定目标并清零轨迹速度
        *state  = goal;
        *traj_v = 0.0f;
        return 1;
    }
}

// removed: use Traj_Update directly

// ========= 遥控器“丝滑”轨迹接口（角度与高度） =========
// 为遥控模式提供角度/高度的速度-加速度受限平滑控制，降低噪音与卡顿
// 速度/加速度上限（可根据机械能力调整）
// 提升响应：在安全前提下增大速度/加速度上限
#define REMOTE_Z_V_MAX      1.20f   // m/s   (原 0.80)
#define REMOTE_Z_A_MAX      6.00f   // m/s^2 (原 3.00)
#define REMOTE_PITCH_V_MAX  360.0f  // deg/s (原 120.0)
#define REMOTE_PITCH_A_MAX  900.0f  // deg/s^2 (原 600.0)
#define REMOTE_ROLL_V_MAX   240.0f  // deg/s (原 180.0)
#define REMOTE_ROLL_A_MAX   1100.0f // deg/s^2 (原 800.0)
#define REMOTE_YAW_V_MAX    200.0f  // deg/s (原 150.0)
#define REMOTE_YAW_A_MAX    900.0f  // deg/s^2 (原 600.0)

// 角度平滑更新：与 Z 的用户速度版同思路，支持任意角度（度）
// removed: use Traj_Update directly

// 遥控角度轨迹速度状态
static float pitch_traj_v = 0.0f;
static float roll_traj_v  = 0.0f;
static float yaw_traj_v   = 0.0f;

static inline void AngleSmoothUpdate(float dt,
                                     float pitch_goal, float roll_goal,
                                     float v_pitch, float a_pitch,
                                     float v_roll,  float a_roll)
{
    Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
    (void)Traj_Update(pitch_goal, dt, v_pitch, a_pitch,
                      &Platform->tar.plat_pitch, &pitch_traj_v,
                      -30.0f, 30.0f, 1e-3f, 0);
    (void)Traj_Update(roll_goal,  dt, v_roll,  a_roll,
                      &Platform->tar.plat_roll,  &roll_traj_v,
                      -45.0f, 45.0f, 1e-3f, 0);
}

static inline void WriteTargetFromState(void)
{
    Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
    Platform_Set_Target_Pos(0,0,l_tar, Platform->tar.plat_pitch, Platform->tar.plat_yaw, Platform->tar.plat_roll);
}

static inline int AnglesReady(float pitch_goal, float roll_goal, float ang_tol, float v_ang_near_zero)
{
    Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
    int pitch_ok = (fabsf(pitch_goal - Platform->tar.plat_pitch) <= ang_tol) || (fabsf(pitch_traj_v) <= v_ang_near_zero);
    int roll_ok  = (fabsf(roll_goal  - Platform->tar.plat_roll)  <= ang_tol) || (fabsf(roll_traj_v)  <= v_ang_near_zero);
    return pitch_ok && roll_ok;
}

// 对外接口：将目标位姿转为丝滑的轨迹目标
// void Platform_Set_Target_Pos_Smooth(float x, float y, float z_goal,
//                                     float pitch_goal, float yaw_goal, float roll_goal)
// {
//     Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
//     float dt = Platform->update_dt;
//     if (dt <= 0.0f || dt > 0.05f) {
//         dt = 1.0f; // 5ms 安全步长
//     }

//     (void)Traj_Update(z_goal, dt, REMOTE_Z_V_MAX, REMOTE_Z_A_MAX,
//                       &l_tar, &l_traj_v,
//                       Initial_Z_Position, PLATFORM_Z_MAX,
//                       l_tar_tol, 1);

//     // 角度：按速度/加速度上限做平滑推进（单位：度）
//     (void)Traj_Update(pitch_goal, dt, REMOTE_PITCH_V_MAX, REMOTE_PITCH_A_MAX,
//                       &Platform->tar.plat_pitch, &pitch_traj_v, -30.0f, 30.0f, 1e-3f, 0);
//     // (void)Platform_Update_Angle_TrajWithUserSpeed(yaw_goal,   dt, REMOTE_YAW_V_MAX,   REMOTE_YAW_A_MAX,
//     //                                               &Platform->tar.plat_yaw,   &yaw_traj_v,   -30.0f, 30.0f);
//     (void)Traj_Update(roll_goal,  dt, REMOTE_ROLL_V_MAX,  REMOTE_ROLL_A_MAX,
//                       &Platform->tar.plat_roll,  &roll_traj_v,  -45.0f, 45.0f, 1e-3f, 0);

//     // 更新目标位姿（X/Y 保持原接口习惯）
//     Platform->tar.plat_x = x;
//     Platform->tar.plat_y = y;
//     Platform->tar.plat_z = l_tar; // 已由 Z 平滑函数更新
// }

// 封装：激进/平稳两套丝滑接口（切档后调用统一 Smooth）
// void Platform_Set_Target_Pos_Smooth_Aggressive(float x,float y,float z,float pitch,float yaw,float roll)
// {
//     Z_ApplyProfileAggressive();
//     Platform_Set_Target_Pos_Smooth(x,y,z,pitch,yaw,roll);
// }

// void Platform_Set_Target_Pos_Smooth_Stable(float x,float y,float z,float pitch,float yaw,float roll)
// {
//     Z_ApplyProfileSafe();
//     Platform_Set_Target_Pos_Smooth(x,y,z,pitch,yaw,roll);
// }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 接发球调度函数
void Platform_Jiefa_Cal(float hit) {
  
    Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
    // 使用系统已计算的精确 dt（秒），若异常则给一个保守默认值
    float dt = Platform->update_dt;
    if (dt <= 0.0f || dt > 0.05f) {
        dt = 1.0f; // 5ms 安全步长
    }

    // 内嵌接口：使用用户速度/加速度版本进行平滑控制
    // 说明：为了将 "Platform_Update_Z_TrajWithUserSpeed" 的接口整合到本函数中，
    // 原始对外声明已在头文件中注释；此处作为内部调用使用。
    // 统一使用配置化参数（z_profile 派生）
    const float v_set_up   = z_profile.v_up_max;
    const float a_limit_up = z_profile.a_up_max;
    const float v_set_down = z_profile.v_up_max * z_profile.ret_ratio;
    const float a_limit_dn =  z_profile.a_up_max * z_profile.ret_ratio;

    // 判据阈值（无需时间窗口）
    const float z_top_tol     = 0.001f;   // 距离上限的收敛阈值
    const float v_near_zero   = 0.01f;   // 认为速度近零的阈值（m/s）
    const float ang_tol       = 3.0f; // 认为角度近零的阈值（度）
    const float v_ang_near_zero = 3.0f; // 认为角度近零的阈值（度/s）

    if (hit_state == STATE_RISING) {
            l_tar_goal = PLATFORM_Z_MAX;
            Platform_Set_OutputState(Platform_fast);
            
            // 想要回退回到正常的击打逻辑中，注释这个区间中的代码即可
            // -----------------------------------------------------------------------------
            // ===== 开环加速检测：距顶点较远时直接输出最大扭矩 =====
            if (Platform_Check_OpenLoop_Boost(l_tar, PLATFORM_Z_MAX)) {
                // 开环加速中：更新轨迹位置（用于状态跟踪），但扭矩已由开环函数输出
                l_tar += l_traj_v * dt + 0.5f * z_profile.a_up_max * dt * dt; // 估算位置
                l_traj_v += z_profile.a_up_max * dt; // 估算速度
                if (l_tar > PLATFORM_Z_MAX - boost_cfg.pid_switch_dist) {
                    l_tar = PLATFORM_Z_MAX - boost_cfg.pid_switch_dist; // 限幅
                }
                // 角度仍按平滑推进
                AngleSmoothUpdate(dt, hit_pitch_goal, hit_roll_goal,
                                  REMOTE_PITCH_V_MAX, REMOTE_PITCH_A_MAX,
                                  REMOTE_ROLL_V_MAX,  REMOTE_ROLL_A_MAX);
                WriteTargetFromState();
                return; // 直接返回，不执行后续PID控制
            }
            // ===== 接近顶点：使用PID减速控制 =====
            // -----------------------------------------------------------------------------


            // Z 高度推进（加速度受限 + 接近顶端线性刹停）
            int reached_up_z = Traj_Update(PLATFORM_Z_MAX, dt, v_set_up, a_limit_up,
                                           &l_tar, &l_traj_v,
                                           Initial_Z_Position, PLATFORM_Z_MAX,
                                           l_tar_tol, 1);
            // 角度推进（速度/加速度受限）、写入目标
            AngleSmoothUpdate(dt, hit_pitch_goal, hit_roll_goal,
                              REMOTE_PITCH_V_MAX, REMOTE_PITCH_A_MAX,
                              REMOTE_ROLL_V_MAX,  REMOTE_ROLL_A_MAX);
            WriteTargetFromState();

            // 上顶误差与刹车判据：进入制动距离后使速度指令为零以线性刹停
            float e_top = PLATFORM_Z_MAX - l_tar;
            float stop_dist_up = (l_traj_v * l_traj_v) / (2.0f * a_limit_up);
            int entering_brake = (fabsf(e_top) <= stop_dist_up * z_profile.brake_gain);
            // 满足 Z 到顶/接近/速度近零/进入刹车段 → 切换到回落（避免等待角度导致卡死）
            if (reached_up_z || (fabsf(e_top) <= z_top_tol) || (fabsf(l_traj_v) <= v_near_zero) || entering_brake) {
                hit_state = STATE_RETURNING;
            }
    } else if (hit_state == STATE_RETURNING) {
            Platform_Reset_OpenLoop_Boost(); // 重置开环加速状态
            Platform_Set_OutputState(Platform_slow);
            const float scale = z_profile.ret_ratio;
            // Z 回落推进（加速度受限 + 下降提前刹车在 Traj_Update 内部实现）
            (void)Traj_Update(Initial_Z_Position, dt, v_set_down, a_limit_dn,
                              &l_tar, &l_traj_v,
                              Initial_Z_Position, PLATFORM_Z_MAX,
                              l_tar_tol, 1);
            // 角度回落推进（按 ret_ratio 缩放的速度/加速度）
            AngleSmoothUpdate(dt, 0.0f, 0.0f,
                              REMOTE_PITCH_V_MAX * scale, REMOTE_PITCH_A_MAX * scale,
                              REMOTE_ROLL_V_MAX  * scale, REMOTE_ROLL_A_MAX  * scale);
            WriteTargetFromState();
            // Z 回落完成：位置接近初始且速度近零
            int reached_dn_z = (fabsf(Initial_Z_Position - l_tar) <= z_top_tol) && (fabsf(l_traj_v) <= v_near_zero);
            
            // 角度回落完成：接近 0 度或角速度近零
            int pitch_back = (fabsf(Platform->tar.plat_pitch) <= ang_tol) || (fabsf(pitch_traj_v) <= v_ang_near_zero);
            int roll_back  = (fabsf(Platform->tar.plat_roll)  <= ang_tol) || (fabsf(roll_traj_v)  <= v_ang_near_zero);
            // 三项均满足 → 标记击球完成，进入冷却
            if (reached_dn_z && pitch_back && roll_back) {
                start_back_time = DWT_GetTimeline_s();
                finishhit = 1;
                hit_state = STATE_COOLDOWN;
            }
    } else {
        float cooldown_elapsed = DWT_GetTimeline_s() - start_back_time;
        int trigger = ((int)hit > 0.8f && (int)hit < 1.2f);
        if (hit_state == STATE_COOLDOWN) {
            if (cooldown_elapsed >= 2.0f && trigger) {
                Platform_Set_OutputState(Platform_slow);
                Platform_Set_Target_Pos(0,0,Initial_Z_Position,0,0,0);
                l_traj_v = 0.0f;
                l_tar_goal = PLATFORM_Z_MAX;
                pitch_traj_v = 0.0f;
                roll_traj_v  = 0.0f;
                finishhit = 0;
                hit_state = STATE_RISING;
            } else {
                Platform_Set_Target_Pos(0,0,Initial_Z_Position,0,0,0);
                Platform_Set_OutputState(Platform_middle);
            }
        } else {
            if (trigger && cooldown_elapsed >= 2.0f) {
                Platform_Set_OutputState(Platform_slow);
                Platform_Set_Target_Pos(0,0,Initial_Z_Position,0,0,0);
                l_traj_v = 0.0f;
                l_tar_goal = PLATFORM_Z_MAX;
                pitch_traj_v = 0.0f;
                roll_traj_v  = 0.0f;
                finishhit = 0;
                hit_state = STATE_RISING;
            } else {
                Platform_Set_Target_Pos(0,0,Initial_Z_Position,0,0,0);
                Platform_Set_OutputState(Platform_middle);
                hit_state = STATE_IDLE;
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

void Platform_Set_Hit_Pose(float pitch, float roll) {
    hit_pitch_goal = pitch;
    hit_roll_goal  = roll;
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

            // 想要回退回到正常的击打逻辑中，注释这个区间中的代码即可
            // -----------------------------------------------------------------------------
            // 开环加速阶段：避免IK+PID覆盖开环满扭矩输出
            if (boost_cfg.enabled && boost_cfg.in_openloop) {
                Platform_OpenLoop_Boost_Output(1);
                break;
            }
            // ----------------------------------------------------------------------------
            
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
			l_tar = Initial_Z_Position;	
			Platform_Set_OutputState(Platform_slow);  
			Platform_Set_Target_Pos(0,0,Initial_Z_Position,0,0,0);  
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
    return (uint8_t)hit_state;
}

/**
 * @brief      重置击球状态到空闲
 * @param      None
 * @retval     None
 */
void Platform_Reset_Hitting_State(void) {
    hit_state = STATE_IDLE;
}

/////////////////////////////////////////////////////////////////////////////////////////
