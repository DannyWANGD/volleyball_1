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
HitState_t hit_state = STATE_IDLE;

// 前置声明（在初始化与更新入口中使用）
 static inline void Z_ApplyProfileAggressive(void);
 static inline void Z_ApplyProfileSafe(void);
// Z 轴高度安全上限（保留）
float PLATFORM_Z_MAX = 0.25f;
//#define PITCH_TORQUE_LIMIT   30.0f
//#define YAW_TORQUE_LIMIT     30.0f

#define PITCH_TORQUE_LIMIT   30.0f
#define YAW_TORQUE_LIMIT     30.0f

// 纠偏增益：如果偏差 1 度（约 0.017rad），输出 0.85Nm 纠偏力矩
// 调试时：若没效果就加大，若剧烈抖动就减小
#define K_TWIST_COMP  8.0f
// 轨迹平滑控制（Z 高度）——参数与状态（简化版：统一配置 + 兼容宏）
typedef struct {
    float v_up_max;       // 上升最大速度
    float a_up_max;       // 上升最大加速度
    float ret_ratio;      // 下降阶段占比（用于速度和加速度派生）
    float brake_gain;     // 刹车判据系数
    float boost_err;      // 触发加速增强的误差阈值（m）
    float boost_gain;     // 加速增强因子
} ZTrajProfile;

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
    .pid_switch_dist = 0.15f,   // 距顶点5cm时切换到PID减速
    .max_torque      = 20.0f,   // 最大扭矩
    .in_openloop     = 0
};

  /**
  * @brief      Get the pointer of Platform control object
  * @param      NULL
  * @retval     Pointer to Platform control object
  */
Platform_DataTypeDef* Platform_GetPlatformPtr() {
    return &Platform_Data;
}

// --------------------------------------------------------------------
// A1电机控制代码区间 BEGIN
// --------------------------------------------------------------------


// static PID_PIDTypeDef A1_Pitch_Ang_PID[3];  // Front=0, Left=1, Right=2
// static PID_PIDTypeDef A1_Pitch_Spd_PID[3];
// static PID_PIDParamTypeDef A1_Ang_PIDParam;
// static PID_PIDParamTypeDef A1_Spd_PIDParam;

// A1 目标角度
// static float a1_target_angle[3] = {0.0f, 0.0f, 0.0f};

// void A1_Init(void) {
//     // 初始化角度环 PID 参数
//     PID_InitPIDParam(&A1_Ang_PIDParam,
//         Const_A1_Pitch_Ang_Param[0][0], Const_A1_Pitch_Ang_Param[0][1], 
//         Const_A1_Pitch_Ang_Param[0][2], Const_A1_Pitch_Ang_Param[0][3], 
//         Const_A1_Pitch_Ang_Param[0][4],
//         Const_A1_Pitch_Ang_Param[1][0], Const_A1_Pitch_Ang_Param[1][1],
//         Const_A1_Pitch_Ang_Param[2][0], Const_A1_Pitch_Ang_Param[2][1],
//         Const_A1_Pitch_Ang_Param[3][0], Const_A1_Pitch_Ang_Param[3][1],
//         PID_POSITION);
    
//     // 初始化速度环 PID 参数
//     PID_InitPIDParam(&A1_Spd_PIDParam,
//         Const_A1_Pitch_Spd_Param[0][0], Const_A1_Pitch_Spd_Param[0][1], 
//         Const_A1_Pitch_Spd_Param[0][2], Const_A1_Pitch_Spd_Param[0][3], 
//         Const_A1_Pitch_Spd_Param[0][4],
//         Const_A1_Pitch_Spd_Param[1][0], Const_A1_Pitch_Spd_Param[1][1],
//         Const_A1_Pitch_Spd_Param[2][0], Const_A1_Pitch_Spd_Param[2][1],
//         Const_A1_Pitch_Spd_Param[3][0], Const_A1_Pitch_Spd_Param[3][1],
//         PID_POSITION);
    
//     // 设置目标位置 (从常量表读取)
//     a1_target_angle[0] = Const_A1_Init_Position[0];
//     a1_target_angle[1] = Const_A1_Init_Position[1];
//     a1_target_angle[2] = Const_A1_Init_Position[2];

// }


// --------------------------------------------------------------------
// A1电机控制代码区间 END
// --------------------------------------------------------------------


void Platform_Init() {
	
  	Platform_DataTypeDef *Platform = &Platform_Data;  
	
	Platform->output_state = Platform_slow;
	Platform->ctrl_mode = Platform_Initpose;
    hit_state = STATE_IDLE;
    Platform->flag = 0;
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
	Platform->tar.plat_z = 0.01f;
	Platform->tar.plat_pitch = 0;
	Platform->tar.plat_yaw = 0;
	Platform->tar.plat_roll = 0;

	Platform->flag = 0;

	PID_InitPIDParam(&Platform->pid.Pitch_Ang_Fast_PIDParam,
                    Const_Platform_Pitch_Ang_Param[0][0][0], // kp
                    Const_Platform_Pitch_Ang_Param[0][0][1], // ki
                    Const_Platform_Pitch_Ang_Param[0][0][2], // kd
                    Const_Platform_Pitch_Ang_Param[0][0][3], // sum_max
                    Const_Platform_Pitch_Ang_Param[0][0][4], // output_max
                    Const_Platform_Pitch_Ang_Param[0][1][0], // kd_fil_param
                    Const_Platform_Pitch_Ang_Param[0][1][1], // delta_fil_param                                               		
                    Const_Platform_Pitch_Ang_Param[0][2][0], // kf_1 静态前馈系数
                    Const_Platform_Pitch_Ang_Param[0][2][1], // kf_2 动态前馈系数                                                   		
                    Const_Platform_Pitch_Ang_Param[0][3][0], // kf1_fil_param
                    Const_Platform_Pitch_Ang_Param[0][3][1], // kf2_fil_param
                    PID_POSITION);
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
    // A1_Init();
}

uint16_t watchdog2;

uint8_t output_state;
uint64_t last_output_time;

// 从底层获取传感器数据，更新到主数据结构中
void Platform_Update_Fdb() {
	 
  	Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
	
	// 进行坐标系标定，处理零点偏移量
    // A1 电机角度反馈（尝试将 Right 也取反，与 Front/Left 一致）
    Platform->fdb.front_pitch_angle = -58.0f + Motor_Pitch_Front_Motor_A1.encoder.angle;
		Platform->fdb.left_pitch_angle  = -63.0f + Motor_Pitch_Left_Motor_A1.encoder.angle;
  	Platform->fdb.right_pitch_angle = -69.0f + Motor_Pitch_Right_Motor_A1.encoder.angle;
//	    Platform->fdb.front_pitch_angle = -58.0f + Motor_Pitch_Front_Motor_A1.encoder.angle;
//		Platform->fdb.left_pitch_angle  = -63.0f + Motor_Pitch_Left_Motor_A1.encoder.angle;
//  	Platform->fdb.right_pitch_angle = -69.0f + Motor_Pitch_Right_Motor_A1.encoder.angle;
	// A1 电机看门狗递增
	Motor_Pitch_Front_Motor_A1.watchdog += 1;
    Motor_Pitch_Left_Motor_A1.watchdog += 1;
	Motor_Pitch_Right_Motor_A1.watchdog += 1; 
	
	// 计算自上次调用以来经过的精确时间，为PID的微分项（D）提供准确的时间基准
  	Platform->update_dt = DWT_GetDeltaT(&Platform->last_update_tick);

}


// 安全监控函数，用于检测系统运行中的异常情况
// 在其他地方（通常是CAN接收中断中），当收到对应电机的数据时，这个计数器会被清零。
// 如果长时间未收到数据，计数值会持续累加，最终在 Platform_Check 中触发离线错误。
void Platform_Check() {
	
	Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
	//if(Motor_Pitch_Front_Motor_A1.watchdog>20)
	//	{Platform->error_code = 1;}
		if(Motor_Pitch_Left_Motor_A1.watchdog>20)
			{Platform->error_code = 2;}	
	//	if(Motor_Pitch_Right_Motor_A1.watchdog>20)
	//		{Platform->error_code = 3;}		
		if(fabsf(Motor_Pitch_Left_Motor_A1.encoder.torque + Motor_Pitch_Right_Motor_A1.encoder.torque)>20.0f)
		{watchdog2++;}
		else{watchdog2=0;}
	if(watchdog2>20){
			Platform->error_code = 4;
		}
}

/**
 * @brief      开环加速模式：直接输出最大扭矩（绕过PID）
 * @param      direction  加速方向（1=向上推，-1=向下）
 * @retval     None
 * @note       用于初始阶段快速加速，充分利用电机峰值扭矩
 */
void Platform_OpenLoop_Boost_Output(int8_t direction) {
		Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
	
    float base_torque = boost_cfg.max_torque * direction;
	
		// 2. 获取当前三个电机的实时角度反馈
		float ang_f = Platform->fdb.front_pitch_angle;
    float ang_l = Platform->fdb.left_pitch_angle;
    float ang_r = Platform->fdb.right_pitch_angle;
	  float avg_ang = (ang_f + ang_l + ang_r) / 3.0f;//计算平均位置
	
    // 4. 定义死区（单位：度）
    // 排球冲击大，不建议死区设置过大。建议 0.5f - 1.0f 左右
    float deadzone = 4.0f; 
    
    // 5. 计算偏差并应用死区逻辑
    float diff_f = avg_ang - ang_f;
    float diff_l = avg_ang - ang_l;
    float diff_r = avg_ang - ang_r;
    
    // 纠偏分量计算函数（局部逻辑：超过死区的部分才计算力矩）
    // 这样可以保证力矩从 0 开始平滑增加，而不是瞬间跳变
    if (fabs(diff_f) < deadzone) diff_f = 0.0f;
    else diff_f = (diff_f > 0) ? (diff_f - deadzone) : (diff_f + deadzone);
    
    if (fabs(diff_l) < deadzone) diff_l = 0.0f;
    else diff_l = (diff_l > 0) ? (diff_l - deadzone) : (diff_l + deadzone);
    
    if (fabs(diff_r) < deadzone) diff_r = 0.0f;
    else diff_r = (diff_r > 0) ? (diff_r - deadzone) : (diff_r + deadzone);

    // 6. 计算最终力矩
    // K_TWIST_COMP 对排球建议在 8.0 - 15.0 左右
    float t_f = base_torque + diff_f * K_TWIST_COMP;
    float t_l = (base_torque + 0.5f) + diff_l * K_TWIST_COMP;
    float t_r = (base_torque + 0.5f) + diff_r * K_TWIST_COMP;
    
    // 7. 发送输出
    Motor_SetMotorOutput(&Motor_Pitch_Front_Motor_A1, t_f);
    Motor_SetMotorOutput(&Motor_Pitch_Left_Motor_A1,  t_l); // 修正：原代码重复加了0.5f，这里统一在计算时处理
    Motor_SetMotorOutput(&Motor_Pitch_Right_Motor_A1, t_r);
}

float dist_to_top;
/**
 * @brief      检查是否应使用开环加速（距顶点较远时开环，接近时切PID）
 * @param      current_z  当前Z高度
 * @param      target_z   目标Z高度（顶点）
 * @retval     1=开环加速中，0=使用PID控制
 */
uint8_t Platform_Check_OpenLoop_Boost(float current_z, float target_z) {
    if (!boost_cfg.enabled) return 0;
    
    dist_to_top = target_z - current_z;
    
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
	// ======== A1 电机版本 ========
  	Motor_SetMotorOutput(&Motor_Pitch_Front_Motor_A1, torque1);
	Motor_SetMotorOutput(&Motor_Pitch_Left_Motor_A1, torque2);
	Motor_SetMotorOutput(&Motor_Pitch_Right_Motor_A1, torque3);
}

static float spd_filtered[3] = {0.0f};  // 三个电机的滤波速度
#define SPEED_FILTER_ALPHA 0.5f  // 滤波强度
static float right_left_gain = 1.2;

// 实现位置-速度串级双环 PID 控制器，是整个闭环控制的核心算法
// 内部消除重复：使用宏封装串级计算，保持接口与行为不变
/*
- ANG_PID：角度环 PID 控制器对象
- SPD_PID：速度环 PID 控制器对象
- ANG_PARAM：角度环 PID 参数对象
- SPD_PARAM：速度环 PID 参数对象
- SETPOINT：目标角度（位置设定值）
- FDB_ANG：实际角度反馈值
- ENC_SPD：实际速度反馈值
- SIGN：速度反馈符号（根据电机安装方向调整）
- BIAS：输出力矩偏置（用于克服重力等静态负载）
- OUT_TORQUE：输出力矩变量（结果存储位置）
*/
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

float torque1,torque2,torque3,torque4,torque5,torque6;
void Platform_Set_Angle_Output(float ang1,float ang2 ,float ang3,float ang4,float ang5,float ang6) {
	
  	Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
	
	LimitMaxMin(ang1,50.0f,-30.0f);
	LimitMaxMin(ang2,30.0f,-30.0f);
	LimitMaxMin(ang3,30.0f,-30.0f);
	LimitMaxMin(ang4,30.0f,-30.0f);
	LimitMaxMin(ang5,150.0f,90.0f);
	LimitMaxMin(ang6,270.0f,210.0f);

  // ======== A1 电机：使用原始 CASCADE_PITCH 宏结构 ========
  // 直接用 A1 电机替代 Go1 电机，保持完全相同的控制逻辑
  switch (Platform->output_state) {
    case Platform_slow:
      // Go1原版：CASCADE_PITCH(..., Motor_Pitch_Front_Motor.encoder.speed, -1.0f, -1.5f, torque1);
      CASCADE_PITCH(Platform->pid.Pitch1_Ang_PID, Platform->pid.Pitch1_Spd_PID, Platform->pid.Pitch_Ang_Slow_PIDParam, Platform->pid.Pitch_Spd_Slow_PIDParam,
                    ang1, Platform->fdb.front_pitch_angle, Motor_Pitch_Front_Motor_A1.encoder.speed, 1.0f, 1.5f, torque1);
      CASCADE_PITCH(Platform->pid.Pitch2_Ang_PID, Platform->pid.Pitch2_Spd_PID, Platform->pid.Pitch_Ang_Slow_PIDParam, Platform->pid.Pitch_Spd_Slow_PIDParam,
                    ang2, Platform->fdb.left_pitch_angle, Motor_Pitch_Left_Motor_A1.encoder.speed, 1.0f, 1.5f, torque2);
      // Right 电机测试：速度取反
      CASCADE_PITCH(Platform->pid.Pitch3_Ang_PID, Platform->pid.Pitch3_Spd_PID, Platform->pid.Pitch_Ang_Slow_PIDParam, Platform->pid.Pitch_Spd_Slow_PIDParam,
                    ang3, Platform->fdb.right_pitch_angle, Motor_Pitch_Right_Motor_A1.encoder.speed, 1.0f, 1.5f, torque3);
      break;

    case Platform_middle:
      CASCADE_PITCH(Platform->pid.Pitch1_Ang_PID, Platform->pid.Pitch1_Spd_PID, Platform->pid.Pitch_Ang_Middle_PIDParam, Platform->pid.Pitch_Spd_Middle_PIDParam,
                    ang1, Platform->fdb.front_pitch_angle, Motor_Pitch_Front_Motor_A1.encoder.speed, 1.0f, 1.5f, torque1);
      CASCADE_PITCH(Platform->pid.Pitch2_Ang_PID, Platform->pid.Pitch2_Spd_PID, Platform->pid.Pitch_Ang_Middle_PIDParam, Platform->pid.Pitch_Spd_Middle_PIDParam,
                    ang2, Platform->fdb.left_pitch_angle, Motor_Pitch_Left_Motor_A1.encoder.speed, 1.0f, 1.5f, torque2);
      // Right 电机测试：速度取反
      CASCADE_PITCH(Platform->pid.Pitch3_Ang_PID, Platform->pid.Pitch3_Spd_PID, Platform->pid.Pitch_Ang_Middle_PIDParam, Platform->pid.Pitch_Spd_Middle_PIDParam,
                    ang3, Platform->fdb.right_pitch_angle, Motor_Pitch_Right_Motor_A1.encoder.speed, 1.0f, 1.5f, torque3);
      break;

    case Platform_fast:
      CASCADE_PITCH(Platform->pid.Pitch1_Ang_PID, Platform->pid.Pitch1_Spd_PID, Platform->pid.Pitch_Ang_Fast_PIDParam, Platform->pid.Pitch_Spd_Fast_PIDParam,
                    ang1, Platform->fdb.front_pitch_angle, Motor_Pitch_Front_Motor_A1.encoder.speed, 1.0f, 1.5f, torque1);
      CASCADE_PITCH(Platform->pid.Pitch2_Ang_PID, Platform->pid.Pitch2_Spd_PID, Platform->pid.Pitch_Ang_Fast_PIDParam, Platform->pid.Pitch_Spd_Fast_PIDParam,
                    ang2, Platform->fdb.left_pitch_angle, Motor_Pitch_Left_Motor_A1.encoder.speed, 1.0f, 1.5f, torque2);
      // Right 电机测试：速度取反
      CASCADE_PITCH(Platform->pid.Pitch3_Ang_PID, Platform->pid.Pitch3_Spd_PID, Platform->pid.Pitch_Ang_Fast_PIDParam, Platform->pid.Pitch_Spd_Fast_PIDParam,
                    ang3, Platform->fdb.right_pitch_angle, Motor_Pitch_Right_Motor_A1.encoder.speed, 1.0f, 1.5f, torque3);
      break;

    default:
      break;
  }

	
  // ------------------------------------------------------------------------------------------------
  // [Algorithm Added] 击球阻力补偿算法 (仅在上升阶段 STATE_RISING 生效)
  // ------------------------------------------------------------------------------------------------
  // 静态变量保存上一帧速度，用于计算加速度
  static float last_spd_A1[3] = {0.0f, 0.0f, 0.0f};

  // 仅在上升状态下检测阻力 (Returning 和 Cooldown 不处理)
  if (hit_state == STATE_RISING) {
      float dt = Platform->update_dt;
      // 防止 dt 异常或过小导致微分噪声
      if (dt > 0.0001f && dt < 0.1f) {
          // === 参数配置区 (建议根据实测波形调整) ===
          const float ACC_TRIGGER_THRESHOLD = -1000.0f; // 触发阈值: 减速度超过此值 (rad/s^2)
          const float IMPACT_GAIN = 0.02f;             // 增益: 每 1 unit 减速度补偿的力矩 (Nm)
          const float MAX_EXTRA_TORQUE = 5.0f;        // 补偿限幅: 最大允许额外增加的力矩 (Nm)
          // ======================================

          float current_spd[3];
          float *torques[3] = {&torque1, &torque2, &torque3}; // 指向待输出力矩的指针
          
          // 获取当前电机速度
          current_spd[0] = Motor_Pitch_Front_Motor_A1.encoder.speed;
          current_spd[1] = Motor_Pitch_Left_Motor_A1.encoder.speed;
          current_spd[2] = Motor_Pitch_Right_Motor_A1.encoder.speed;

          for(int i=0; i<3; i++) {
              // 1. 计算实时加速度 (rad/s^2)
              float acc = (current_spd[i] - last_spd_A1[i]) / dt;

              // 2. 触发条件判断
              // 条件A: 电机正在正向运动 (假设上升对应速度 > 0)。如果您的电机上升是反转，请改为 current_spd[i] < -1.0f
              // 条件B: 加速度为较大的负值 (说明受阻剧烈减速)
              if (current_spd[i] > 8.0f && acc < ACC_TRIGGER_THRESHOLD) {
                   // 3. 线性计算补偿值: (ACC - 阈值) 是负数，取反后乘以增益
                   float boost = -(acc - ACC_TRIGGER_THRESHOLD) * IMPACT_GAIN;
                   
                   // 4. 安全限幅
                   if (boost > MAX_EXTRA_TORQUE) boost = MAX_EXTRA_TORQUE;
                   if (boost < 0.0f) boost = 0.0f; // 不反向补偿

                   // 5. 叠加到最终输出 (torque1/2/3)
                   *(torques[i]) += boost;
              }
              // 更新历史值
              last_spd_A1[i] = current_spd[i];
          }
      }
  } else {
      // 非上升阶段：持续更新历史速度，保证状态切换瞬间加速度计算正确
      last_spd_A1[0] = Motor_Pitch_Front_Motor_A1.encoder.speed;
      last_spd_A1[1] = Motor_Pitch_Left_Motor_A1.encoder.speed;
      last_spd_A1[2] = Motor_Pitch_Right_Motor_A1.encoder.speed;
  }

    // ================================================================================================
    // [核心新增] 底层软件缓冲 (Soft Buffer)：防止超调撞击限位与起车
    // ================================================================================================
    float current_z = Platform->fdb.plat_z;
	  // 使用三个电机的平均速度来判定上升趋势
    float avg_v = (Motor_Pitch_Front_Motor_A1.encoder.speed + 
                   Motor_Pitch_Left_Motor_A1.encoder.speed + 
                   Motor_Pitch_Right_Motor_A1.encoder.speed) / 3.0f;

    // 触发点设在 0.302f（目标 0.30m 上方 2mm），保护力臂限位
    const float SOFT_STOP_START_Z = 0.232f; 
    const float SOFT_STOP_K       = 1200.0f; // 虚拟弹簧系数
    const float SOFT_STOP_D       = 20.0f;   // 虚拟阻尼，吸收撞击动能
    
    // 只有当位置超过阈值且仍在向上冲时介入
    if (current_z > SOFT_STOP_START_Z && avg_v > 5.0f) {
        float delta_z = current_z - SOFT_STOP_START_Z;
        // 计算反向压制力矩
				float brake_force = (delta_z * SOFT_STOP_K) + (avg_v * SOFT_STOP_D);
        // 即使在高速下，也设置一个天花板，防止瞬间反拉力矩过猛
				if (brake_force > 20.0f) brake_force = 20.0f;
					torque1 -= brake_force;
					torque2 -= brake_force;
					torque3 -= brake_force;
    }
  // ------------------------------------------------------------------------------------------------

  Platform_Set_Torque_Output(torque1, torque2, torque3, 0.0f, 0.0f, 0.0f);

  // ======== Go1 原版代码（已注释）========
//   switch (Platform->output_state) {
//     case Platform_slow:
//       CASCADE_PITCH(Platform->pid.Pitch1_Ang_PID, Platform->pid.Pitch1_Spd_PID, Platform->pid.Pitch_Ang_Slow_PIDParam, Platform->pid.Pitch_Spd_Slow_PIDParam,
//                     ang1, Platform->fdb.front_pitch_angle, Motor_Pitch_Front_Motor.encoder.speed, -1.0f, -1.5f, torque1);
//       CASCADE_PITCH(Platform->pid.Pitch2_Ang_PID, Platform->pid.Pitch2_Spd_PID, Platform->pid.Pitch_Ang_Slow_PIDParam, Platform->pid.Pitch_Spd_Slow_PIDParam,
//                     ang2, Platform->fdb.left_pitch_angle, Motor_Pitch_Left_Motor.encoder.speed, -1.0f, -1.5f, torque2);
//       CASCADE_PITCH(Platform->pid.Pitch3_Ang_PID, Platform->pid.Pitch3_Spd_PID, Platform->pid.Pitch_Ang_Slow_PIDParam, Platform->pid.Pitch_Spd_Slow_PIDParam,
//                     ang3, Platform->fdb.right_pitch_angle, Motor_Pitch_Right_Motor.encoder.speed, 1.0f, 1.5f, torque3);

//       CASCADE_YAW(Platform->pid.Yaw1_Ang_PID, Platform->pid.Yaw1_Spd_PID, Platform->pid.Yaw_Ang_Slow_PIDParam, Platform->pid.Yaw_Spd_Slow_PIDParam,
//                   ang4, Platform->fdb.front_yaw_angle, Motor_Yaw_Front_Motor.encoder.speed, 0.0f, torque4);
//       CASCADE_YAW(Platform->pid.Yaw2_Ang_PID, Platform->pid.Yaw2_Spd_PID, Platform->pid.Yaw_Ang_Slow_PIDParam, Platform->pid.Yaw_Spd_Slow_PIDParam,
//                   ang5, Platform->fdb.left_yaw_angle, Motor_Yaw_Left_Motor.encoder.speed, 0.5f, torque5);
//       CASCADE_YAW(Platform->pid.Yaw3_Ang_PID, Platform->pid.Yaw3_Spd_PID, Platform->pid.Yaw_Ang_Slow_PIDParam, Platform->pid.Yaw_Spd_Slow_PIDParam,
//                   ang6, Platform->fdb.right_yaw_angle, Motor_Yaw_Right_Motor.encoder.speed, -0.5f, torque6);
//       break;

//     case Platform_middle:
//       CASCADE_PITCH(Platform->pid.Pitch1_Ang_PID, Platform->pid.Pitch1_Spd_PID, Platform->pid.Pitch_Ang_Middle_PIDParam, Platform->pid.Pitch_Spd_Middle_PIDParam,
//                     ang1, Platform->fdb.front_pitch_angle, Motor_Pitch_Front_Motor.encoder.speed, -1.0f, -1.5f, torque1);
//       CASCADE_PITCH(Platform->pid.Pitch2_Ang_PID, Platform->pid.Pitch2_Spd_PID, Platform->pid.Pitch_Ang_Middle_PIDParam, Platform->pid.Pitch_Spd_Middle_PIDParam,
//                     ang2, Platform->fdb.left_pitch_angle, Motor_Pitch_Left_Motor.encoder.speed, -1.0f, -1.5f, torque2);
//       CASCADE_PITCH(Platform->pid.Pitch3_Ang_PID, Platform->pid.Pitch3_Spd_PID, Platform->pid.Pitch_Ang_Middle_PIDParam, Platform->pid.Pitch_Spd_Middle_PIDParam,
//                     ang3, Platform->fdb.right_pitch_angle, Motor_Pitch_Right_Motor.encoder.speed, 1.0f, 1.5f, torque3);

//       CASCADE_YAW(Platform->pid.Yaw1_Ang_PID, Platform->pid.Yaw1_Spd_PID, Platform->pid.Yaw_Ang_Middle_PIDParam, Platform->pid.Yaw_Spd_Middle_PIDParam,
//                   ang4, Platform->fdb.front_yaw_angle, Motor_Yaw_Front_Motor.encoder.speed, 0.0f, torque4);
//       CASCADE_YAW(Platform->pid.Yaw2_Ang_PID, Platform->pid.Yaw2_Spd_PID, Platform->pid.Yaw_Ang_Middle_PIDParam, Platform->pid.Yaw_Spd_Middle_PIDParam,
//                   ang5, Platform->fdb.left_yaw_angle, Motor_Yaw_Left_Motor.encoder.speed, 0.5f, torque5);
//       CASCADE_YAW(Platform->pid.Yaw3_Ang_PID, Platform->pid.Yaw3_Spd_PID, Platform->pid.Yaw_Ang_Middle_PIDParam, Platform->pid.Yaw_Spd_Middle_PIDParam,
//                   ang6, Platform->fdb.right_yaw_angle, Motor_Yaw_Right_Motor.encoder.speed, -0.5f, torque6);
//       break;

//     case Platform_fast:
//       CASCADE_PITCH(Platform->pid.Pitch1_Ang_PID, Platform->pid.Pitch1_Spd_PID, Platform->pid.Pitch_Ang_Fast_PIDParam, Platform->pid.Pitch_Spd_Fast_PIDParam,
//                     ang1, Platform->fdb.front_pitch_angle, Motor_Pitch_Front_Motor.encoder.speed, -1.0f, -1.5f, torque1);
//       CASCADE_PITCH(Platform->pid.Pitch2_Ang_PID, Platform->pid.Pitch2_Spd_PID, Platform->pid.Pitch_Ang_Fast_PIDParam, Platform->pid.Pitch_Spd_Fast_PIDParam,
//                     ang2, Platform->fdb.left_pitch_angle, Motor_Pitch_Left_Motor.encoder.speed, -1.0f, -1.5f, torque2);
//       CASCADE_PITCH(Platform->pid.Pitch3_Ang_PID, Platform->pid.Pitch3_Spd_PID, Platform->pid.Pitch_Ang_Fast_PIDParam, Platform->pid.Pitch_Spd_Fast_PIDParam,
//                     ang3, Platform->fdb.right_pitch_angle, Motor_Pitch_Right_Motor.encoder.speed, 1.0f, 1.5f, torque3);

//       CASCADE_YAW(Platform->pid.Yaw1_Ang_PID, Platform->pid.Yaw1_Spd_PID, Platform->pid.Yaw_Ang_Fast_PIDParam, Platform->pid.Yaw_Spd_Fast_PIDParam,
//                   ang4, Platform->fdb.front_yaw_angle, Motor_Yaw_Front_Motor.encoder.speed, 0.0f, torque4);
//       CASCADE_YAW(Platform->pid.Yaw2_Ang_PID, Platform->pid.Yaw2_Spd_PID, Platform->pid.Yaw_Ang_Fast_PIDParam, Platform->pid.Yaw_Spd_Fast_PIDParam,
//                   ang5, Platform->fdb.left_yaw_angle, Motor_Yaw_Left_Motor.encoder.speed, 0.5f, torque5);
//       CASCADE_YAW(Platform->pid.Yaw3_Ang_PID, Platform->pid.Yaw3_Spd_PID, Platform->pid.Yaw_Ang_Fast_PIDParam, Platform->pid.Yaw_Spd_Fast_PIDParam,
//                   ang6, Platform->fdb.right_yaw_angle, Motor_Yaw_Right_Motor.encoder.speed, -0.5f, torque6);
//       break;

//     default:
//       break;
//   }
//   // 将力矩值发送给 Go1 电机
//   Motor_SetMotorOutput(&Motor_Pitch_Front_Motor, torque1);
//   Motor_SetMotorOutput(&Motor_Pitch_Left_Motor, torque2);
//   Motor_SetMotorOutput(&Motor_Pitch_Right_Motor, torque3);

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
	LimitMaxMin(Platform->tar.plat_pitch,10,-40);
	LimitMaxMin(Platform->tar.plat_yaw,30,-30);
	LimitMaxMin(Platform->tar.plat_roll,20,-20);

}

// 逆运动学解算【数学核心】
void Platform_Cal_3Degree_IK_Output() {
	
    if (hit_state == STATE_BRAKING) {
        return;  // 跳过逆运动学计算，保持锁存角度
    }
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
  	Platform_Set_Angle_Output(0.5*rad2deg(theta1),right_left_gain*rad2deg(theta2),right_left_gain*rad2deg(theta3),0,120,240);

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





// =================================================================
// -------------------------- 运动参数赋值 --------------------------
// =================================================================
ZTrajProfile z_profile = {
    .v_up_max      = 0.0f,   // 上升最大速度 (m/s)
    .a_up_max      = 0.0f,   // 上升最大加速度 (m/s^2)
    .ret_ratio     = 0.0f,   // 下降速度/加速度比率
    .brake_gain    = 0.0f,   // 下降刹车距离增益
    .boost_err     = 0.0f,   // 开环加速误差阈值
    .boost_gain    = 0.0f    // 开环加速增益
};

static inline void Z_ApplyProfileSafe(void) {
    z_profile.v_up_max      = 15.0f;
    z_profile.a_up_max      = 30.0f;
    z_profile.ret_ratio     = 0.05f;
    z_profile.brake_gain    = 0.10f;
    z_profile.boost_err     = 0.05f;
    z_profile.boost_gain    = 1.00f;
}

// 保留速度状态和目标
static float l_traj_v              = 0.0f;        // 高度 z 方向的速度状态
static float l_traj_z              = 0.0f;       // 高度 z 方向的位置状态
static float hit_pitch_goal      = 15.0f;        // 击球时的目标 pitch 角度
static float hit_roll_goal       = 0.0f;        // 击球时的目标 roll 角度
static const float z_near_zero   = 0.1f;      // 认为高度 z 值收敛的阈值
static const float v_near_zero   = 0.01f;       // 认为速度 v 收敛的阈值（m/s）
static const float ang_near_zero = 3.0f;        // 认为角度 ang 收敛的阈值（度）
static const float w_near_zero   = 3.0f;        // 认为角速度 w 收敛的阈值（度/s）

float start_hit_time = 0.0f;                    //击球动作开始（工具）
float start_back_time = 0.0f;                   //击球动作回拉开始（工具）
uint8_t finishhit = 1;                          //击球动作的状态标志，0为击球中，1为击打已完成
static uint8_t  dianqiu_mid_active = 0;          // MIDDLE 模式直接角度控制标志
static float    dianqiu_mid_target[3] = {0};     // MIDDLE 模式三电机目标角度
static uint8_t  dianqiu_down_active = 0;         // DOWN 模式比例同步标志
static float    dianqiu_down_start[3] = {0};     // DOWN 模式三电机起始角度
static float    dianqiu_down_target[3] = {0};    // DOWN 模式三电机目标角度



// =================================================================
// ------------------------- 通用轨迹更新器 -------------------------
// =================================================================
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
static inline int Traj_Update(float goal, float dt, float v_limit, float a_limit,
                              float *state, float *current_v,
                              float min, float max, float tol, int is_z)
{
    // 输入参数检查
    if (dt <= 0.0f || state == NULL || current_v == NULL) return 0;
    // 期望最大速度限幅，确保非零
    float v_cap = fabsf(v_limit);
    if (v_cap < 1e-6f) v_cap = 1e-3f;
    // 期望最大加速度限幅，确保非零
    float a_cap = (a_limit > 1e-6f) ? a_limit : 1e-3f;
    // 计算当前误差（目标 - 当前状态）
    float error = goal - (*state);
    float abs_error = fabsf(error);
    
    if (abs_error > tol) {

        // step 1 根据当前速度计算制动距离
        float stop_dist = ((*current_v) * (*current_v)) / (2.0f * a_cap);

        // step 2 计算刹车距离增益（Z 轴下降时加倍）
        float brake_k = (is_z ? z_profile.brake_gain : 1.0f);
        // if (is_z && error < 0.0f) brake_k *= 2.0f;(已整合到 z_profile.brake_gain 中)

        // step 3 期望速度计算：远离目标时为最大速度，接近目标时为刹车速度
        float v_cmd = (error > 0.0f) ? v_cap : -0.5f * v_cap;
        // 当偏差小于刹车距离时，以最大刹车加速度 dt 步长停止
        if (abs_error <= stop_dist * brake_k) v_cmd = 0.0f;

        // step 4 以最大加速度 dt 步长逼近期望速度
        float dv = v_cmd - (*current_v);
        float dv_max = a_cap * dt;
        if (dv >  dv_max) dv =  dv_max;
        if (dv < -dv_max) dv = -dv_max;

        // step 5 速度与状态更新 + 限幅
        *current_v += dv; // 速度以最大加速度逐步逼近期望速度
        if (*current_v >  v_cap) *current_v =  v_cap;
        if (*current_v < -v_cap) *current_v = -v_cap;
        *state += (*current_v) * dt;
        if (*state < min) *state = min;
        if (*state > max) *state = max;
        return 0;

    } else {
        // 进入到达阶段：锁定目标并清零轨迹速度
        *state  = goal;
        *current_v = 0.0f;
        return 1;
    }
}

// =================================================================
// -------------------------- 角度平滑接口 --------------------------
// =================================================================
// 遥控角度轨迹限幅参数
#define REMOTE_PITCH_V_MAX  360.0f  // deg/s (原 120.0)
#define REMOTE_PITCH_A_MAX  900.0f  // deg/s^2 (原 600.0)
#define REMOTE_ROLL_V_MAX   240.0f  // deg/s (原 180.0)
#define REMOTE_ROLL_A_MAX   1100.0f // deg/s^2 (原 800.0)

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
    Platform_Set_Target_Pos(0,0,l_traj_z, Platform->tar.plat_pitch, Platform->tar.plat_yaw, Platform->tar.plat_roll);
}

static inline int AnglesReady(float pitch_goal, float roll_goal)
{
    Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
    int pitch_ok = (fabsf(pitch_goal - Platform->tar.plat_pitch) <= ang_near_zero) || (fabsf(pitch_traj_v) <= w_near_zero);
    int roll_ok  = (fabsf(roll_goal  - Platform->tar.plat_roll)  <= ang_near_zero) || (fabsf(roll_traj_v)  <= w_near_zero);
    return pitch_ok && roll_ok;
}

float cooldown_elapsed;

// =================================================================
// ------------------------- 平台击打控制函数 -------------------------
// =================================================================

// Jiefa 击球状态机参数
#define JIEFA_MAX_TORQUE      33.0f   // 加速阶段最大力矩 (Nm)
#define JIEFA_TORQUE      20.0     // 期望的初始力矩
#define JIEFA_ALPHA_TARGET    40.0f   // 加速旋转角度阈值 (度, 相对起始位置的增量)
#define JIEFA_BRAKE_ANG_TOL   2.0f    // 刹车角度容差 (度)
#define JIEFA_BRAKE_SPD_TOL   2.0f    // 刹车速度容差 (rad/s)
#define JIEFA_RETURN_SPD_TOL  2.0f    // 返回阶段速度容差 (rad/s)
#define JIEFA_COOLDOWN_S      2.0f    // 两次击球最小间隔 (秒)
#define JIEFA_FRONT_OFFSET    0.0f    // 前侧电机补偿角

//#define JIEFA_TORQUE      18.0     // 期望的初始力矩
//#define JIEFA_ALPHA_TARGET    35.0f   // 加速旋转角度阈值 (度, 相对起始位置的增量)

// Dianqiu DOWN 模式：三电机比例同步参数
#define DIANQIU_DOWN_FRONT_DEG  80.0f  // Front 电机目标旋转角度 (°)
#define DIANQIU_DOWN_LEFT_DEG   35.0f  // Left  电机目标旋转角度 (°)
#define DIANQIU_DOWN_RIGHT_DEG  35.0f  // Right 电机目标旋转角度 (°)

static float jiefa_start_angle[3] = {0};  // 击球起始角度（绝对）
static float jiefa_alpha[3] = {0};        // 刹车锁定角度（绝对）
static float jiefa_last_trigger = -10.0f; // 上次触发时刻
static float jiefa_power = 1.0f;           // 击球力度因子

void Platform_Jiefa_Cal(float hit) {

    Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
		if(hit > 1.0f){
			hit = 1.0f;
		}

    // 修复1：不再盲目 return。
    // 如果已经在打击动作中（非 IDLE），即使 hit 不是 1.0f 也必须继续执行状态机直到动作闭环完成
    if (hit_state == STATE_IDLE && hit <= 0.0f) {
        return;
    }

    float now = DWT_GetTimeline_s();

    // ===== STATE_IDLE: 等待触发（含 2s 冷却）=====
    if (hit_state == STATE_IDLE) {
        Platform->flag = 0.0f; // 收到指令立马消费，阻断连发
        jiefa_power = hit;      // 锁定本次击球力度

        if (now - jiefa_last_trigger < JIEFA_COOLDOWN_S) return;
        jiefa_last_trigger = now;

        static uint8_t is_start_angle_recorded = 0;

        // 记录击球起始的绝对角度
        if (is_start_angle_recorded == 0) {
            jiefa_start_angle[0] = Platform->fdb.front_pitch_angle;
            jiefa_start_angle[1] = Platform->fdb.left_pitch_angle;
            jiefa_start_angle[2] = Platform->fdb.right_pitch_angle;
            is_start_angle_recorded = 1;
        }

        // // 判断：只有当三个起步角度的绝对值都小于阈值（例如安全范围5.0度）时，才允许进入击打状态
        // if (fabsf(jiefa_start_angle[0]) < 5.0f && 
        //     fabsf(jiefa_start_angle[1]) < 5.0f && 
        //     fabsf(jiefa_start_angle[2]) < 5.0f) {
            
        //     hit_state = STATE_RISING;
        //     Platform_Set_OutputState(Platform_fast);
        // }
        hit_state = STATE_RISING;
        Platform_Set_OutputState(Platform_fast);
    }

    // ===== STATE_RISING: 最大力矩加速直到旋转增量达到 alpha =====
    if (hit_state == STATE_RISING) {
        float ang_f = Platform->fdb.front_pitch_angle-JIEFA_FRONT_OFFSET;
        float ang_l = Platform->fdb.left_pitch_angle;
        float ang_r = Platform->fdb.right_pitch_angle;

        // 用三电机平均旋转增量（相对起始位置）判断是否达到阈值
        float delta_f = ang_f - jiefa_start_angle[0];
        float delta_l = ang_l - jiefa_start_angle[1];
        float delta_r = ang_r - jiefa_start_angle[2];
        float avg_delta = (delta_f + delta_l + delta_r) / 3.0f;

        if (avg_delta >= JIEFA_ALPHA_TARGET) {
            // 锁定当前绝对角度作为刹车目标
            jiefa_alpha[0] = ang_f-5.0f;
            jiefa_alpha[1] = ang_l-5.0f;
            jiefa_alpha[2] = ang_r-5.0f;
            PID_ClearPID(&Platform->pid.Pitch1_Ang_PID);
            PID_ClearPID(&Platform->pid.Pitch1_Spd_PID);
            PID_ClearPID(&Platform->pid.Pitch2_Ang_PID);
            PID_ClearPID(&Platform->pid.Pitch2_Spd_PID);
            PID_ClearPID(&Platform->pid.Pitch3_Ang_PID);
            PID_ClearPID(&Platform->pid.Pitch3_Spd_PID);
            hit_state = STATE_BRAKING;
        } else {
            // 三电机平衡补偿后直接输出最大力矩
            float deadzone = 3.0f; // 误差死区，避免过度补偿导致震荡
            float comp_gain = 6.0f; // 补偿增益，调整补偿力度（过大可能震荡，过小可能不够）
            float ang_avg = (ang_f + ang_l + ang_r) / 3.0f;
            float diff_f = ang_avg - ang_f;
            float diff_l = ang_avg - ang_l;
            float diff_r = ang_avg - ang_r;

            if (fabsf(diff_f) < deadzone) diff_f = 0;
            else diff_f = (diff_f > 0) ? (diff_f - deadzone) : (diff_f + deadzone);
            if (fabsf(diff_l) < deadzone) diff_l = 0;
            else diff_l = (diff_l > 0) ? (diff_l - deadzone) : (diff_l + deadzone);
            if (fabsf(diff_r) < deadzone) diff_r = 0;
            else diff_r = (diff_r > 0) ? (diff_r - deadzone) : (diff_r + deadzone);

            float t_f = (JIEFA_TORQUE + diff_f * comp_gain) * jiefa_power;
            float t_l = (JIEFA_TORQUE + diff_l * comp_gain) * jiefa_power;
            float t_r = (JIEFA_TORQUE + diff_r * comp_gain) * jiefa_power;

            LimitMaxMin(t_f, JIEFA_MAX_TORQUE, -JIEFA_MAX_TORQUE);
            LimitMaxMin(t_l, JIEFA_MAX_TORQUE, -JIEFA_MAX_TORQUE);
            LimitMaxMin(t_r, JIEFA_MAX_TORQUE, -JIEFA_MAX_TORQUE);

            Motor_SetMotorOutput(&Motor_Pitch_Front_Motor_A1, t_f);
            Motor_SetMotorOutput(&Motor_Pitch_Left_Motor_A1,  t_l);
            Motor_SetMotorOutput(&Motor_Pitch_Right_Motor_A1, t_r);
        }
    }

    // ===== STATE_BRAKING: PID fast 刹车到锁定角度（稳定后保持 1s 再返回）=====
    if (hit_state == STATE_BRAKING) {
        Platform_Set_OutputState(Platform_fast);
        float ang_err[3], spd[3];

        ang_err[0] = fabsf(Platform->fdb.front_pitch_angle - jiefa_alpha[0]);
        ang_err[1] = fabsf(Platform->fdb.left_pitch_angle  - jiefa_alpha[1]);
        ang_err[2] = fabsf(Platform->fdb.right_pitch_angle - jiefa_alpha[2]);
        spd[0] = fabsf(Motor_Pitch_Front_Motor_A1.encoder.speed);
        spd[1] = fabsf(Motor_Pitch_Left_Motor_A1.encoder.speed);
        spd[2] = fabsf(Motor_Pitch_Right_Motor_A1.encoder.speed);

        uint8_t converged = (ang_err[0] < JIEFA_BRAKE_ANG_TOL && ang_err[1] < JIEFA_BRAKE_ANG_TOL && ang_err[2] < JIEFA_BRAKE_ANG_TOL &&
                             spd[0] < JIEFA_BRAKE_SPD_TOL && spd[1] < JIEFA_BRAKE_SPD_TOL && spd[2] < JIEFA_BRAKE_SPD_TOL);

        static float jiefa_stable_time = 0;
        if (!converged) {
            jiefa_stable_time = 0;
						Platform_Set_Angle_Output(jiefa_alpha[0], jiefa_alpha[1], jiefa_alpha[2], 0, 120, 240);

        } else {
            if (jiefa_stable_time == 0) jiefa_stable_time = now;
            if (now - jiefa_stable_time >= 1.0f && keep_state_flag == 0.0f) {
                jiefa_stable_time = 0;
                Platform_Set_Target_Pos(0, 0, Initial_Z_Position, 0, 0, 0);
                PID_ClearPID(&Platform->pid.Pitch1_Ang_PID);
                PID_ClearPID(&Platform->pid.Pitch1_Spd_PID);
                PID_ClearPID(&Platform->pid.Pitch2_Ang_PID);
                PID_ClearPID(&Platform->pid.Pitch2_Spd_PID);
                PID_ClearPID(&Platform->pid.Pitch3_Ang_PID);
                PID_ClearPID(&Platform->pid.Pitch3_Spd_PID);
                hit_state = STATE_RETURNING;

            } else {
                Platform_Set_Angle_Output(jiefa_alpha[0], jiefa_alpha[1], jiefa_alpha[2], 0, 120, 240);
						}
        }
    }

    // ===== STATE_RETURNING: PID slow 返回 Initpose 位置 =====
    if (hit_state == STATE_RETURNING) {
        Platform_Set_OutputState(Platform_slow);
        float spd[3];
        spd[0] = fabsf(Motor_Pitch_Front_Motor_A1.encoder.speed);
        spd[1] = fabsf(Motor_Pitch_Left_Motor_A1.encoder.speed);
        spd[2] = fabsf(Motor_Pitch_Right_Motor_A1.encoder.speed);

        if (spd[0] < JIEFA_RETURN_SPD_TOL && spd[1] < JIEFA_RETURN_SPD_TOL && spd[2] < JIEFA_RETURN_SPD_TOL
						&& fabsf(Platform->fdb.front_pitch_angle - jiefa_start_angle[0]) < 2.0f
						&& fabsf(Platform->fdb.left_pitch_angle  - jiefa_start_angle[1]) < 2.0f
						&& fabsf(Platform->fdb.right_pitch_angle - jiefa_start_angle[2]) < 2.0f) {
            hit_state = STATE_IDLE;
            Platform_Set_OutputState(Platform_slow);
        } else {
            Platform_Cal_3Degree_IK_Output();
            keep_state_flag = 0.0f;
        }
    }
}

// 颠球调度函数（传入distance，且时间参数如 0.8s 和轨迹参数如 +=0.05 不同）
void Platform_Dianqiu_Cal(float hit, uint8_t distance) {
	Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
	if (finishhit == 0 ){
		if(DWT_GetTimeline_s() - start_hit_time  <= 0.8f) {
			switch (distance){		
				case Remote_SWITCH_UP://老的PID驱动模式
					dianqiu_mid_active = 0;
                    dianqiu_down_active = 0;
					l_traj_z +=0.30f;
					LimitMax (l_traj_z,0.3f);
					Platform_Set_Target_Pos(0,0,l_traj_z,-30.0f,0,0);
					Platform_Set_OutputState(Platform_fast);
					break ;
			    case Remote_SWITCH_MIDDLE:
			    	// 首次进入时捕获三电机当前角度，之后每帧用双环 PID 驱动到各自目标
			    	if (!dianqiu_mid_active) {
			    		dianqiu_mid_target[0] = Platform->fdb.front_pitch_angle + 80.0f;
			    		dianqiu_mid_target[1] = Platform->fdb.left_pitch_angle  + 40.0f;
			    		dianqiu_mid_target[2] = Platform->fdb.right_pitch_angle + 40.0f;
			    		dianqiu_mid_active = 1;
			    	}
			    	Platform_Set_OutputState(Platform_fast);
			    	Platform_Set_Angle_Output(dianqiu_mid_target[0], dianqiu_mid_target[1], dianqiu_mid_target[2], 0, 120, 240);
				  	break ;
			  	case Remote_SWITCH_DOWN:
			  		if (!dianqiu_down_active) {
			  			dianqiu_down_start[0]  = Platform->fdb.front_pitch_angle;
			  			dianqiu_down_start[1]  = Platform->fdb.left_pitch_angle;
			  			dianqiu_down_start[2]  = Platform->fdb.right_pitch_angle;
			  			dianqiu_down_target[0] = dianqiu_down_start[0] + DIANQIU_DOWN_FRONT_DEG;
			  			dianqiu_down_target[1] = dianqiu_down_start[1] + DIANQIU_DOWN_LEFT_DEG;
			  			dianqiu_down_target[2] = dianqiu_down_start[2] + DIANQIU_DOWN_RIGHT_DEG;
			  			dianqiu_down_active = 1;
			  		}
			  		{
			  			float cur[3] = {Platform->fdb.front_pitch_angle, Platform->fdb.left_pitch_angle, Platform->fdb.right_pitch_angle};
			  			float progress[3], fastest = 0.0f;
			  			for (int i = 0; i < 3; i++) {
			  				float range = dianqiu_down_target[i] - dianqiu_down_start[i];
			  				if (fabsf(range) > 0.01f) {
			  					progress[i] = (cur[i] - dianqiu_down_start[i]) / range;
			  					if (progress[i] < 0.0f) progress[i] = 0.0f;
			  					if (progress[i] > 1.0f) progress[i] = 1.0f;
			  				} else progress[i] = 1.0f;
			  				if (progress[i] > fastest) fastest = progress[i]; // 选取最快的进度作为整体进度，确保三电机同步完成动作
			  			}
			  			float ref[3];
                        for (int i = 0; i < 3; i++) {
                            // 自己的完整目标（保底，不会停在起点）
                            float own_ref = dianqiu_down_target[i];
                            // 按最快进度算出的同步参考（助推慢电机）
                            float push_ref = dianqiu_down_start[i] + fastest * (dianqiu_down_target[i] - dianqiu_down_start[i]);
                            // 取更远的那个：快电机用自己的目标，慢电机被推到前面
                            ref[i] = (push_ref > own_ref) ? push_ref : own_ref;
                        }
			  			Platform_Set_OutputState(Platform_fast);
			  			Platform_Set_Angle_Output(ref[0], ref[1], ref[2], 0, 120, 240);
			  		}
				  	break;
		 	}
		} else {
			dianqiu_mid_active = 0;
            dianqiu_down_active = 0;
			Platform_Set_OutputState(Platform_slow);
			Platform_Set_Target_Pos(0,0,Initial_Z_Position,0,0,0);
		  	start_back_time = DWT_GetTimeline_s();
		  	finishhit = 1;
	  }
 	} else if (finishhit == 1) {
		if(DWT_GetTimeline_s() - start_back_time <= 1.0f) { // 从返回到现在是否未超过1.5秒，保持“冷却状态”
			Platform_Set_OutputState(Platform_slow);
		} else {
		  	if((int)hit == 1) {
		  		// 确保平台在初始位置
			  	Platform_Set_Target_Pos(0,0,Initial_Z_Position,0,0,0);
		    	Platform_Set_OutputState(Platform_middle);
				start_hit_time = DWT_GetTimeline_s();
				finishhit = 0;
			} else if((int)hit != 1) {
				Platform_Set_Target_Pos(0,0,Initial_Z_Position,0,0,0);
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
            if (!dianqiu_mid_active && !dianqiu_down_active) {
		  	    Platform_Cal_3Degree_IK_Output();
            }
      		break;
		case Platform_Jiefa:
            // 修复2：平台主控制循环应主动调用状态机，防止未下发 hit 标志时底盘失控
            // 这里将 Jiefa 的结算放生在 Platform_Task 中，使其以稳定的频率运行输出计算
            Platform_Jiefa_Cal(Platform->flag);

            // 如果状态机目前是空闲的，说明不需要纯力矩/锁死计算，通过 IK 保持位置
            if (hit_state == STATE_IDLE) {
                Platform_Cal_3Degree_IK_Output();
            }
			break;				
		case Platform_Chuanqiu:
			Platform_Set_OutputState(Platform_fast); 
			Platform_Cal_3Degree_IK_Output(); 
			break;				
		case Platform_Initpose: // 复位模式
			start_hit_time = 0.0f;
			start_back_time = 0.0f;
			finishhit = 1;
			l_traj_z = Initial_Z_Position;
            hit_state = STATE_IDLE;
			Platform_Set_OutputState(Platform_slow);  
			Platform_Set_Target_Pos(0,0,Initial_Z_Position,0,0,0);  
			Platform_Cal_3Degree_IK_Output(); 
			break;
		case Platform_Stop:
			Platform_Set_OutputState(Platform_stop);
			// 直接将 A1 电机力矩设为 0（不通过 PID）
			Motor_SetMotorOutput(&Motor_Pitch_Front_Motor_A1, 0.0f);
			Motor_SetMotorOutput(&Motor_Pitch_Left_Motor_A1, 0.0f);
			Motor_SetMotorOutput(&Motor_Pitch_Right_Motor_A1, 0.0f);
			break;				
		default:
			break;
	}
}

// 输出分组节流配置：用于在发送不同电机组数据之间留出时间间隔，避免总线/电源瞬时拥塞。
// A1电机RS485通信时序：发送34字节命令 + 等待电机处理 + 接收78字节反馈
// 按4.5Mbps波特率：发送约76us，接收约173us，加上电机处理时间，建议间隔>=500us
#ifndef PLATFORM_A1_OUTPUT_GAP_US
#define PLATFORM_A1_OUTPUT_GAP_US 500U  // A1电机发送间隔，需要等待反馈完成
#endif

// 为三个电机发送数据
void Platform_Output() {
    Platform_DataTypeDef *Platform = Platform_GetPlatformPtr(); 
        
/*//while( (output_state !=1) && (DWT_GetTimeline_us()-last_output_time < 150)){}
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
    // last_output_time = DWT_GetTimeline_us();*/
    
    // A1电机发送：必须保证每个电机命令发送后有足够时间接收反馈
    // RS485是半双工，发送期间无法接收，需要等待反馈数据完整接收后再发下一个
    Motor_SendMotorGroupOutput(&Motor_Pitch_Front_Motors_A1);
    DWT_Delayus(PLATFORM_A1_OUTPUT_GAP_US);  // 等待Front电机反馈
    Motor_SendMotorGroupOutput(&Motor_Pitch_Right_Motors_A1);
    DWT_Delayus(PLATFORM_A1_OUTPUT_GAP_US);  // 等待Right电机反馈
    Motor_SendMotorGroupOutput(&Motor_Pitch_Left_Motors_A1);
    DWT_Delayus(PLATFORM_A1_OUTPUT_GAP_US);  // 等待Left电机反馈


    
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
