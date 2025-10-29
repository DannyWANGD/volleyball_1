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
	Platform->tar.plat_z = 0.1;
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
float l_tar = 0.13f;

// 新增：击球位姿控制相关变量
static float hitting_start_time = 0.0f;
static float hitting_return_time = 0.0f;
static uint8_t hitting_state = 0; // 0-空闲, 1-击球中, 2-回位中
static float target_roll = 0.0f, target_pitch = 0.0f, target_yaw = 0.0f;
static uint8_t target_speed = 2;

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
void Platform_Set_Torque_Output(float torque1,float torque2,float torque3,float torque4,float torque5,float torque6){
	
	// 输出力矩限幅
	LimitMaxMin(torque1,23.7f,-23.7f);
	LimitMaxMin(torque2,23.7f,-23.7f);
	LimitMaxMin(torque3,23.7f,-23.7f);
	LimitMaxMin(torque4,27.0f,-27.0f);
	LimitMaxMin(torque5,27.0f,-27.0f);
	LimitMaxMin(torque6,27.0f,-27.0f);
	
	// 将力矩值写入电机对象的数据结构中，等待 Platform_Output 函数将其发送出去。
  	Motor_SetMotorOutput(&Motor_Pitch_Front_Motor,torque1) ;  
	Motor_SetMotorOutput(&Motor_Pitch_Left_Motor ,torque2) ; 
	Motor_SetMotorOutput(&Motor_Pitch_Right_Motor,torque3) ; 
	 
  	Motor_SetMotorOutput(&Motor_Yaw_Front_Motor,torque4) ;
  	Motor_SetMotorOutput(&Motor_Yaw_Left_Motor ,torque5) ;
  	Motor_SetMotorOutput(&Motor_Yaw_Right_Motor,torque6) ;

 }

// 实现位置-速度串级双环 PID 控制器，是整个闭环控制的核心算法
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
				// 设置角度环的目标值 Reference，即我们期望电机到达的角度
			PID_SetPIDRef(&Platform->pid.Pitch1_Ang_PID, ang1);
				// 设置角度环的反馈值 Feedback，即从传感器读到的当前实际角度
			PID_SetPIDFdb(&Platform->pid.Pitch1_Ang_PID,Platform->fdb.front_pitch_angle); 
				// 使用慢速模式的角度PID参数，这一步的输出不是力矩，而是一个期望的电机转速
			PID_CalcPID(&Platform->pid.Pitch1_Ang_PID, &Platform->pid.Pitch_Ang_Slow_PIDParam);
				// 将角度环的输出设置为速度环的目标值
			PID_SetPIDRef(&Platform ->pid.Pitch1_Spd_PID,PID_GetPIDOutput(&Platform->pid.Pitch1_Ang_PID) );
				// 设置速度环的反馈值，即从电机编码器读到的当前实际转速
				// 前面的负号(-)很可能是因为电机安装方向或者编码器读数方向与坐标系定义相反，用来校正方向
			PID_SetPIDFdb(&Platform ->pid.Pitch1_Spd_PID, - Motor_Pitch_Front_Motor.encoder.speed);
				// 使用慢速模式的速度PID参数，这一步的输出，才是最终控制电机转动的物理量——力矩
			PID_CalcPID(&Platform ->pid.Pitch1_Spd_PID, &Platform ->pid.Pitch_Spd_Slow_PIDParam);
				// 计算最终的扭矩，负号用于方向矫正？“-1.5f”是用于抵抗重力的偏置项？
			torque1 = -PID_GetPIDOutput(&Platform->pid.Pitch1_Spd_PID)-1.5f;
  
			PID_SetPIDRef(&Platform->pid.Pitch2_Ang_PID, ang2);
			PID_SetPIDFdb(&Platform->pid.Pitch2_Ang_PID,Platform->fdb.left_pitch_angle);
			PID_CalcPID(&Platform->pid.Pitch2_Ang_PID, &Platform->pid.Pitch_Ang_Slow_PIDParam);
			PID_SetPIDRef(&Platform ->pid.Pitch2_Spd_PID,PID_GetPIDOutput(&Platform->pid.Pitch2_Ang_PID) );
			PID_SetPIDFdb(&Platform ->pid.Pitch2_Spd_PID, -Motor_Pitch_Left_Motor.encoder.speed);
			PID_CalcPID(&Platform ->pid.Pitch2_Spd_PID, &Platform ->pid.Pitch_Spd_Slow_PIDParam);
			torque2 = -PID_GetPIDOutput(&Platform->pid.Pitch2_Spd_PID)-1.5f;
		
			PID_SetPIDRef(&Platform->pid.Pitch3_Ang_PID, ang3);
			PID_SetPIDFdb(&Platform->pid.Pitch3_Ang_PID,Platform->fdb.right_pitch_angle);
			PID_CalcPID(&Platform->pid.Pitch3_Ang_PID, &Platform->pid.Pitch_Ang_Slow_PIDParam);
			PID_SetPIDRef(&Platform ->pid.Pitch3_Spd_PID,PID_GetPIDOutput(&Platform->pid.Pitch3_Ang_PID) );
			PID_SetPIDFdb(&Platform ->pid.Pitch3_Spd_PID, Motor_Pitch_Right_Motor.encoder.speed);
			PID_CalcPID(&Platform ->pid.Pitch3_Spd_PID, &Platform ->pid.Pitch_Spd_Slow_PIDParam);
			torque3 = PID_GetPIDOutput(&Platform->pid.Pitch3_Spd_PID)+1.5f;

			PID_SetPIDRef(&Platform->pid.Yaw1_Ang_PID, ang4);
			PID_SetPIDFdb(&Platform->pid.Yaw1_Ang_PID,Platform->fdb.front_yaw_angle);
			PID_CalcPID(&Platform->pid.Yaw1_Ang_PID, &Platform->pid.Yaw_Ang_Slow_PIDParam);
			PID_SetPIDRef(&Platform ->pid.Yaw1_Spd_PID,PID_GetPIDOutput(&Platform->pid.Yaw1_Ang_PID) );
			PID_SetPIDFdb(&Platform ->pid.Yaw1_Spd_PID, Motor_Yaw_Front_Motor.encoder.speed);
			PID_CalcPID(&Platform ->pid.Yaw1_Spd_PID, &Platform ->pid.Yaw_Spd_Slow_PIDParam);
			torque4 = PID_GetPIDOutput(&Platform->pid.Yaw1_Spd_PID);
		
			PID_SetPIDRef(&Platform->pid.Yaw2_Ang_PID, ang5);
			PID_SetPIDFdb(&Platform->pid.Yaw2_Ang_PID,Platform->fdb.left_yaw_angle);
			PID_CalcPID(&Platform->pid.Yaw2_Ang_PID, &Platform->pid.Yaw_Ang_Slow_PIDParam);
			PID_SetPIDRef(&Platform ->pid.Yaw2_Spd_PID,PID_GetPIDOutput(&Platform->pid.Yaw2_Ang_PID) );
			PID_SetPIDFdb(&Platform ->pid.Yaw2_Spd_PID, Motor_Yaw_Left_Motor.encoder.speed);
			PID_CalcPID(&Platform ->pid.Yaw2_Spd_PID, &Platform ->pid.Yaw_Spd_Slow_PIDParam);
			torque5 = PID_GetPIDOutput(&Platform->pid.Yaw2_Spd_PID)+0.5;
		
			PID_SetPIDRef(&Platform->pid.Yaw3_Ang_PID, ang6);
			PID_SetPIDFdb(&Platform->pid.Yaw3_Ang_PID,Platform->fdb.right_yaw_angle);
			PID_CalcPID(&Platform->pid.Yaw3_Ang_PID, &Platform->pid.Yaw_Ang_Slow_PIDParam);
			PID_SetPIDRef(&Platform ->pid.Yaw3_Spd_PID,PID_GetPIDOutput(&Platform->pid.Yaw3_Ang_PID) );
			PID_SetPIDFdb(&Platform ->pid.Yaw3_Spd_PID, Motor_Yaw_Right_Motor.encoder.speed);
			PID_CalcPID(&Platform ->pid.Yaw3_Spd_PID, &Platform ->pid.Yaw_Spd_Slow_PIDParam);
			torque6 = PID_GetPIDOutput(&Platform->pid.Yaw3_Spd_PID)-0.5;
			
			break;

		case Platform_middle:
			PID_SetPIDRef(&Platform->pid.Pitch1_Ang_PID, ang1);
			PID_SetPIDFdb(&Platform->pid.Pitch1_Ang_PID,Platform->fdb.front_pitch_angle); 
			PID_CalcPID(&Platform->pid.Pitch1_Ang_PID, &Platform->pid.Pitch_Ang_Middle_PIDParam);
			PID_SetPIDRef(&Platform ->pid.Pitch1_Spd_PID,PID_GetPIDOutput(&Platform->pid.Pitch1_Ang_PID) );
			PID_SetPIDFdb(&Platform ->pid.Pitch1_Spd_PID, -Motor_Pitch_Front_Motor.encoder.speed);
			PID_CalcPID(&Platform ->pid.Pitch1_Spd_PID, &Platform ->pid.Pitch_Spd_Middle_PIDParam);
			torque1 = -PID_GetPIDOutput(&Platform->pid.Pitch1_Spd_PID)-1.5f;
			
			PID_SetPIDRef(&Platform->pid.Pitch2_Ang_PID, ang2);
			PID_SetPIDFdb(&Platform->pid.Pitch2_Ang_PID,Platform->fdb.left_pitch_angle);
			PID_CalcPID(&Platform->pid.Pitch2_Ang_PID, &Platform->pid.Pitch_Ang_Middle_PIDParam);
			PID_SetPIDRef(&Platform ->pid.Pitch2_Spd_PID,PID_GetPIDOutput(&Platform->pid.Pitch2_Ang_PID) );
			PID_SetPIDFdb(&Platform ->pid.Pitch2_Spd_PID, -Motor_Pitch_Left_Motor.encoder.speed);
			PID_CalcPID(&Platform ->pid.Pitch2_Spd_PID, &Platform ->pid.Pitch_Spd_Middle_PIDParam);
			torque2 = -PID_GetPIDOutput(&Platform->pid.Pitch2_Spd_PID)-1.5f;
			
			PID_SetPIDRef(&Platform->pid.Pitch3_Ang_PID, ang3);
			PID_SetPIDFdb(&Platform->pid.Pitch3_Ang_PID,Platform->fdb.right_pitch_angle);
			PID_CalcPID(&Platform->pid.Pitch3_Ang_PID, &Platform->pid.Pitch_Ang_Middle_PIDParam);
			PID_SetPIDRef(&Platform ->pid.Pitch3_Spd_PID,PID_GetPIDOutput(&Platform->pid.Pitch3_Ang_PID) );
			PID_SetPIDFdb(&Platform ->pid.Pitch3_Spd_PID, Motor_Pitch_Right_Motor.encoder.speed);
			PID_CalcPID(&Platform ->pid.Pitch3_Spd_PID, &Platform ->pid.Pitch_Spd_Middle_PIDParam);
			torque3 = PID_GetPIDOutput(&Platform->pid.Pitch3_Spd_PID)+1.5f;

			PID_SetPIDRef(&Platform->pid.Yaw1_Ang_PID, ang4);
			PID_SetPIDFdb(&Platform->pid.Yaw1_Ang_PID,Platform->fdb.front_yaw_angle);
			PID_CalcPID(&Platform->pid.Yaw1_Ang_PID, &Platform->pid.Yaw_Ang_Middle_PIDParam);
			PID_SetPIDRef(&Platform ->pid.Yaw1_Spd_PID,PID_GetPIDOutput(&Platform->pid.Yaw1_Ang_PID) );
			PID_SetPIDFdb(&Platform ->pid.Yaw1_Spd_PID, Motor_Yaw_Front_Motor.encoder.speed);
			PID_CalcPID(&Platform ->pid.Yaw1_Spd_PID, &Platform ->pid.Yaw_Spd_Middle_PIDParam);
			torque4 = PID_GetPIDOutput(&Platform->pid.Yaw1_Spd_PID);
			
			PID_SetPIDRef(&Platform->pid.Yaw2_Ang_PID, ang5);
			PID_SetPIDFdb(&Platform->pid.Yaw2_Ang_PID,Platform->fdb.left_yaw_angle);
			PID_CalcPID(&Platform->pid.Yaw2_Ang_PID, &Platform->pid.Yaw_Ang_Middle_PIDParam);
			PID_SetPIDRef(&Platform ->pid.Yaw2_Spd_PID,PID_GetPIDOutput(&Platform->pid.Yaw2_Ang_PID) );
			PID_SetPIDFdb(&Platform ->pid.Yaw2_Spd_PID, Motor_Yaw_Left_Motor.encoder.speed);
			PID_CalcPID(&Platform ->pid.Yaw2_Spd_PID, &Platform ->pid.Yaw_Spd_Middle_PIDParam);
			torque5 = PID_GetPIDOutput(&Platform->pid.Yaw2_Spd_PID)+0.5;
			
			PID_SetPIDRef(&Platform->pid.Yaw3_Ang_PID, ang6);
			PID_SetPIDFdb(&Platform->pid.Yaw3_Ang_PID,Platform->fdb.right_yaw_angle);
			PID_CalcPID(&Platform->pid.Yaw3_Ang_PID, &Platform->pid.Yaw_Ang_Middle_PIDParam);
			PID_SetPIDRef(&Platform ->pid.Yaw3_Spd_PID,PID_GetPIDOutput(&Platform->pid.Yaw3_Ang_PID) );
			PID_SetPIDFdb(&Platform ->pid.Yaw3_Spd_PID, Motor_Yaw_Right_Motor.encoder.speed);
			PID_CalcPID(&Platform ->pid.Yaw3_Spd_PID, &Platform ->pid.Yaw_Spd_Middle_PIDParam);
			torque6 = PID_GetPIDOutput(&Platform->pid.Yaw3_Spd_PID)-0.5;
			
			break;

		case Platform_fast:
			PID_SetPIDRef(&Platform->pid.Pitch1_Ang_PID, ang1);
			PID_SetPIDFdb(&Platform->pid.Pitch1_Ang_PID,Platform->fdb.front_pitch_angle); 
			PID_CalcPID(&Platform->pid.Pitch1_Ang_PID, &Platform->pid.Pitch_Ang_Fast_PIDParam);
			PID_SetPIDRef(&Platform ->pid.Pitch1_Spd_PID,PID_GetPIDOutput(&Platform->pid.Pitch1_Ang_PID) );
			PID_SetPIDFdb(&Platform ->pid.Pitch1_Spd_PID, -Motor_Pitch_Front_Motor.encoder.speed);
			PID_CalcPID(&Platform ->pid.Pitch1_Spd_PID, &Platform ->pid.Pitch_Spd_Fast_PIDParam);
			torque1 = -PID_GetPIDOutput(&Platform->pid.Pitch1_Spd_PID)-1.5f;
			
			PID_SetPIDRef(&Platform->pid.Pitch2_Ang_PID, ang2);
			PID_SetPIDFdb(&Platform->pid.Pitch2_Ang_PID,Platform->fdb.left_pitch_angle);
			PID_CalcPID(&Platform->pid.Pitch2_Ang_PID, &Platform->pid.Pitch_Ang_Fast_PIDParam);
			PID_SetPIDRef(&Platform ->pid.Pitch2_Spd_PID,PID_GetPIDOutput(&Platform->pid.Pitch2_Ang_PID) );
			PID_SetPIDFdb(&Platform ->pid.Pitch2_Spd_PID, -Motor_Pitch_Left_Motor.encoder.speed);
			PID_CalcPID(&Platform ->pid.Pitch2_Spd_PID, &Platform ->pid.Pitch_Spd_Fast_PIDParam);
			torque2 = -PID_GetPIDOutput(&Platform->pid.Pitch2_Spd_PID)-1.5f;
			
			PID_SetPIDRef(&Platform->pid.Pitch3_Ang_PID, ang3);
			PID_SetPIDFdb(&Platform->pid.Pitch3_Ang_PID,Platform->fdb.right_pitch_angle);
			PID_CalcPID(&Platform->pid.Pitch3_Ang_PID, &Platform->pid.Pitch_Ang_Fast_PIDParam);
			PID_SetPIDRef(&Platform ->pid.Pitch3_Spd_PID,PID_GetPIDOutput(&Platform->pid.Pitch3_Ang_PID) );
			PID_SetPIDFdb(&Platform ->pid.Pitch3_Spd_PID, Motor_Pitch_Right_Motor.encoder.speed);
			PID_CalcPID(&Platform ->pid.Pitch3_Spd_PID, &Platform ->pid.Pitch_Spd_Fast_PIDParam);
			torque3 = PID_GetPIDOutput(&Platform->pid.Pitch3_Spd_PID)+1.5f;

			PID_SetPIDRef(&Platform->pid.Yaw1_Ang_PID, ang4);
			PID_SetPIDFdb(&Platform->pid.Yaw1_Ang_PID,Platform->fdb.front_yaw_angle);
			PID_CalcPID(&Platform->pid.Yaw1_Ang_PID, &Platform->pid.Yaw_Ang_Fast_PIDParam);
			PID_SetPIDRef(&Platform ->pid.Yaw1_Spd_PID,PID_GetPIDOutput(&Platform->pid.Yaw1_Ang_PID) );
			PID_SetPIDFdb(&Platform ->pid.Yaw1_Spd_PID, Motor_Yaw_Front_Motor.encoder.speed);
			PID_CalcPID(&Platform ->pid.Yaw1_Spd_PID, &Platform ->pid.Yaw_Spd_Fast_PIDParam);
			torque4 = PID_GetPIDOutput(&Platform->pid.Yaw1_Spd_PID);
			
			PID_SetPIDRef(&Platform->pid.Yaw2_Ang_PID, ang5);
			PID_SetPIDFdb(&Platform->pid.Yaw2_Ang_PID,Platform->fdb.left_yaw_angle);
			PID_CalcPID(&Platform->pid.Yaw2_Ang_PID, &Platform->pid.Yaw_Ang_Fast_PIDParam);
			PID_SetPIDRef(&Platform ->pid.Yaw2_Spd_PID,PID_GetPIDOutput(&Platform->pid.Yaw2_Ang_PID) );
			PID_SetPIDFdb(&Platform ->pid.Yaw2_Spd_PID, Motor_Yaw_Left_Motor.encoder.speed);
			PID_CalcPID(&Platform ->pid.Yaw2_Spd_PID, &Platform ->pid.Yaw_Spd_Fast_PIDParam);
			torque5 = PID_GetPIDOutput(&Platform->pid.Yaw2_Spd_PID)+0.5;
			
			PID_SetPIDRef(&Platform->pid.Yaw3_Ang_PID, ang6);
			PID_SetPIDFdb(&Platform->pid.Yaw3_Ang_PID,Platform->fdb.right_yaw_angle);
			PID_CalcPID(&Platform->pid.Yaw3_Ang_PID, &Platform->pid.Yaw_Ang_Fast_PIDParam);
			PID_SetPIDRef(&Platform ->pid.Yaw3_Spd_PID,PID_GetPIDOutput(&Platform->pid.Yaw3_Ang_PID) );
			PID_SetPIDFdb(&Platform ->pid.Yaw3_Spd_PID, Motor_Yaw_Right_Motor.encoder.speed);
			PID_CalcPID(&Platform ->pid.Yaw3_Spd_PID, &Platform ->pid.Yaw_Spd_Fast_PIDParam);
			torque6 = PID_GetPIDOutput(&Platform->pid.Yaw3_Spd_PID)-0.5;
    
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
	LimitMaxMin(Platform->tar.plat_z,0.37f,0.10f);
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
 
/*
【注意】
这里是后期可以主要更改的地方，在此我们只是传入了一个 hit，即是否击球的标志位，
后面将会传入击球的期望空间位姿 pitch、yaw、roll 和 z 等参数，从而实现更复杂的击球策略。
【疑问】
我该如何解决当位姿一定时击球的空间速度呢？（或者说我该如何控制击球的力度呢？）
*/

// 接发球调度函数
void Platform_Jiefa_Cal(float hit) {
	
  	Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();

	if (finishhit == 0 ){

		if(DWT_GetTimeline_s() - start_hit_time  <= 1.0f) {
			//【疑问：这一步是在干啥？为啥要加那么多？】
			// 是希望在这里快速达到某一高度实现快速击打么？
			l_tar += 0.15;
			LimitMax (l_tar,0.3);
		  	Platform_Set_Target_Pos(0,0,l_tar,0,0,0);
      		Platform_Set_OutputState(Platform_fast);
	 	} else {
			Platform_Set_OutputState(Platform_slow);	
			Platform_Set_Target_Pos(0,0,0.13f,0,0,0);
		  	start_back_time = DWT_GetTimeline_s();	
		  	finishhit = 1;	
	    }

	} else if (finishhit == 1) {

		if(DWT_GetTimeline_s() - start_back_time <= 1.5f){ // 从返回到现在是否未超过1.5秒，保持“冷却状态”
			Platform_Set_OutputState(Platform_slow);
		} else {
			if((int)hit == 1) {
				// 确保平台在初始位置
				Platform_Set_Target_Pos(0,0,0.13f,0,0,0);
				Platform_Set_OutputState(Platform_middle);
				start_hit_time = DWT_GetTimeline_s();
				finishhit = 0;
			} else if ((int)hit != 1) {
				Platform_Set_Target_Pos(0,0,0.13f,0,0,0);
				Platform_Set_OutputState(Platform_middle);
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


// 为三个电极发送数据
void Platform_Output() {
    Platform_DataTypeDef *Platform = Platform_GetPlatformPtr(); 
		
//while( (output_state !=1) && (DWT_GetTimeline_us()-last_output_time < 150)){}
	Motor_SendMotorGroupOutput(Motor_groupHandle[3]) ; 
	Motor_SendMotorGroupOutput(Motor_groupHandle[0]) ;
	// last_output_time = DWT_GetTimeline_us();
	DWT_Delayus(150);
	
//while( (output_state !=2) && (DWT_GetTimeline_us()-last_output_time < 150)){}
	Motor_SendMotorGroupOutput(Motor_groupHandle[4]) ;
	Motor_SendMotorGroupOutput(Motor_groupHandle[1]) ;
	// last_output_time = DWT_GetTimeline_us();
	DWT_Delayus(150);

//while( (output_state !=3) && (DWT_GetTimeline_us()-last_output_time < 150)){}
	Motor_SendMotorGroupOutput(Motor_groupHandle[5]) ;
	Motor_SendMotorGroupOutput(Motor_groupHandle[2]) ;
	// last_output_time = DWT_GetTimeline_us();
	
  }

/////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief      击球板位姿控制函数（带自动回位功能）
 * @param      roll: 击球板期望的横滚角 (度)
 * @param      pitch: 击球板期望的俯仰角 (度) 
 * @param      yaw: 击球板期望的偏航角 (度)
 * @param      hitting_speed: 期望的击球速度等级 (1-慢速, 2-中速, 3-快速)
 * @retval     None
 * @note       该函数设置击球板的空间位姿，击球后自动以慢速回到初始位置
 */
void Platform_Set_Hitting_Pose(float roll, float pitch, float yaw, uint8_t hitting_speed) {
    
    Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
    
    // 如果当前处于空闲状态，开始新的击球动作
    if (hitting_state == 0) {
        // 保存目标参数
        target_roll = roll;
        target_pitch = pitch;
        target_yaw = yaw;
        target_speed = hitting_speed;
        
        // 开始击球动作
        hitting_state = 1;
        hitting_start_time = DWT_GetTimeline_s();
        
        // 设置击球位姿：x=y=0, z=0.2，以及传入的roll、pitch、yaw
        Platform_Set_Target_Pos(0.0f, 0.0f, 0.2f, pitch, yaw, roll);
        
        // 根据击球速度参数设置输出状态
        switch (hitting_speed) {
            case 1:  // 慢速击球
                Platform_Set_OutputState(Platform_slow);
                break;
            case 2:  // 中速击球  
                Platform_Set_OutputState(Platform_middle);
                break;
            case 3:  // 快速击球
                Platform_Set_OutputState(Platform_fast);
                break;
            default: // 默认中速
                Platform_Set_OutputState(Platform_middle);
                break;
        }
        
        // 调用逆运动学解算函数，计算各电机目标角度并启动闭环控制
        Platform_Cal_3Degree_IK_Output();
    }
}

/**
 * @brief      击球位姿控制状态机（需要在主循环中调用）
 * @param      None
 * @retval     None
 * @note       管理击球动作的完整流程：击球 -> 保持 -> 回位 -> 冷却
 */
void Platform_Hitting_Pose_Update(void) {
    
    Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
    float current_time = DWT_GetTimeline_s();
    
    switch (hitting_state) {
        case 1: // 击球中
            if (current_time - hitting_start_time >= 0.8f) { // 击球动作持续0.8秒
                // 开始回位
                hitting_state = 2;
                hitting_return_time = current_time;
                
                // 设置回到初始位置：x=y=0, z=0.13, roll=pitch=yaw=0
                Platform_Set_Target_Pos(0.0f, 0.0f, 0.13f, 0.0f, 0.0f, 0.0f);
                Platform_Set_OutputState(Platform_slow); // 慢速回位
                Platform_Cal_3Degree_IK_Output();
            }
            break;
            
        case 2: // 回位中
            if (current_time - hitting_return_time >= 1.5f) { // 回位和冷却总共1.5秒
                // 回位完成，进入空闲状态
                hitting_state = 0;
                
                // 确保在初始位置
                Platform_Set_Target_Pos(0.0f, 0.0f, 0.13f, 0.0f, 0.0f, 0.0f);
                Platform_Set_OutputState(Platform_middle);
                Platform_Cal_3Degree_IK_Output();
            }
            break;
            
        case 0: // 空闲状态
        default:
            // 保持在初始位置
            Platform_Set_Target_Pos(0.0f, 0.0f, 0.13f, 0.0f, 0.0f, 0.0f);
            Platform_Set_OutputState(Platform_middle);
            break;
    }
}

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
