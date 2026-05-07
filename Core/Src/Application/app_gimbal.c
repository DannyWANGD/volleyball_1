#include "app_gimbal.h"
#include "periph_remote.h"
#include "periph_motor.h"
#include "sys_dwt.h"
#include "sys_const.h"
#include "comm_receive.h"

Gimbal_TypeDef Gimbal;
void Gimbal_Task(void const * argument)
{
  /* USER CODE BEGIN Gimbal_Task */
	DWT_DataTypeDef *dwt = DWT_GetDWTDataPtr();
	Gimbal_TypeDef * gimbal_data = Gimbal_GetPtr();
	osDelay(1000);
	
  for(;;)
  {
	Gimbal.update_dt = DWT_GetDeltaT(&Gimbal.last_update_time);
	if(Gimbal.gimbal_state != Gimbal_Off) {
		Gimbal_SetPos();
		Gimbal_PIDCalc();
		Motor_SendMotorGroupOutput(gimbal_data->motorGroup);
	}
    osDelay(2);
  }
  /* USER CODE END Gimbal_Task */
}

void Gimbal_Init(void)
{
	PID_InitPIDParam(&Gimbal.Yaw_Pospidparam, Const_YawPosMotorParam[0][0], Const_YawPosMotorParam[0][1], Const_YawPosMotorParam[0][2], Const_YawPosMotorParam[0][3], Const_YawPosMotorParam[0][4],
                    Const_YawPosMotorParam[1][0], Const_YawPosMotorParam[1][1], Const_YawPosMotorParam[2][0], Const_YawPosMotorParam[2][1],
                    Const_YawPosMotorParam[3][0],Const_YawPosMotorParam[3][1], PID_POSITION);
	PID_InitPIDParam(&Gimbal.Yaw_Spdpidparam, Const_YawSpdMotorParam[0][0], Const_YawSpdMotorParam[0][1], Const_YawSpdMotorParam[0][2], Const_YawSpdMotorParam[0][3], Const_YawSpdMotorParam[0][4],
                    Const_YawSpdMotorParam[1][0], Const_YawSpdMotorParam[1][1], Const_YawSpdMotorParam[2][0], Const_YawSpdMotorParam[2][1],
                    Const_YawSpdMotorParam[3][0],Const_YawSpdMotorParam[3][1], PID_POSITION);
	PID_InitPIDParam(&Gimbal.Pitch_Pospidparam, Const_PitchPosMotorParam[0][0], Const_PitchPosMotorParam[0][1], Const_PitchPosMotorParam[0][2], Const_PitchPosMotorParam[0][3], Const_PitchPosMotorParam[0][4],
                    Const_PitchPosMotorParam[1][0], Const_PitchPosMotorParam[1][1], Const_PitchPosMotorParam[2][0], Const_PitchPosMotorParam[2][1],
                    Const_PitchPosMotorParam[3][0],Const_PitchPosMotorParam[3][1], PID_POSITION);
	PID_InitPIDParam(&Gimbal.Pitch_Spdpidparam, Const_PitchSpdMotorParam[0][0], Const_PitchSpdMotorParam[0][1], Const_PitchSpdMotorParam[0][2], Const_PitchSpdMotorParam[0][3], Const_PitchSpdMotorParam[0][4],
                    Const_PitchSpdMotorParam[1][0], Const_PitchSpdMotorParam[1][1], Const_PitchSpdMotorParam[2][0], Const_PitchSpdMotorParam[2][1],
                    Const_PitchSpdMotorParam[3][0],Const_PitchSpdMotorParam[3][1], PID_POSITION);
	
	Gimbal.motorGroup = Motor_groupHandle[9];
	Gimbal.init = 1;
	Gimbal.gimbal_state = Gimbal_Off;
	Gimbal.Pitch_fdb = 0.0f;
	Gimbal.Pitch_ref = 0.0f;
	Gimbal.Yaw_fdb = 0.0f;
	Gimbal.Yaw_ref = 0.0f;

	//init_average_filter();
}

void Gimbal_SetPos()
{	
	// Let see the robot from behind, then for Gimbal,the origin state is
	// As for Yaw, Turn Left is Positive, Turn Right is Negative
	// As for Pitch, Turn Up is Positive, Turn Down is Negative
	// Aim to provide better use for human
	// we overturn the Pitch order, and keep the Yaw order
	Remote_DataTypeDef *rc = Remote_GetRemotePtr(); 

	Gimbal_TypeDef * gimbal_data = Gimbal_GetPtr();
	Remote_RemoteDataTypeDef* remote = Remote_GetRemoteDataPtr();
	
	if(gimbal_data->gimbal_state == Gimbal_remote) {
		gimbal_data->Pitch_ref = -(rc->ch[1] / 660.0f * 45.0f);
		gimbal_data->Yaw_ref = -(rc->ch[0] / 660.0f * 90.0f);
	}
	else if(gimbal_data->gimbal_state == Gimbal_auto) {
		
	}
	Gimbal_PosLimit();
	
}

void Gimbal_PosLimit(void)
{
	Gimbal_TypeDef * gimbal_data = Gimbal_GetPtr();
	
	LimitMaxMin(gimbal_data->Yaw_ref, 90, -90);
	LimitMaxMin(gimbal_data->Pitch_ref, 43, -45);
	
}

	
Gimbal_TypeDef* Gimbal_GetPtr(void)
{
    return &Gimbal;
}

void Gimbal_StateSet(Gimbal_State state)
{
	Gimbal_TypeDef * gimbal_data = Gimbal_GetPtr();
	
	gimbal_data->gimbal_state = state;
}

void Gimbal_PIDCalc(void)
{
	Gimbal_TypeDef * gimbal_data = Gimbal_GetPtr();
		
	// 1. 获取你原本认为“正确”的反馈
    float raw_yaw_fdb = (gimbal_data->motorGroup->motor_handle[0]->encoder.consequent_angle - Const_Gimbal_Yaw_OFFSET);
    
    // 2. 【核心保险丝】：最短路径误差映射
    // 这一步能解决：
    // - round_count 误跳变 360 度
    // - init_offset 异常导致的 360 度偏移
    // - 任何导致反馈值瞬间“多出一圈”的软件 Bug
    float yaw_error = gimbal_data->Yaw_ref - raw_yaw_fdb;
    
    while (yaw_error > 180.0f)  yaw_error -= 360.0f;
    while (yaw_error < -180.0f) yaw_error += 360.0f;
	
	gimbal_data->Yaw_fdb = (gimbal_data->motorGroup->motor_handle[0]->encoder.angle - Const_Gimbal_Yaw_OFFSET)  ;
	
	gimbal_data->Pitch_fdb = (gimbal_data->motorGroup->motor_handle[1]->encoder.angle - Const_Gimbal_Pitch_OFFSET);

	PID_SetPIDRef(&gimbal_data->Yaw_Pospid, gimbal_data->Yaw_ref);
	PID_SetPIDFdb(&gimbal_data->Yaw_Pospid, gimbal_data->Yaw_fdb);
	PID_CalcPID(&gimbal_data->Yaw_Pospid, &gimbal_data->Yaw_Pospidparam);	
	PID_SetPIDRef(&gimbal_data->Yaw_Spdpid, PID_GetPIDOutput(&gimbal_data->Yaw_Pospid));
	PID_SetPIDFdb(&gimbal_data->Yaw_Spdpid, gimbal_data->motorGroup->motor_handle[0]->encoder.speed);
	PID_CalcPID(&gimbal_data->Yaw_Spdpid, &gimbal_data->Yaw_Spdpidparam);
	
	PID_SetPIDRef(&gimbal_data->Pitch_Pospid, gimbal_data->Pitch_ref);
	PID_SetPIDFdb(&gimbal_data->Pitch_Pospid, gimbal_data->Pitch_fdb);
	PID_CalcPID(&gimbal_data->Pitch_Pospid, &gimbal_data->Pitch_Pospidparam);
	PID_SetPIDRef(&gimbal_data->Pitch_Spdpid, PID_GetPIDOutput(&gimbal_data->Pitch_Pospid));
	PID_SetPIDFdb(&gimbal_data->Pitch_Spdpid, gimbal_data->motorGroup->motor_handle[1]->encoder.speed);
	PID_CalcPID(&gimbal_data->Pitch_Spdpid, &gimbal_data->Pitch_Spdpidparam);
	
	Motor_SetMotorOutput(gimbal_data->motorGroup->motor_handle[0],  -PID_GetPIDOutput(&gimbal_data->Yaw_Spdpid));

	Motor_SetMotorOutput(gimbal_data->motorGroup->motor_handle[1],  -PID_GetPIDOutput(&gimbal_data->Pitch_Spdpid));
}
