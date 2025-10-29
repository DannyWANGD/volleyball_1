#include "app_gimbal.h"
#include "periph_motor.h"
#include "sys_dwt.h"
#include "sys_const.h"

Gimbal_DataTypeDef GimbalData;
//float yaw = 0.0f, pitch = 0.0f;
void Gimbal_Task(void const * argument)
{
  /* USER CODE BEGIN Gimbal_Task */
	DWT_DataTypeDef *dwt = DWT_GetDWTDataPtr();
	
  /* Infinite loop */
  for(;;)
  {
	//Gimbal_SetPos(yaw, pitch);
//	GimbalData.update_dt = DWT_GetDeltaT(&GimbalData.last_update_time);
    
//	GimbalData.Yaw_fdb = GimbalData.yaw_motorGroup->motor_handle[0]->encoder.angle - 15.0f;
//    GimbalData.Pitch_fdb = GimbalData.pitch_motorGroup->motor_handle[0]->encoder.angle + 153.0f - 360;
//	Gimbal_SendOutput();
	osDelay(2);
  }
  /* USER CODE END Gimbal_Task */
}

void Gimbal_Init(void)
{
	GimbalData.yaw_motorGroup = Motor_groupHandle[4];
	GimbalData.listen_mode  = Gimbal_OffListening;
	GimbalData.init = 1;
	GimbalData.pitch_motorGroup = Motor_groupHandle[5];
	GimbalData.listen_mode  = Gimbal_OffListening;
	GimbalData.init = 1;
	
	GimbalData.Pitch_fdb = 0.0f;
	GimbalData.Pole_Pitch_ref = 0.0f;
	GimbalData.Yaw_fdb = 0.0f;
	GimbalData.Pole_Yaw_ref = 0.0f;
}


void Gimbal_SetPos()
{
	Gimbal_DataTypeDef * gimbal = Gimbal_GetDataPtr();
	float yaw = gimbal->Pole_Yaw_ref;
	LimitMaxMin(yaw, 200, -200);
	yaw = yaw + 15.0f;
	Motor_SetMotorOutput(gimbal->yaw_motorGroup->motor_handle[0], (int32_t)(yaw*100));
	
	//now_yaw = data_yaw->motorGroup->motor_handle[0]->encoder.angle + 40;

	float pitch = gimbal->Pole_Pitch_ref;
	LimitMaxMin(pitch ,35,-27)	;	//pitch > 140 && pitch < 225
	
	pitch = pitch -153 ;
	Motor_SetMotorOutput(gimbal->pitch_motorGroup->motor_handle[0], (int32_t)(pitch*100));		

		//now_pitch = data_pitch->motorGroup->motor_handle[0]->encoder.angle - 147;
	
	
 }
int jjjjjj = 0;
void Gimbal_SendOutput() {
	Gimbal_DataTypeDef * gimbal = Gimbal_GetDataPtr();
	Motor_SendMotorGroupOutput(gimbal->yaw_motorGroup);
	Motor_SendMotorGroupOutput(gimbal->pitch_motorGroup);	
	jjjjjj ++;
}

//void Gimbal_GetPos(void)
//{
//	Gimbal_StateTypeDef * state_yaw = Gimbal_GetStateYawPtr();
//	Gimbal_StateTypeDef * state_pitch = Gimbal_GetStatePitchPtr();
//	Gimbal_DataTypeDef * data_yaw = Gimbal_GetDataYawPtr();
//	Gimbal_DataTypeDef * data_pitch = Gimbal_GetDataPitchPtr();
//	
//	state_yaw->Yaw_fdb = data_yaw->motorGroup->motor_handle[0]->encoder.angle + 40;
//    state_pitch->Pitch_fdb = data_pitch->motorGroup->motor_handle[0]->encoder.angle - 147;
//}

Gimbal_DataTypeDef* Gimbal_GetDataPtr(void)
{
    return &GimbalData;
}

