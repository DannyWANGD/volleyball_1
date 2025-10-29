 # include "app_serve.h"
 #include "util_can.h"

#include "periph_motor.h"
#include "sys_const.h"
#include "sys_dwt.h"
#include "app_remote.h"
#include "cmsis_os.h"
#include "main.h"
#include "math.h"
#include "gpio.h"
#include <stdio.h>

Balc_DataTypeDef  BalcData;
Balc_StateTypeDef BalcState;

void Serve_Task(void const * argument)
{
	DWT_DataTypeDef *dwt = DWT_GetDWTDataPtr();
	for(;;)
    {
		if(BalcData.mode == Balc_HitMode)
		{
			Balc_StartingBatting();
		}
		if(BalcData.mode == Balc_WaitMode)
		{
			Balc_Wait();
		}
		BalcData.update_dt = DWT_GetDeltaT(&BalcData.last_update_time);
		osDelay(3);
    }
}

void Balc_Init(void)
{
    PID_InitPIDParam(&BalcData.L_Pospidparam, Const_LeftPosMotorParam[0][0], Const_RightPosMotorParam[0][1], Const_LeftPosMotorParam[0][2], Const_LeftPosMotorParam[0][3], Const_LeftPosMotorParam[0][4],
                    Const_LeftPosMotorParam[1][0], Const_LeftPosMotorParam[1][1], Const_LeftPosMotorParam[2][0], Const_LeftPosMotorParam[2][1],
                    Const_LeftPosMotorParam[3][0],Const_LeftPosMotorParam[3][1], PID_POSITION);
    PID_InitPIDParam(&BalcData.L_Spdpidparam, Const_LeftSpdMotorParam[0][0], Const_LeftSpdMotorParam[0][1], Const_LeftSpdMotorParam[0][2], Const_LeftSpdMotorParam[0][3], Const_LeftSpdMotorParam[0][4],
                    Const_LeftSpdMotorParam[1][0], Const_LeftSpdMotorParam[1][1], Const_LeftSpdMotorParam[2][0], Const_LeftSpdMotorParam[2][1],
                    Const_LeftSpdMotorParam[3][0], Const_LeftSpdMotorParam[3][1], PID_POSITION);
	  PID_InitPIDParam(&BalcData.R_Pospidparam, Const_RightPosMotorParam[0][0], Const_RightPosMotorParam[0][1], Const_RightPosMotorParam[0][2], Const_RightPosMotorParam[0][3], Const_RightPosMotorParam[0][4],
                    Const_RightPosMotorParam[1][0], Const_RightPosMotorParam[1][1], Const_RightPosMotorParam[2][0], Const_RightPosMotorParam[2][1],
                    Const_RightPosMotorParam[3][0],Const_RightPosMotorParam[3][1],PID_POSITION);
    PID_InitPIDParam(&BalcData.R_Spdpidparam, Const_RightSpdMotorParam[0][0], Const_RightSpdMotorParam[0][1], Const_RightSpdMotorParam[0][2], Const_RightSpdMotorParam[0][3], Const_RightSpdMotorParam[0][4],
                    Const_RightSpdMotorParam[1][0], Const_RightSpdMotorParam[1][1], Const_RightSpdMotorParam[2][0], Const_RightSpdMotorParam[2][1],
                    Const_RightSpdMotorParam[3][0], Const_RightSpdMotorParam[3][1], PID_POSITION);
	//PID_InitPIDCrossParam(&BalcData.crosspid, Const_CrossPIDParam[0], Const_CrossPIDParam[1] );
    BalcData.motorGroup = Motor_groupHandle[3];
	BalcData.GravMulLen = 0.0f;
    BalcData.mode = Balc_WaitMode;
	BalcData.listen_mode = Balc_OffListening;
	BalcData.ready = Balc_NotReady;
	BalcData.init = 0;
	BalcData.end = 0;
    BalcState.Pole_Speed_ref = 0.0f;
    BalcState.Pole_Angle_ref = 0.0f;
    BalcState.Pole_Force_ref = 0.0f;
    BalcState.Pole_Angle_fdb = 0.0f;
    BalcState.Pole_Speed_fdb = 0.0f;
    BalcState.Pole_Force_fdb = 0.0f;
	BalcState.Pole_Speed_Y = 0.0f;

}

void Balc_IsAngle(float L_angle_ref, float R_angle_ref)
{
    Balc_DataTypeDef *data = Balc_GetDataPtr();
    Balc_StateTypeDef *state = Balc_GetStatePtr();

    if(fabs(data->motorGroup->motor_handle[0]->encoder.cousequence_angle_reduction - L_angle_ref) < 5.0f)
    {
      	data->motorGroup->motor_handle[0]->Is_Angle = 1;
    }
	else
	{
		data->motorGroup->motor_handle[0]->Is_Angle = 0;
	}

	if(fabs(data->motorGroup->motor_handle[1]->encoder.cousequence_angle_reduction - R_angle_ref) < 5.0f)
	{
		data->motorGroup->motor_handle[1]->Is_Angle = 1;
	}
	else
	{
		data->motorGroup->motor_handle[1]->Is_Angle = 0;
	}
		
}


void Balc_SetPos(float angle, float limit_speed)
{
	Balc_DataTypeDef *data = Balc_GetDataPtr();
	Balc_StateTypeDef *state = Balc_GetStatePtr();
	state->Pole_Angle_ref = angle;
	state->Pole_Speed_ref = limit_speed;
	Balc_BattingEncode();
	Balc_CalAverGarvMulLen();

	if(limit_speed != 0.0f)
	{
		if(data->motorGroup->motor_handle[0]->Is_Angle == 0 && data->motorGroup->motor_handle[1]->Is_Angle == 0)
		{
			
			PID_SetPIDRef(&data->L_Spdpid, state->Lspeed_ref);
			PID_SetPIDFdb(&data->L_Spdpid, data->motorGroup->motor_handle[0]->encoder.consequence_speed);
			PID_CalcPID(&data->L_Spdpid, &data->L_Spdpidparam);

			PID_SetPIDRef(&data->R_Spdpid, state->Rspeed_ref);
			PID_SetPIDFdb(&data->R_Spdpid, data->motorGroup->motor_handle[1]->encoder.consequence_speed);
			PID_CalcPID(&data->R_Spdpid, &data->R_Spdpidparam);

			Motor_SetMotorOutput(data->motorGroup->motor_handle[0], data->L_Spdpid.output + data->GravMulLen / 0.3f / 20.0f * 16384.0f / 2.0f);
			Motor_SetMotorOutput(data->motorGroup->motor_handle[1], data->R_Spdpid.output - data->GravMulLen / 0.3f / 20.0f * 16384.0f / 2.0f);
		
			Motor_SendMotorGroupOutput(data->motorGroup);
		}
			
		else if(data->motorGroup->motor_handle[0]->Is_Angle == 1 || data->motorGroup->motor_handle[1]->Is_Angle == 1)
		{
			PID_SetPIDRef(&data->L_Pospid, state->Ltheta_ref);
			PID_SetPIDFdb(&data->L_Pospid, data->motorGroup->motor_handle[0]->encoder.cousequence_angle_reduction);
			PID_CalcPID(&data->L_Pospid, &data->L_Pospidparam);
			
			PID_SetPIDRef(&data->L_Spdpid, data->L_Pospid.output);
			PID_SetPIDFdb(&data->L_Spdpid, data->motorGroup->motor_handle[0]->encoder.consequence_speed);
			PID_CalcPID(&data->L_Spdpid, &data->L_Spdpidparam);
		
			PID_SetPIDRef(&data->R_Pospid, state->Rtheta_ref);
			PID_SetPIDFdb(&data->R_Pospid, data->motorGroup->motor_handle[1]->encoder.cousequence_angle_reduction);
			PID_CalcPID(&data->R_Pospid, &data->R_Pospidparam);
			
			PID_SetPIDRef(&data->R_Spdpid, data->R_Pospid.output);
			PID_SetPIDFdb(&data->R_Spdpid, data->motorGroup->motor_handle[1]->encoder.consequence_speed);
			PID_CalcPID(&data->R_Spdpid, &data->R_Spdpidparam);

			Motor_SetMotorOutput(data->motorGroup->motor_handle[0], data->L_Spdpid.output + data->GravMulLen / 0.3f / 20.0f * 16384.0f / 2.0f);
			Motor_SetMotorOutput(data->motorGroup->motor_handle[1], data->R_Spdpid.output - data->GravMulLen / 0.3f / 20.0f * 16384.0f / 2.0f);
			
			Motor_SendMotorGroupOutput(data->motorGroup);
		}
	}
	else
	{
		PID_SetPIDRef(&data->L_Pospid, state->Ltheta_ref);
		PID_SetPIDFdb(&data->L_Pospid, data->motorGroup->motor_handle[0]->encoder.cousequence_angle_reduction);
		PID_CalcPID(&data->L_Pospid, &data->L_Pospidparam);
		
		PID_SetPIDRef(&data->L_Spdpid, data->L_Pospid.output);
		PID_SetPIDFdb(&data->L_Spdpid, data->motorGroup->motor_handle[0]->encoder.consequence_speed);
		PID_CalcPID(&data->L_Spdpid, &data->L_Spdpidparam);
	
		PID_SetPIDRef(&data->R_Pospid, state->Rtheta_ref);
		PID_SetPIDFdb(&data->R_Pospid, data->motorGroup->motor_handle[1]->encoder.cousequence_angle_reduction);
		PID_CalcPID(&data->R_Pospid, &data->R_Pospidparam);
		
		PID_SetPIDRef(&data->R_Spdpid, data->R_Pospid.output);
		PID_SetPIDFdb(&data->R_Spdpid, data->motorGroup->motor_handle[1]->encoder.consequence_speed);
		PID_CalcPID(&data->R_Spdpid, &data->R_Spdpidparam);

		Motor_SetMotorOutput(data->motorGroup->motor_handle[0], data->L_Spdpid.output + data->GravMulLen / 0.3f / 20.0f * 16384.0f / 2.0f);
		Motor_SetMotorOutput(data->motorGroup->motor_handle[1], data->R_Spdpid.output - data->GravMulLen / 0.3f / 20.0f * 16384.0f / 2.0f);
		
		Motor_SendMotorGroupOutput(data->motorGroup);
	}
		
	Balc_IsAngle(state->Ltheta_ref, state->Rtheta_ref);
}

void Balc_CalAverGarvMulLen(void)
{
	Balc_DataTypeDef *data = Balc_GetDataPtr();
	Balc_StateTypeDef *state = Balc_GetStatePtr();
	
	data->GravMulLen = 1.10 * sin(Math_Angle2Rad((data->motorGroup->motor_handle[0]->encoder.cousequence_angle_reduction)));
}


void Balc_BattingEncode(void)
{
    Balc_StateTypeDef *state = Balc_GetStatePtr();
    Balc_DataTypeDef *data = Balc_GetDataPtr();

    state->Lspeed_ref = state->Pole_Speed_ref;
    state->Rspeed_ref = - state->Pole_Speed_ref;

    state->Ltheta_ref = state->Pole_Angle_ref;
    state->Rtheta_ref =  -state->Pole_Angle_ref;

    state->L_ref = state->Pole_Force_ref / Curr2ForceConstant_3508 * Fact2Ctrl_3508;
    state->R_ref =  - state->Pole_Force_ref / Curr2ForceConstant_3508 * Fact2Ctrl_3508;
 
    state->Pole_Angle_fdb = data->motorGroup->motor_handle[1]->encoder.cousequence_angle_reduction;

    state->Pole_Speed_fdb = data->motorGroup->motor_handle[1]->encoder.consequence_speed;

    state->Pole_Force_fdb = data->motorGroup->motor_handle[1]->encoder.current * Fact2Ctrl_3508;
} 

Balc_ReadyMode Balc_GetReady(void)
{
	Balc_DataTypeDef *data = Balc_GetDataPtr();
	return data->ready;
}


Balc_DataTypeDef* Balc_GetDataPtr(void)
{
    return &BalcData;
}
Balc_StateTypeDef* Balc_GetStatePtr(void)   
{
    return &BalcState;
}

void Balc_BattingReady(void)
{
	Balc_DataTypeDef *data = Balc_GetDataPtr();
 	Balc_StateTypeDef *state = Balc_GetStatePtr();
	
		if(data->ready == Balc_NotReady)
		{
			Balc_SetPos(180.0f, 0.5f);
			if(data->motorGroup->motor_handle[0]->Is_Angle == 1 && data->motorGroup->motor_handle[1]->Is_Angle == 1)
			{
				data->ready = Balc_Ready;
			}
		}
}


void Balc_StartingBatting(void)
{
	DWT_DataTypeDef *dwt = DWT_GetDWTDataPtr();
	Balc_DataTypeDef *data = Balc_GetDataPtr();
  	Balc_StateTypeDef *state = Balc_GetStatePtr();
	
	Balc_BattingReady();
				
	if(Balc_GetReady() == Balc_Ready && data->end == 0)
	{
		state->Pole_Angle_ref = 180.0f;
		state->Pole_Speed_ref = 6.0f;
		Balc_BattingEncode();
		Balc_CalAverGarvMulLen();
		
		PID_SetPIDRef(&data->L_Spdpid, state->Lspeed_ref);
		PID_SetPIDFdb(&data->L_Spdpid, data->motorGroup->motor_handle[0]->encoder.consequence_speed);
		PID_CalcPID(&data->L_Spdpid, &data->L_Spdpidparam);

		PID_SetPIDRef(&data->R_Spdpid, state->Rspeed_ref);
		PID_SetPIDFdb(&data->R_Spdpid, data->motorGroup->motor_handle[1]->encoder.consequence_speed);
		PID_CalcPID(&data->R_Spdpid, &data->R_Spdpidparam);

		Motor_SetMotorOutput(data->motorGroup->motor_handle[0], data->L_Spdpid.output + data->GravMulLen / 0.3f / 20.0f * 16384.0f / 2.0f);
		Motor_SetMotorOutput(data->motorGroup->motor_handle[1], data->R_Spdpid.output - data->GravMulLen / 0.3f / 20.0f * 16384.0f / 2.0f);
	
		Motor_SendMotorGroupOutput(data->motorGroup);
		if(data->motorGroup->motor_handle[0]->encoder.cousequence_angle_reduction > 135.0f)
		{
			data->end = 1;
			data->mode = Balc_WaitMode;
		}
	
	}
}

void Balc_Wait(void)
{
		DWT_DataTypeDef *dwt = DWT_GetDWTDataPtr();
		Balc_DataTypeDef *data = Balc_GetDataPtr();
		Balc_StateTypeDef *state = Balc_GetStatePtr();
	
		state->Pole_Angle_ref = 0.0f;
		state->Pole_Speed_ref = 0.0f;
		Balc_BattingEncode();
		Balc_CalAverGarvMulLen();
		
		PID_SetPIDRef(&data->L_Pospid, state->Ltheta_ref);
		PID_SetPIDFdb(&data->L_Pospid, data->motorGroup->motor_handle[0]->encoder.cousequence_angle_reduction);
		PID_CalcPID(&data->L_Pospid, &data->L_Pospidparam);
		
		PID_SetPIDRef(&data->L_Spdpid, data->L_Pospid.output);
		PID_SetPIDFdb(&data->L_Spdpid, data->motorGroup->motor_handle[0]->encoder.consequence_speed);
		PID_CalcPID(&data->L_Spdpid, &data->L_Spdpidparam);
	
		PID_SetPIDRef(&data->R_Pospid, state->Rtheta_ref);
		PID_SetPIDFdb(&data->R_Pospid, data->motorGroup->motor_handle[1]->encoder.cousequence_angle_reduction);
		PID_CalcPID(&data->R_Pospid, &data->R_Pospidparam);
		
		PID_SetPIDRef(&data->R_Spdpid, data->R_Pospid.output);
		PID_SetPIDFdb(&data->R_Spdpid, data->motorGroup->motor_handle[1]->encoder.consequence_speed);
		PID_CalcPID(&data->R_Spdpid, &data->R_Spdpidparam);

		Motor_SetMotorOutput(data->motorGroup->motor_handle[0], data->L_Spdpid.output + data->GravMulLen / 0.3f / 20.0f * 16384.0f / 2.0f);
		Motor_SetMotorOutput(data->motorGroup->motor_handle[1], data->R_Spdpid.output - data->GravMulLen / 0.3f / 20.0f * 16384.0f / 2.0f);
	
		Motor_SendMotorGroupOutput(data->motorGroup);
	
}