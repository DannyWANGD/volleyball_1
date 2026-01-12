#ifndef __APP_GIMBAL_H
#define __APP_GIMBAL_H

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "periph_motor.h"
#include "alg_pid.h"

typedef enum
{
	Gimbal_Off = 0,
	Gimbal_remote = 1,
	Gimbal_auto = 2
} Gimbal_State;

typedef struct
{
	Motor_MotorGroupTypeDef * motorGroup;


    float Yaw_ref;
    float Pitch_ref;
	
	float Yaw_fdb;
    float Pitch_fdb;

    float Pole_Yaw_ref;
    float Pole_Pitch_ref;

    float Pole_Yaw_fdb;
    float Pole_Pitch_fdb;
	
	  PID_PIDTypeDef Yaw_Pospid;
    PID_PIDParamTypeDef Yaw_Pospidparam;

    PID_PIDTypeDef Pitch_Pospid;
    PID_PIDParamTypeDef Pitch_Pospidparam;

    PID_PIDTypeDef Yaw_Spdpid;
    PID_PIDParamTypeDef Yaw_Spdpidparam;

    PID_PIDTypeDef Pitch_Spdpid;
    PID_PIDParamTypeDef Pitch_Spdpidparam;
	
	uint8_t init;
	Gimbal_State gimbal_state;
	float update_dt;
	uint32_t last_update_time;
}Gimbal_TypeDef;

void Gimbal_Init(void);
void Gimbal_SetPos(void);
void Gimbal_PosLimit(void);
void Gimbal_PIDCalc(void);
void Gimbal_StateSet(Gimbal_State state);
Gimbal_TypeDef* Gimbal_GetPtr(void);

#endif
