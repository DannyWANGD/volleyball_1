#ifndef __APP_GIMBAL_H
#define __APP_GIMBAL_H

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "periph_motor.h"
#include "alg_pid.h"

typedef enum
{
    Gimbal_OffListening  = 0,
    Gimbal_Listening     = 1

} Gimbal_ListenModeEnum;




typedef struct
{
    float Pole_Yaw_ref;
    float Pole_Pitch_ref;

    float Yaw_fdb;
    float Pitch_fdb;
	
	Motor_MotorGroupTypeDef * pitch_motorGroup;
	Motor_MotorGroupTypeDef * yaw_motorGroup;
	uint8_t init;
    Gimbal_ListenModeEnum listen_mode;
	float update_dt;
	uint32_t last_update_time;
}Gimbal_DataTypeDef;

void Gimbal_Init(void);
void Gimbal_SetPos(void);
void Gimbal_SendOutput(void);
Gimbal_DataTypeDef* Gimbal_GetDataPtr(void);



#endif
