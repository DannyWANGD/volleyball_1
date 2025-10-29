#ifndef __APP_SERVE_H__
#define __APP_SERVE_H__

#include "alg_pid.h"
#include "FreeRTOS.h"
#include "main.h"
#include "periph_motor.h"
#include <stdio.h>


#define Balc_GravMulLen_Average   0.001
#define Balc_Angle                60

typedef enum 
{
    Balc_WaitMode         = 0,
    Balc_HitMode          = 1,
		Balc_DisConnectedMode = 2,

} Balc_ModeEnum;

typedef enum
{
    Balc_OffListening  = 0,
    Balc_Listening     = 1

} Balc_ListenModeEnum;

typedef enum
{
    Balc_NotReady = 0,
    Balc_Ready    = 1

} Balc_ReadyMode;

typedef struct 
{
    float Ltheta_ref;
    float Rtheta_ref;

    float dLtheta;
    float dRtheta;

    float L_ref;
    float R_ref;

    float Lspeed_ref;
    float Rspeed_ref;

    float Pole_Angle_ref;
    float Pole_Speed_ref;
    float Pole_Force_ref;

    float Pole_Angle_fdb;
    float Pole_Speed_fdb;
    float Pole_Force_fdb;
	
	float Pole_Speed_Y;

} Balc_StateTypeDef;

typedef struct 
{
    Motor_MotorGroupTypeDef * motorGroup;

    PID_PIDTypeDef L_Pospid;
    PID_PIDParamTypeDef L_Pospidparam;

    PID_PIDTypeDef R_Pospid;
    PID_PIDParamTypeDef R_Pospidparam;

    PID_PIDTypeDef L_Spdpid;
    PID_PIDParamTypeDef L_Spdpidparam;

    PID_PIDTypeDef R_Spdpid;
    PID_PIDParamTypeDef R_Spdpidparam;


    Balc_ModeEnum mode;
    Balc_ListenModeEnum listen_mode;
	uint8_t init;
	uint8_t end;
	Balc_ReadyMode ready;

    float update_dt;
    uint32_t last_update_time;

    float GravMulLen;

} Balc_DataTypeDef;

extern Balc_DataTypeDef  BalcData;
extern Balc_StateTypeDef BalcState;

void speed_calculate(float y);
void Serve_Task(void const * argument);
void Balc_Init(void);
Balc_DataTypeDef* Balc_GetDataPtr(void);
Balc_StateTypeDef* Balc_GetStatePtr(void);
void Balc_SetMode(Balc_ModeEnum mode);
void Balc_SetListenMode(Balc_ListenModeEnum mode);
void Balc_SetReady(Balc_ReadyMode ready);
void Balc_SetThetaRef(float Ltheta_ref, float Rtheta_ref);
void Balc_SetRef(float L_ref, float R_ref);
void Balc_SetSpeedRef(float Lspeed_ref, float Rspeed_ref);
Balc_ReadyMode Balc_GetReady(void);
Balc_ModeEnum Balc_GetMode(void);
Balc_ListenModeEnum Balc_GetListenMode(void);
void Balc_Control(void);
void Balc_TestforSpeed(float velocity);
void Balc_TestforAngle(void);
void Balc_TestforForce(void);
void Balc_TestforHit(float velocity);
float Balc_CalMonForce(float f, float len, float ang);
void Balc_CalAverGarvMulLen(void);
void Balc_BattingEncode(void);
void Balc_SharpBreak(void);
void Balc_SetPos(float angle, float speed);
void Balc_IsAngle(float L_angle_ref, float R_angle_ref);
void Balc_IsReady(void);
void Balc_SetSpdPosRef(float Pole_Pos, float Pole_Spd);
void Balc_BattingReady(void);
void Balc_BattingResponseReady(void);
void Balc_Batting(float angle, float limit_speed);
void Balc_StartingBatting(void);
void Balc_Wait(void);


#endif