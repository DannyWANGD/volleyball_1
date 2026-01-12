/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : comm_transmit.c
 *  Description  : This file is for transmit communication
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:18:41
 *  LastEditTime : 2023-09-22 13:31:40
 */


#include "comm_common.h"
#include "comm_transmit.h"

#include "stdlib.h"
#include "sys_dwt.h"
#include "sys_const.h"
#include "lib_buff.h"
#include "periph_remote.h"

#include "module_platform.h"
#include "app_gimbal.h"

static uint32_t _send_Remote_data(uint8_t *buff);
static uint32_t _send_Platform_data(uint8_t *buff);
static uint32_t _send_Gimbal_data(uint8_t *buff);
static uint32_t _send_RUNTick_data(uint8_t *buff);


Comm_SendEntry CommCmd_Send[Const_Comm_Transmit_BUFF_SIZE] = {
    { _send_Gimbal_data },
	//{ _send_Remote_data },
    //{ _send_Platform_data },
};

static uint32_t _send_Remote_data(uint8_t *buff) {

    Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr();

    buff[0] = (uint8_t)(rc->remote.s[0]) | ((uint8_t)(rc->remote.s[1]) << 4);
    buff[1] = (uint8_t)(rc->remote.ch[0] >> 8);
    buff[2] = (uint8_t)(rc->remote.ch[0]);
    buff[3] = (uint8_t)(rc->remote.ch[1] >> 8);
    buff[4] = (uint8_t)(rc->remote.ch[1]);
    buff[5] = (uint8_t)(rc->remote.ch[2] >> 8);
    buff[6] = (uint8_t)(rc->remote.ch[2]);
    buff[7] = (uint8_t)(rc->remote.ch[3] >> 8);
    buff[8] = (uint8_t)(rc->remote.ch[3]);
    buff[9] = (uint8_t)(rc->remote.ch[4] >> 8);
    buff[10] =(uint8_t)(rc->remote.ch[4]);

    return 11;
}

static uint32_t _send_Platform_data(uint8_t *buff) {
    
    Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
    float2buff( Platform->fdb.front_pitch_angle, buff);
    float2buff( Platform->fdb.left_pitch_angle, buff+4);
    float2buff( Platform->fdb.right_pitch_angle, buff+8);
    float2buff( Platform->fdb.front_yaw_angle, buff+12);
    float2buff( Platform->fdb.left_yaw_angle, buff+16);
    float2buff( Platform->fdb.right_yaw_angle, buff+20);
    return 24;
}

static uint32_t _send_Gimbal_data(uint8_t *buff) {
	
	Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
    
	Gimbal_TypeDef *gimbal = Gimbal_GetPtr();

    float2buff( gimbal->Yaw_fdb, buff);
	
    float2buff( gimbal->Pitch_fdb, buff + 4);
	
	if((data->remote.s[0] == Remote_SWITCH_UP || data->remote.s[0] == Remote_SWITCH_MIDDLE) && data->remote.s[1] == Remote_SWITCH_MIDDLE)
	{
		float2buff(1.0f, buff + 8);
	}

	else
	{
		float2buff(0.0f, buff + 8);
	}
	return 12;
}

static uint32_t _send_RUNTick_data(uint8_t *buff) {

    DWT_DataTypeDef* dwt = DWT_GetDWTDataPtr();
    float2buff(dwt->SysTime.ms_tick, buff);
    return 4;
}

