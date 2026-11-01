/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : comm_receive.c
 *  Description  : This file is for receive communication
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:10:30
 *  LastEditTime : 2023-10-04 00:47:42
 */


#include "comm_common.h"
#include "comm_receive.h"

#include "stdlib.h"
#include "math.h"
#include "lib_buff.h"
#include "module_platform.h"
#include "app_gimbal.h"
#include "module_chassis.h"
#include "sys_const.h"
#include "sys_dwt.h"

// Platform data receive function
// 参数: buff 接收到的数据缓冲区指针
// 返回值: uint32_t 实际处理的数据长度
static uint32_t _set_Platform_Data_(uint8_t *buff); 
static uint32_t _set_Platform_Data_1(uint8_t *buff); 
static uint32_t _set_Gimbal_Data_(uint8_t *buff);
static uint32_t _get_Chassis_Data_(uint8_t *buff); // Chassis data receive function
static uint32_t _set_Remote_Data_(uint8_t *buff);
Filter_WindowTypeDef avg_filter;
Remote_DataTypeDef Remote_Data;

float pitch;
float roll;

// Communication receive function table
// 每一个通信指令对应一个处理函数
// CommCmd_Receive 数组中的每一个元素都是一个 Comm_ReceiveEntry 结构体
// 该结构体包含一个函数指针 bus_func，指向具体的处理函数
Comm_ReceiveEntry CommCmd_Receive[Const_Comm_Receive_BUFF_SIZE] = {
  	{&_set_Remote_Data_    },
		{&_set_Gimbal_Data_},
		{&_set_Platform_Data_},
    {&_get_Chassis_Data_}
};

static uint32_t _get_Chassis_Data_(uint8_t *buff)
{
      Chassis_DataTypeDef *Chassis = Chassis_GetChassisPtr();
      Comm_DataTypeDef *buscomm = Comm_GetBusDataPtr();
      Chassis->vy = buff2float(&buscomm->UART_Buff[45]);
			Chassis->y = buff2float(&buscomm->UART_Buff[49]);
      Chassis->y_rx_time = DWT_GetTimeline_s();
      // Chassis->vx = buff2float(buff);
      return 8;
}
Remote_DataTypeDef *Remote_GetRemotePtr(void) { return &Remote_Data; }

static uint32_t _set_Remote_Data_(uint8_t *buff)
{
	
	Remote_DataTypeDef *rc = Remote_GetRemotePtr(); 
	Comm_DataTypeDef *buscomm = Comm_GetBusDataPtr();
	
	if (buscomm->UART_Buff[4] == 0x0F && buscomm->UART_Buff[28] == 0x00) {
    rc->connect = Remote_Connect;
  } else {
    rc->connect = Remote_DisConnect;
  }

  if (rc->connect == Remote_Connect) {
    rc->rawdata.ch[0] = (((uint16_t)buscomm->UART_Buff[5] | (uint16_t)buscomm->UART_Buff[6] << 8) & 0x07FF);
    rc->rawdata.ch[1] = (((uint16_t)buscomm->UART_Buff[6] >> 3 | (uint16_t)buscomm->UART_Buff[7] << 5) & 0x07FF);
    rc->rawdata.ch[2] = (((uint16_t)buscomm->UART_Buff[7] >> 6 | (uint16_t)buscomm->UART_Buff[8] << 2 | (uint16_t)buscomm->UART_Buff[9] << 10) & 0x07FF);
    rc->rawdata.ch[3] = (((uint16_t)buscomm->UART_Buff[9] >> 1 | (uint16_t)buscomm->UART_Buff[10] << 7) & 0x07FF);
    rc->rawdata.ch[4] = (((uint16_t)buscomm->UART_Buff[10] >> 4 | (uint16_t)buscomm->UART_Buff[11] << 4) & 0x07FF);
    rc->rawdata.ch[5] = (((uint16_t)buscomm->UART_Buff[11] >> 7 | (uint16_t)buscomm->UART_Buff[12] << 1 | (uint16_t)buscomm->UART_Buff[13] << 9) & 0x07FF);
    rc->rawdata.ch[6] = (((uint16_t)buscomm->UART_Buff[13] >> 2 | (uint16_t)buscomm->UART_Buff[14] << 6) & 0x07FF);
    rc->rawdata.ch[7] = (((uint16_t)buscomm->UART_Buff[14] >> 5 | (uint16_t)buscomm->UART_Buff[15] << 3) & 0x07FF);

    rc->rawdata.ch[8] = (((uint16_t)buscomm->UART_Buff[16] | (uint16_t)buscomm->UART_Buff[17] << 8) & 0x07FF);
    rc->rawdata.ch[9] = (((uint16_t)buscomm->UART_Buff[17] >> 3 | (uint16_t)buscomm->UART_Buff[18] << 5) & 0x07FF);
    rc->rawdata.ch[10] = (((uint16_t)buscomm->UART_Buff[18] >> 6 | (uint16_t)buscomm->UART_Buff[19] << 2 | (uint16_t)buscomm->UART_Buff[20] << 10) & 0x07FF);
    rc->rawdata.ch[11] = (((uint16_t)buscomm->UART_Buff[20] >> 1 | (uint16_t)buscomm->UART_Buff[21] << 7) & 0x07FF);
    rc->rawdata.ch[12] = (((uint16_t)buscomm->UART_Buff[21] >> 4 | (uint16_t)buscomm->UART_Buff[22] << 4) & 0x07FF);
    rc->rawdata.ch[13] = (((uint16_t)buscomm->UART_Buff[22] >> 7 | (uint16_t)buscomm->UART_Buff[23] << 1 | (uint16_t)buscomm->UART_Buff[24] << 9) & 0x07FF);
    rc->rawdata.ch[14] = (((uint16_t)buscomm->UART_Buff[24] >> 2 | (uint16_t)buscomm->UART_Buff[25] << 6) & 0x07FF);
    rc->rawdata.ch[15] = (((uint16_t)buscomm->UART_Buff[25] >> 5 | (uint16_t)buscomm->UART_Buff[26] << 3) & 0x07FF);

    rc->ch[0] = - Remote_RockerOffset(rc->rawdata.ch[0]);
    rc->ch[1] = Remote_RockerOffset(rc->rawdata.ch[1]);
    rc->ch[2] = Remote_RockerOffset(rc->rawdata.ch[2]);
    rc->ch[3] = - Remote_RockerOffset(rc->rawdata.ch[3]);

    rc->s[0] = Remote_LeverOffset(rc->rawdata.ch[4]);
    rc->s[1] = Remote_LeverOffset(rc->rawdata.ch[5]);
    rc->s[2] = Remote_LeverOffset(rc->rawdata.ch[6]);
    rc->s[3] = Remote_LeverOffset(rc->rawdata.ch[7]);
  }
	return 25;
}

int16_t Remote_RockerOffset(uint16_t data) {
  return data - Remote_RockerMiddle;
}

Remote_LeverMode Remote_LeverOffset(uint16_t data) {
  if (data == 192)	return Lever_up;
  else if (data == 992)	return Lever_middle;
  else if (data == 1792)	return Lever_down;
}

static uint32_t _set_Platform_Data_(uint8_t *buff) {
	Comm_DataTypeDef *buscomm = Comm_GetBusDataPtr();
  Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr(); 
  Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();

  if (Platform->ctrl_mode == Platform_Jiefa && hit_state == STATE_IDLE) {
	  Platform->flag = buff2float(&buscomm->UART_Buff[41]);
    float pc_cmd = buff2float(&buscomm->UART_Buff[41]);
    if (pc_cmd == 1.0f) {
			Platform->flag = 1.0f; // 如果上位机下达击打命令，置1
			
    }
  }


  return 4; // 这个返回值用于告诉调用者实际处理了多少数据,方便后续的数据处理和管理

}
//static uint32_t _set_Platform_Data_1(uint8_t *buff){

//  Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr(); 
//  Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();

//  pitch = buff2float(buff);
//  roll = buff2float(buff + 4);

//  return 8;

//}

static uint32_t _set_Gimbal_Data_(uint8_t *buff) 
{
	Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr(); 
	Gimbal_TypeDef *gimbal = Gimbal_GetPtr();
	Comm_DataTypeDef *buscomm = Comm_GetBusDataPtr();

	if (gimbal->gimbal_state == Gimbal_auto) {

	gimbal->Yaw_ref = buff2float(&buscomm->UART_Buff[29]);
	gimbal->Pitch_ref = buff2float(&buscomm->UART_Buff[33]);
	}
		Gimbal_PosLimit();
	
//	float mode = buff2float(&buscomm->UART_Buff[37]);

//	if (mode == 1.0f)
//	{
//			gimbal->gimbal_state = Gimbal_auto;
//	}
//	else if (mode == 0.0f)
//	{
//			gimbal->gimbal_state = Gimbal_remote;
//	}
//	Gimbal_PosLimit();
 
	return 12;
}

//static uint32_t _set_Chassis_Data_(uint8_t *buff)
//{
//	Chassis_DataTypeDef *Chassis = Chassis_GetChassisPtr();
//  Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr(); 
//	
//  if (Chassis->Chassis_CtrlMode == Chassis_PC) {

//    float x =  buff2float(buff); // 从 buff 前 4 字节解出 float，为 x 方向速度或目标
//    float y =  buff2float(buff + 4); // 从 buff 第 4 到 8 字节解出 float，为 y 方向速度或目标
//    float w =  buff2float(buff + 8); // 从 buff 第 8 到 12 字节解出 float，为角速度或目标
//    int state = (int)buff2float(buff + 12); // 从 buff 第 12 到 16 字节解出 float，为控制状态，强转为 int 
//    LimitMaxMin(x,3.6,-3.6);
//    LimitMaxMin(y,3.6,-3.6);
//    LimitMaxMin(w,0.6,-0.6);
//    if( state == 0 ) { // speed
//      Chassis_Set_Speed(Chassis_Run,x,y,w); 
//    } else if(state == 1) { //position
//      Chassis_Set_Speed(Chassis_Stop,0,0,0); 
//    } else if(state == 2) { //position
//      Chassis_Set_Speed(Chassis_Lock,0,0,0); 
//    }
//	}
//  return 16;        
//}

void init_average_filter(void) {
    for (int i = 0; i < MAX_LENGTH; i++) {
        avg_filter.val[i] = 0.0f;
    }
    avg_filter.sum = 0.0f;
}
