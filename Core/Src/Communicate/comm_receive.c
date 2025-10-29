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
#include "periph_remote.h"
#include "module_platform.h"
#include "app_gimbal.h"
#include "module_chassis.h"

// Platform data receive function
// 参数: buff 接收到的数据缓冲区指针
// 返回值: uint32_t 实际处理的数据长度
static uint32_t _set_Platform_Data_(uint8_t *buff); 
//static uint32_t _set_Gimbal_Data_(uint8_t *buff); // Gimbal data receive function
static uint32_t _set_Chassis_Data_(uint8_t *buff); // Chassis data receive function

// Communication receive function table
// 每一个通信指令对应一个处理函数
// CommCmd_Receive 数组中的每一个元素都是一个 Comm_ReceiveEntry 结构体
// 该结构体包含一个函数指针 bus_func，指向具体的处理函数
Comm_ReceiveEntry CommCmd_Receive[Const_Comm_Receive_BUFF_SIZE] = {
  {&_set_Platform_Data_},
  {&_set_Chassis_Data_}
};


static uint32_t _set_Platform_Data_(uint8_t *buff) {
	
  Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr(); 
  Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();

  if (Platform->ctrl_mode == Platform_Jiefa) {
	  Platform_Jiefa_Cal(buff2float(buff));
  }
  if (Platform->ctrl_mode == Platform_Dianqiu) {
	  Platform_Dianqiu_Cal(buff2float(buff), rc->remote.s[1]);
  }

  return 4; // 这个返回值用于告诉调用者实际处理了多少数据,方便后续的数据处理和管理

}

// static uint32_t _set_Gimbal_Data_(uint8_t *buff) {
	
// 	Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr(); 
// 	Gimbal_DataTypeDef *gimbal = Gimbal_GetDataPtr();
	
// 	gimbal->Pole_Yaw_ref = buff2float(buff);
// 	gimbal->Pole_Pitch_ref = buff2float(buff + 4);

// 	Gimbal_SetPos();
	
// 	return 8;
// }

static uint32_t _set_Chassis_Data_(uint8_t *buff)
{
	Chassis_DataTypeDef *Chassis = Chassis_GetChassisPtr();
  Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr(); 
	
  if (Chassis->Chassis_CtrlMode == Chassis_PC) {

    float x =  buff2float(buff); // 从 buff 前 4 字节解出 float，为 x 方向速度或目标
    float y =  buff2float(buff + 4); // 从 buff 第 4 到 8 字节解出 float，为 y 方向速度或目标
    float w =  buff2float(buff + 8); // 从 buff 第 8 到 12 字节解出 float，为角速度或目标
    int state = (int)buff2float(buff + 12); // 从 buff 第 12 到 16 字节解出 float，为控制状态，强转为 int 
    LimitMaxMin(x,3.6,-3.6);
    LimitMaxMin(y,3.6,-3.6);
    LimitMaxMin(w,0.6,-0.6);
    if( state == 0 ) { // speed
      Chassis_Set_Speed(Chassis_Run,x,y,w); 
    } else if(state == 1) { //position
      Chassis_Set_Speed(Chassis_Stop,0,0,0); 
    } else if(state == 2) { //position
      Chassis_Set_Speed(Chassis_Lock,0,0,0); 
    }
	}
  return 16;        
}