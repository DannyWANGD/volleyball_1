#ifndef MODULE_CHASSIS_H
#define MODULE_CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "periph_motor.h"
#include "alg_pid.h"

// 车体状态枚举类型，定义了车体的三种工作状态：运行、锁定和停止
typedef enum
{
	Chassis_Run = 1,
  Chassis_Lock  = 2,
	Chassis_Stop = 3,
} Chassis_StateEnum;

// 车体控制模式枚举类型，定义了车体的三种控制模式：遥控、PC 控制和定位控制
typedef enum
{
	Chassis_Remote = 1,
  Chassis_PC  = 2,
	Chassis_Location = 3,
} Chassis_CtrlModeEnum;

// 车体 PID 控制器结构体，包含用于位置控制的距离 PID 和角度 PID 以及它们的参数
typedef struct
{
  PID_PIDTypeDef Location_Distance_PID;// 位置控制 PID
  PID_PIDTypeDef Location_Anglew_PID;// 角度控制 PID
	
  PID_PIDParamTypeDef  Location_Distance_PIDParam;
	PID_PIDParamTypeDef  Location_Anglew_PIDParam;
} Chassis_PIDTypeDef;

// 车体控制数据结构体，用于车体的运动控制和数据传输
// 包含车体的状态、控制模式、PID 控制器以及速度等信息，以及 CAN 通信所需的头信息和句柄
// 例如，可以通过修改 vx、vy、vw 来控制车体的运动速度
// 通过 Chassis_State 和 Chassis_CtrlMode 来设置车体的工作状态和控制模式
// PID 控制器用于实现精确的运动控制
// CAN 相关成员用于与其他模块进行通信
typedef struct {

  Chassis_StateEnum  Chassis_State;
  Chassis_CtrlModeEnum Chassis_CtrlMode;

  Chassis_PIDTypeDef pid;
  float vx;
  float vy;
  float vw;
	float y;

  float update_dt;
  uint32_t last_update_tick;
  float y_rx_time;// 上一刻接受底盘数据的时间

  CAN_TxHeaderTypeDef can_header;
  CAN_HandleTypeDef* can_handle;
  
} Chassis_DataTypeDef;

Chassis_DataTypeDef* Chassis_GetChassisPtr(void);
void	Chassis_Control();
void Chassis_Trans_ControlData();
void Chassis_Init();
uint16_t float_to_2bytes(float value);
void Chassis_SetControlMode(uint8_t mode);
void Chassis_Set_Speed(uint8_t state,float x,float y,float w);
#endif

#ifdef __cplusplus
}
#endif
