#include "module_chassis.h"
#include "app_chassis.h"
#include "util_can.h"
#include "sys_dwt.h"
#include "sys_const.h"
#include "cmsis_os.h"
#include "periph_dt35.h"
#include "alg_math.h" 

Chassis_DataTypeDef Chassis_Data;// 车体控制数据结构体实例

Chassis_DataTypeDef* Chassis_GetChassisPtr() {
    return &Chassis_Data;
}

void Chassis_Init() {

  Chassis_DataTypeDef *Chassis = Chassis_GetChassisPtr();
    
  Chassis->Chassis_CtrlMode = Chassis_Remote;
  Chassis->Chassis_State = Chassis_Stop;
      
  PID_InitPIDParam(&Chassis->pid.Location_Distance_PIDParam,Const_Chassis_Location_Distance_Param[0][0],Const_Chassis_Location_Distance_Param[0][1],Const_Chassis_Location_Distance_Param[0][2],Const_Chassis_Location_Distance_Param[0][3],Const_Chassis_Location_Distance_Param[0][4],
                                                            Const_Chassis_Location_Distance_Param[1][0],Const_Chassis_Location_Distance_Param[1][1],
                                                            Const_Chassis_Location_Distance_Param[2][0],Const_Chassis_Location_Distance_Param[2][1],
                                                            Const_Chassis_Location_Distance_Param[3][0],Const_Chassis_Location_Distance_Param[3][1],PID_POSITION);
  PID_InitPIDParam(&Chassis->pid.Location_Anglew_PIDParam,Const_Chassis_Location_Anglew_Param[0][0],Const_Chassis_Location_Anglew_Param[0][1],Const_Chassis_Location_Anglew_Param[0][2],Const_Chassis_Location_Anglew_Param[0][3],Const_Chassis_Location_Anglew_Param[0][4],
                                                          Const_Chassis_Location_Anglew_Param[1][0],Const_Chassis_Location_Anglew_Param[1][1],
                                                          Const_Chassis_Location_Anglew_Param[2][0],Const_Chassis_Location_Anglew_Param[2][1],
                                                          Const_Chassis_Location_Anglew_Param[3][0],Const_Chassis_Location_Anglew_Param[3][1],PID_POSITION);
  Chassis->vx = 0;
  Chassis->vy = 0;
  Chassis->vw = 0;
  
  Chassis->can_handle = &hcan2; // CAN2

  Can_InitTxHeader(&(Chassis->can_header), 0x07, 0, 8); // 初始化一个CAN发送数据头(车体控制数据 ID 0x07)
    
  Chassis->update_dt = 0;
  Chassis->last_update_tick = DWT_GetTimeline_us();
}

/**
 * @brief  打包底盘控制数据并通过CAN总线发送。
 * @details 该函数将底盘的状态和目标速度 (vx, vy, vw) 压缩为自定义的
 * 16位浮点格式 (float_to_2bytes)，并组装成一个8字节的CAN帧发送出去。
 * 同时，它会更新两次调用之间的时间差 (dt)。
 * @param  无
 * @return 无
 * @note   CAN数据帧格式 (8字节):
 * 
 * - [0]: Chassis_State (底盘状态)、[1]: vx 高8位、[2]: vx 低8位、[3]: vy 高8位
 * 
 * - [4]: vy 低8位、[5]: vw 高8位、[6]: vw 低8位、[7]: 0x05 (固定值)
 */
void Chassis_Trans_ControlData(){
  Chassis_DataTypeDef *Chassis = Chassis_GetChassisPtr(); 
	Chassis->update_dt = DWT_GetDeltaT(&Chassis->last_update_tick);
	uint8_t txbuff[8];
	
  txbuff[0] = Chassis->Chassis_State;
  txbuff[1] = (float_to_2bytes(Chassis->vx))>>8;
  txbuff[2] =  float_to_2bytes(Chassis->vx);
  txbuff[3] = (float_to_2bytes(Chassis->vy))>>8;
  txbuff[4] = float_to_2bytes(Chassis->vy);
  txbuff[5] = (float_to_2bytes(Chassis->vw))>>8;
  txbuff[6] = float_to_2bytes(Chassis->vw);
  txbuff[7] = 5;
  Can_SendMessage(Chassis->can_handle, &(Chassis->can_header), txbuff);
}

/**
 * @brief  设置底盘的目标速度和状态。
 * @details 该函数用于更新底盘控制器的状态及期望的三个速度分量。
 * @param  state  底盘的目标状态 (例如：CHASSIS_STOP, CHASSIS_RUNNING 等)。
 * @param  x      X 轴方向的目标速度 (通常为前进/后退)，单位 m/s。
 * @param  y      Y 轴方向的目标速度 (通常为左/右平移)，单位 m/s。
 * @param  w      绕 Z 轴的旋转角速度 (自旋)，单位 rad/s。
 * @return 无
 * @note   函数内部会对输入速度 x, y, w 进行限幅，范围为 [-7.0, +7.0]。
 * @note   请注意：内部存储的角速度 Chassis->vw 被设置为输入 w 的相反数 (-w)。
 */
void Chassis_Set_Speed(uint8_t state,float x,float y,float w){

  Chassis_DataTypeDef *Chassis = Chassis_GetChassisPtr();
	
	Chassis->Chassis_State = state;
  LimitMaxMin(x,7,-7);
	LimitMaxMin(y,7,-7);
	LimitMaxMin(w,7,-7);
	
	Chassis->vx = x;
	Chassis->vy = y;
	Chassis->vw = -w;	

}

// 设置底盘控制模式
void Chassis_SetControlMode(uint8_t mode) {
  Chassis_DataTypeDef *Chassis = Chassis_GetChassisPtr();
  Chassis->Chassis_CtrlMode = mode;
}

/**
 * @brief  执行一个基于PID的定位控制步骤 (后退或保持距离)。
 * @details 使用 DT35 传感器作为反馈，通过两个PID控制器（一个用于距离，一个用于角度）
 * 来计算底盘的平移速度 `vy` 和旋转速度 `vw`，以达到目标距离和角度。
 * @param  distance  目标距离 (单位与 DT35->distance 一致, e.g., mm)。
 * @param  w_angle   目标角度 (单位与 DT35->w_angle 一致, e.g., 度)。
 * @return 无
 * @note
 * - 包含PID死区处理：距离误差小于3，角度误差小于1时，PID输出为0。  
 * 
 * - 仅当 DT35 传感器状态 `state == 1` (有效) 时才设置速度。
 * 
 * - PID输出的 `vy` 被取反后 (-vy) 作为Y轴速度输入 `Chassis_Set_Speed`。
 */
void Chassis_Stepback(float distance,float w_angle){

  float vy,vw;

  Chassis_DataTypeDef *Chassis = Chassis_GetChassisPtr();	
  DT35_DataTypeDef *DT35 = &DT35_Data; 
	
  // 把目标值 distance 写入 距离 PID 的参考输入
  PID_SetPIDRef(&Chassis->pid.Location_Distance_PID,distance);
  // 把当前测量 DT35->distance 写入距离 PID 的反馈
  PID_SetPIDFdb(&Chassis->pid.Location_Distance_PID, DT35->distance);

	if (fabsf(distance-DT35->distance) >= 3) {
    // 先计算 PID 输出，再取出 PID 输出值作为纵向速度 vy
    PID_CalcPID(&Chassis->pid.Location_Distance_PID, &Chassis->pid.Location_Distance_PIDParam);
    vy = PID_GetPIDOutput(&Chassis->pid.Location_Distance_PID);
  }  else {
    vy=0;
  }

  //角度周期再来一遍
  PID_SetPIDRef(&Chassis->pid.Location_Anglew_PID,  w_angle);
  PID_SetPIDFdb(&Chassis->pid.Location_Anglew_PID, DT35->w_angle);
	if (fabsf(w_angle-DT35->w_angle) >= 1) {
    PID_CalcPID(&Chassis->pid.Location_Anglew_PID, &Chassis->pid.Location_Anglew_PIDParam);
    vw = PID_GetPIDOutput(&Chassis->pid.Location_Anglew_PID);
  } else {
    vw=0;
  }

  // 只有当 DT35 传感器状态有效时，才设置底盘速度
	if(DT35->state == 1){
    	Chassis_Set_Speed(Chassis_Run ,0,-vy,vw);
  } else {
    	Chassis_Set_Speed(Chassis_Stop,0,0,0);
  }
}

/**
 * @brief  底盘控制的主循环/状态机函数。
 * @details 根据底盘的 `Chassis_CtrlMode` (控制模式) 来切换不同的控制逻辑。
 * 此函数应在主循环中被周期性调用。
 * @param  无
 * @return 无
 * @note   - 在 `Chassis_Location` (定位) 模式下，它会固定调用 `Chassis_Stepback`
 * 函数，目标为距离163和角度0。
 * 
 * - `Chassis_Remote` 和 `Chassis_PC` 模式的逻辑可能在其他地方
 * (如中断服务程序或数据接收回调中) 处理。
 */
void Chassis_Control(){
  Chassis_DataTypeDef *Chassis = Chassis_GetChassisPtr();
	
  switch(Chassis->Chassis_CtrlMode){
    case Chassis_Remote: 
      break;
    case Chassis_PC : 
      break;
    case Chassis_Location:
      Chassis_Stepback(163,0);
      break;
  }
}

/**
 * @brief  将浮点数 (float) 压缩打包为 16 位无符号整数 (uint16_t)。
 * @brief  此格式专为 -8.0 至 +8.0 的范围设计。
 * @param  value: 输入的浮点数值。超出 (-8.0 ~ +8.0) 范围的值将被钳位。
 * @return 打包后的 16 位无符号整数。
 */
uint16_t float_to_2bytes(float value) {

  if (value == 0.0f) return 0;

  //利用类型强转直接访问float的符号位
  uint16_t sign = (*(uint32_t*)&value) >> 31;

  float abs_val = fabsf(value);

  //限制浮点值的范围在0~8之间
  if (abs_val > 8.0f) abs_val = 8.0f;

  int exp = 0;
  if (abs_val >= 1.0f) {
      while (abs_val >= 2.0f && exp < 4) {
          abs_val /= 2.0f;
          exp++;
      }
  } else {
      while (abs_val < 1.0f && exp > -3) {
          abs_val *= 2.0f;
          exp--;
      }
  }

  uint16_t mantissa = (uint16_t)((abs_val - 1.0f) * 4096.0f + 0.5f);
  
  return (sign << 15) | ((exp + 3) << 12) | (mantissa & 0x0FFF);
}
 