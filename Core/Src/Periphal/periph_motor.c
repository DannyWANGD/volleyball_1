/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : periph_motor.c
 *  Description  : This file contains motor control function
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 00:43:48
 *  LastEditTime : 2024-05-20 09:49:39
 */
#include "sys_const.h"
#include "periph_motor.h"
#include "stdio.h"
#include "stdlib.h"
#include "alg_filter.h"
#include "lib_buff.h"
#include "cmsis_os.h"
#include "sys_dwt.h"
#include "util_gpio.h"

#define Const_Motor_UART_RX_LEN  78
   
Motor_MotorGroupTypeDef *Motor_groupHandle[MOTOR_GROUP_NUM];

Motor_MotorGroupTypeDef Motor_Pitch_Front_Motors;        // usart * 1
Motor_MotorGroupTypeDef Motor_Pitch_Left_Motors;   
Motor_MotorGroupTypeDef Motor_Pitch_Right_Motors; 

Motor_MotorGroupTypeDef Motor_Pitch_Front_Motors_A1;        // usart * 1
Motor_MotorGroupTypeDef Motor_Pitch_Left_Motors_A1;   
Motor_MotorGroupTypeDef Motor_Pitch_Right_Motors_A1;

Motor_MotorGroupTypeDef Motor_Yaw_Front_Motors;          // can1 * 3
Motor_MotorGroupTypeDef Motor_Yaw_Left_Motors;
Motor_MotorGroupTypeDef Motor_Yaw_Right_Motors; 
Motor_MotorGroupTypeDef Motor_Gimbal_Motors;     // can2 * 2

Motor_MotorTypeDef Motor_Pitch_Front_Motor;
Motor_MotorTypeDef Motor_Pitch_Left_Motor;
Motor_MotorTypeDef Motor_Pitch_Right_Motor;

Motor_MotorTypeDef Motor_Pitch_Front_Motor_A1;
Motor_MotorTypeDef Motor_Pitch_Left_Motor_A1;
Motor_MotorTypeDef Motor_Pitch_Right_Motor_A1;

Motor_MotorTypeDef Motor_Yaw_Front_Motor;
Motor_MotorTypeDef Motor_Yaw_Left_Motor;
Motor_MotorTypeDef Motor_Yaw_Right_Motor;

Motor_MotorTypeDef Motor_Gimbal_YawMotor;
Motor_MotorTypeDef Motor_Gimbal_PitchMotor;

uint8_t Motor_Uart6RxData[Const_Motor_UART_RX_LEN];

// Debug Paramters
volatile uint32_t dbg_uart_callback_cnt = 0;   // UART回调执行次数
volatile int dbg_rxdatalen = 0;                 // 接收到的数据长度
volatile uint8_t dbg_rx_id = 0;                 // 接收到的ID (rxdata[2] & 0xf)
volatile uint8_t dbg_rx_header0 = 0;            // 帧头字节0
volatile uint8_t dbg_rx_header1 = 0;            // 帧头字节1

/**
  * @brief      Motor encoder decoding callback function
  * @param      canid: CAN Handle number
  * @param      stdid: CAN identifier
  * @param      rxdata: CAN rx data buff
  * @retval     NULL
  */
void Motor_EncoderDecodeCallback(CAN_HandleTypeDef* phcan, UART_HandleTypeDef* huart, uint32_t stdid, uint8_t rxdata[], uint32_t len) {
    if (phcan != NULL && huart == NULL) {
        for (int i = 0; i < MOTOR_GROUP_NUM; i++) {
            for (int j = 0; j < Motor_groupHandle[i]->motor_num; j++) {
                if ((phcan == Motor_groupHandle[i]->can_handle) 
                      && (stdid == Motor_groupHandle[i]->motor_handle[j]->id) && phcan != NULL) {
                    Motor_groupHandle[i]->motor_handle[j]->callback(Motor_groupHandle[i]->motor_handle[j], rxdata, len);
                }
            }
        }
    }
    if (phcan == NULL && huart != NULL) {
        __HAL_DMA_DISABLE(huart->hdmarx);
        int rxdatalen = Const_Motor_UART_RX_LEN - Uart_DMACurrentDataCounter(huart->hdmarx->Instance);
        
        // 修复：DMA CIRCULAR模式下，收满78字节时计数器重载，rxdatalen=0
        // 此时实际收到了78字节
        if (rxdatalen == 0) {
            rxdatalen = Const_Motor_UART_RX_LEN;  // 78
        }
        
        // 调试：记录接收信息
        dbg_uart_callback_cnt++;
        dbg_rxdatalen = rxdatalen;
        dbg_rx_header0 = Motor_Uart6RxData[0];
        dbg_rx_header1 = Motor_Uart6RxData[1];
        dbg_rx_id = Motor_Uart6RxData[2] & 0x0f;
        
        for (int i = 0; i < MOTOR_GROUP_NUM; i++) {
            for (int j = 0; j < Motor_groupHandle[i]->motor_num; j++) {
                if ((huart == Motor_groupHandle[i]->uart_handle)&& (rxdatalen == 78 || rxdatalen == 16)) { 
                   if((Motor_Uart6RxData[2]& 0xf)== Motor_groupHandle[i]->motor_handle[j]->id){							
                        Motor_groupHandle[i]->motor_handle[j]->callback(Motor_groupHandle[i]->motor_handle[j], Motor_Uart6RxData , rxdatalen);    }            
                }
            }
        }
        __HAL_DMA_SET_COUNTER(huart->hdmarx, Const_Motor_UART_RX_LEN);
        __HAL_DMA_ENABLE(huart->hdmarx);
    }
}

/**
  * @brief      Initialize all motors
  * @param      NULL
  * @retval     NULL
  */
void Motor_InitAllMotors() {
   Uart_InitUartDMA(&huart6);
   Uart_ReceiveDMA(&huart6, Motor_Uart6RxData, Const_Motor_UART_RX_LEN);
	
  Motor_groupHandle[0] = &Motor_Pitch_Front_Motors;
	Motor_InitMotorGroup(&Motor_Pitch_Front_Motors, Motor_TYPE_Go1, 1, NULL, &huart6, 0);
    Motor_InitMotor(&Motor_Pitch_Front_Motor, Motor_TYPE_Go1, 1, 0.1, Go1_encoder_callback);
    Motor_Pitch_Front_Motors.motor_handle[0] = &Motor_Pitch_Front_Motor;
	
  Motor_groupHandle[1] = &Motor_Pitch_Left_Motors;
	Motor_InitMotorGroup(&Motor_Pitch_Left_Motors, Motor_TYPE_Go1, 1, NULL, &huart6, 0);
    Motor_InitMotor(&Motor_Pitch_Left_Motor, Motor_TYPE_Go1, 2, 0.1, Go1_encoder_callback);
    Motor_Pitch_Left_Motors.motor_handle[0] = &Motor_Pitch_Left_Motor;

  Motor_groupHandle[2] = &Motor_Pitch_Right_Motors;
	Motor_InitMotorGroup(&Motor_Pitch_Right_Motors, Motor_TYPE_Go1, 1, NULL, &huart6, 0);
    Motor_InitMotor(&Motor_Pitch_Right_Motor, Motor_TYPE_Go1, 3, 0.1, Go1_encoder_callback);
    Motor_Pitch_Right_Motors.motor_handle[0] = &Motor_Pitch_Right_Motor; 
	
  Motor_groupHandle[3] = &Motor_Yaw_Front_Motors;
	Motor_InitMotorGroup(&Motor_Yaw_Front_Motors, Motor_TYPE_DM4340, 1, &hcan1,NULL,  0x01);
    Motor_InitMotor(&Motor_Yaw_Front_Motor,Motor_TYPE_DM4340, 0x04, 0.1,  DM4340_encoder_callback);
    Motor_Yaw_Front_Motors.motor_handle[0] = &Motor_Yaw_Front_Motor;
  	  
  Motor_groupHandle[4] = &Motor_Yaw_Left_Motors;
	Motor_InitMotorGroup(&Motor_Yaw_Left_Motors, Motor_TYPE_DM4340, 1, &hcan1,NULL,  0x02);
    Motor_InitMotor(&Motor_Yaw_Left_Motor ,Motor_TYPE_DM4340, 0x05, 0.1,  DM4340_encoder_callback);
    Motor_Yaw_Left_Motors.motor_handle[0] = &Motor_Yaw_Left_Motor;
    
  Motor_groupHandle[5] = &Motor_Yaw_Right_Motors;
	Motor_InitMotorGroup(&Motor_Yaw_Right_Motors, Motor_TYPE_DM4340, 1, &hcan1,NULL,  0x03);
    Motor_InitMotor(&Motor_Yaw_Right_Motor,Motor_TYPE_DM4340, 0x06, 0.1,  DM4340_encoder_callback);
    Motor_Yaw_Right_Motors.motor_handle[0] = &Motor_Yaw_Right_Motor;  

  Motor_groupHandle[6] = &Motor_Pitch_Front_Motors_A1;
	Motor_InitMotorGroup(&Motor_Pitch_Front_Motors_A1, Motor_TYPE_A1, 1, NULL, &huart6, 0);
    Motor_InitMotor(&Motor_Pitch_Front_Motor_A1, Motor_TYPE_A1, 1, 0.1, A1_encoder_callback);
    Motor_Pitch_Front_Motors_A1.motor_handle[0] = &Motor_Pitch_Front_Motor_A1;
	
//   Motor_groupHandle[7] = &Motor_Pitch_Left_Motors_A1;
// 	Motor_InitMotorGroup(&Motor_Pitch_Left_Motors_A1, Motor_TYPE_A1, 1, NULL, &huart6, 0);
//     Motor_InitMotor(&Motor_Pitch_Left_Motor_A1, Motor_TYPE_A1, 2, 0.1, A1_encoder_callback);
//     Motor_Pitch_Left_Motors_A1.motor_handle[0] = &Motor_Pitch_Left_Motor_A1;

//   Motor_groupHandle[8] = &Motor_Pitch_Right_Motors_A1;
// 	Motor_InitMotorGroup(&Motor_Pitch_Right_Motors_A1, Motor_TYPE_A1, 1, NULL, &huart6, 0);
//     Motor_InitMotor(&Motor_Pitch_Right_Motor_A1, Motor_TYPE_A1, 3, 0.1, A1_encoder_callback);
//     Motor_Pitch_Right_Motors_A1.motor_handle[0] = &Motor_Pitch_Right_Motor_A1; 



	Motor_groupHandle[7] = &Motor_Gimbal_Motors;
	Motor_InitMotorGroup(&Motor_Gimbal_Motors, Motor_TYPE_Dji6020, 2, &hcan2, NULL, 0x2FE);
	Motor_InitMotor(&Motor_Gimbal_YawMotor, Motor_TYPE_Dji6020, 0x209, 0.1, Dji6020_encoder_callback);
	Motor_InitMotor(&Motor_Gimbal_PitchMotor, Motor_TYPE_Dji6020, 0x20A, 0.1, Dji6020_encoder_callback);
	Motor_Gimbal_Motors.motor_handle[0] = &Motor_Gimbal_YawMotor;
	Motor_Gimbal_Motors.motor_handle[1] = &Motor_Gimbal_PitchMotor;
	Motor_Gimbal_Motors.motor_handle[2] = NULL;
	Motor_Gimbal_Motors.motor_handle[3] = NULL;
	
	  Motor_groupHandle[8] = &Motor_Pitch_Right_Motors_A1;
	Motor_InitMotorGroup(&Motor_Pitch_Right_Motors_A1, Motor_TYPE_A1, 1, NULL, &huart6, 0);
    Motor_InitMotor(&Motor_Pitch_Right_Motor_A1, Motor_TYPE_A1, 2, 0.1, A1_encoder_callback);
    Motor_Pitch_Right_Motors_A1.motor_handle[0] = &Motor_Pitch_Right_Motor_A1; 
	
}   
/**
  * @brief      Initialize the motor
  * @param      pmotor: Pointer to motor object
  * @param      type: Type of motor (pwm or can)
  * @param      callback: Motor callback function
  * @retval     NULL
  */
void Motor_InitMotor(Motor_MotorTypeDef* pmotor, Motor_MotorTypeEnum type, uint16_t id, float fil_param, 
                     Motor_EncoderCallbackFuncTypeDef callback) {
    if (pmotor == NULL) return;
    pmotor->last_update_tick = 0;
    pmotor->type = type;
    pmotor->id = id;			
    pmotor->init = 0;			
    pmotor->is_online = 0;
    pmotor->output = 0;
    pmotor->callback = callback;
    Filter_LowPassInit(fil_param, &pmotor->fdb_fil_param);
	
}
/**
  * @brief      Initialization of motor group
  * @param      pgroup: Pointer to motor group
  * @param      type: Type of motor (pwm or can)
  * @param      motor_num: Number of motor group
  * @param      phcan: Pointer of can handle
  * @param      stdid: Motor id
  * @retval     NULL
  */
void  Motor_InitMotorGroup(Motor_MotorGroupTypeDef* pgroup, Motor_MotorTypeEnum type, uint8_t motor_num, CAN_HandleTypeDef* phcan, UART_HandleTypeDef *uart, uint16_t stdid) {
      if (pgroup == NULL) return;
   	  pgroup->motor_num = motor_num;
      pgroup->type = type;
	
 if (type == Motor_TYPE_DM8009 || type == Motor_TYPE_DM4310 || type == Motor_TYPE_DM4340|| type == Motor_TYPE_RM3508 || type == Motor_TYPE_LK5005 || type == Motor_TYPE_Dji6020 ) {
        if (phcan == NULL) return;
        pgroup->can_handle = phcan;
        Can_InitTxHeader(&(pgroup->can_header), stdid, 0, 8);
    }
 if (type == Motor_TYPE_A1 || type == Motor_TYPE_Go1) {
        pgroup->uart_handle = uart;
    }
   
    for (int i = 0; i < motor_num; ++i) 
        pgroup->motor_handle[i] = NULL;

}

void Motor_SetMotorOutput(Motor_MotorTypeDef* pmotor, float output) {
    pmotor->output = output;
}

/**
  * @brief      Get motor output value
  * @param      pmotor: Pointer to motor object
  * @retval     Output value
  */
uint16_t Motor_GetMotorOutput(Motor_MotorTypeDef* pmotor) {
if (pmotor == NULL) return 0;
int16_t ret = 0;

if (pmotor->type == Motor_TYPE_ECA4310) {
   ret = (int16_t)(pmotor->output / 1.4f / 50.0f * 5000.0f);
    LimitMax(ret, 2500);
    return (uint16_t)ret;
}
if (pmotor->type == Motor_TYPE_A1) {
        ret = (int16_t)(pmotor->output / 9.0f * 256.0f);               // output / 327.67f * 32767.0f
          return ((uint16_t)(ret & 0xff) << 8) | ((uint16_t)(ret >> 8) & 0xff);
    }
if (pmotor->type == Motor_TYPE_RM3508) {
        ret = (int16_t)((int)pmotor->output);
					return (uint16_t)ret;
        }
if (pmotor->type == Motor_TYPE_Go1) {
        ret = (int16_t)(pmotor->output / 6.33f * 256.0f);
          return (uint16_t)ret;
          }

if (pmotor->type == Motor_TYPE_Dji6020) {
			ret = (int16_t)(pmotor->output / 0.741f / 3.0f * 16384.0f);
			return ret;
	}

return 0;
}

uint32_t Motor_GetLKMotorOutput(Motor_MotorTypeDef* pmotor)
{
	int32_t ret = 0;
	if (pmotor->type == Motor_TYPE_LK5005) {

		ret = (int32_t)((int)pmotor->output);
					return (uint32_t)ret;
        }
	return 0;
}




uint8_t txdata[64];
void Motor_SendMotorGroupOutput(Motor_MotorGroupTypeDef *pgroup) {

    if (pgroup == NULL) return;

    if (pgroup->type == Motor_TYPE_ECA4310 || pgroup->type == Motor_TYPE_RM3508 || pgroup->type == Motor_TYPE_Dji6020 ) {
        txdata[0] = (uint8_t)(Motor_GetMotorOutput(pgroup->motor_handle[0]) >> 8);
        txdata[1] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[0]);
        txdata[2] = (uint8_t)(Motor_GetMotorOutput(pgroup->motor_handle[1]) >> 8);
        txdata[3] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[1]); 
        txdata[4] = (uint8_t)(Motor_GetMotorOutput(pgroup->motor_handle[2]) >> 8);
        txdata[5] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[2]);
        txdata[6] = (uint8_t)(Motor_GetMotorOutput(pgroup->motor_handle[3]) >> 8);
        txdata[7] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[3]);
    
        Can_SendMessage(pgroup->can_handle, &(pgroup->can_header), txdata);
        
    } else if (pgroup->type == Motor_TYPE_DM4310 || pgroup->type == Motor_TYPE_DM4340 ||pgroup->type == Motor_TYPE_DM8009 ) {

        uint16_t  pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
        pos_tmp = float_to_uint(0, Const_DMmotor_P_MIN, Const_DMmotor_P_MAX, 16);
        kp_tmp = float_to_uint(0, Const_DMmotor_KP_MIN, Const_DMmotor_KP_MAX, 12);
        kd_tmp = float_to_uint(0, Const_DMmotor_KD_MIN, Const_DMmotor_KD_MAX, 12);
     	
     	switch (pgroup->motor_handle[0]->type) {
     		case Motor_TYPE_DM4310 :
         	    vel_tmp = float_to_uint(0, -30, 30, 12);
                tor_tmp = float_to_uint(pgroup->motor_handle[0]->output , -10, 10, 12);
     	        break;
     		case Motor_TYPE_DM4340 :
     	        vel_tmp = float_to_uint(0, -30, 30, 12);
                tor_tmp = float_to_uint(pgroup->motor_handle[0]->output , -28, 28, 12);
     	        break;
     		case Motor_TYPE_DM8009 :
     	        vel_tmp = float_to_uint(0, -45, 45, 12);
                tor_tmp = float_to_uint(pgroup->motor_handle[0]->output  , -54, 54, 12);
     	        break;
     	    default:
                break;
     	}

        txdata[0] = (pos_tmp >> 8);
        txdata[1] = pos_tmp;
        txdata[2] = (vel_tmp >> 4);
        txdata[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
        txdata[4] = kp_tmp;
        txdata[5] = (kd_tmp >> 4);
        txdata[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
        txdata[7] = tor_tmp;
    
        Can_SendMessage(pgroup->can_handle, &(pgroup->can_header), txdata);

	} else if (pgroup->type == Motor_TYPE_A1) {

        memset(txdata, 0, 64);
        txdata[0] = 0XFE;
        txdata[1] = 0XEE;
        txdata[2] = (uint8_t)pgroup->motor_handle[0]->id;
        txdata[4] = 10;
                txdata[5] = 0XFF;
        txdata[12] = (uint8_t)(Motor_GetMotorOutput(pgroup->motor_handle[0]) >> 8);
        txdata[13] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[0]);
        uint32_t crc = crc32_core((uint32_t*)txdata, 7);
        txdata[30] = (uint8_t)crc;
        txdata[31] = (uint8_t)(crc >> 8);
        txdata[32] = (uint8_t)(crc >> 16);
        txdata[33] = (uint8_t)(crc >> 24);
        GPIO_Set( RS485_TX );  // 设置RS485为发送模式
        Uart_SendMessage(pgroup->uart_handle, txdata, 34, 100);
        // 等待UART发送完成（等待TC标志）
        while(__HAL_UART_GET_FLAG(pgroup->uart_handle, UART_FLAG_TC) == RESET);
        GPIO_Reset( RS485_TX );  // 发送完成后切回接收模式

    } else if (pgroup->type == Motor_TYPE_Go1) {

        memset(txdata, 0, 64);
        txdata[0] = 0XFE;
        txdata[1] = 0XEE;
        txdata[2] =  (0x01<<4) | ((uint8_t)pgroup->motor_handle[0]->id)  ;
        txdata[4] = (uint8_t)(Motor_GetMotorOutput(pgroup->motor_handle[0]) >> 8);
        txdata[3] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[0]);
        uint16_t crc = crc_ccitt(0,(uint8_t*)txdata, 15);
        txdata[15] = (uint8_t)crc;
        txdata[16] = (uint8_t)(crc >> 8);
        GPIO_Set( RS485_TX );	

        Uart_SendMessageDMA(pgroup->uart_handle, txdata, 17);

    } else if (pgroup->type == Motor_TYPE_LK5005) {

        txdata[0] = 0xA3;
        txdata[1] = 0x00;
        txdata[2] = 0x00;
        txdata[3] = 0x00;
        txdata[4] = (uint8_t)Motor_GetLKMotorOutput(pgroup->motor_handle[0]);
        txdata[5] = (uint8_t)(Motor_GetLKMotorOutput(pgroup->motor_handle[0]) >> 8);
        txdata[6] = (uint8_t)(Motor_GetLKMotorOutput(pgroup->motor_handle[0]) >> 16);
        txdata[7] = (uint8_t)(Motor_GetLKMotorOutput(pgroup->motor_handle[0]) >> 24);
        Can_SendMessage(pgroup->can_handle, &(pgroup->can_header), txdata);
    }
}

void Motor_Set_DM_MIT_Output(Motor_MotorGroupTypeDef* pgroup1, float pos, float vel, float tor, float kp,float kd)
{   
    pgroup1->dmoutput.position = pos;
    pgroup1->dmoutput.velocity = vel;
    pgroup1->dmoutput.torque = tor;
    pgroup1->dmoutput.KP = kp;
    pgroup1->dmoutput.KD = kd;
}
void Motor_Send_MIT_Output(Motor_MotorGroupTypeDef *pgroup)
{  
    if (pgroup == NULL) return;
    uint16_t  pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
    uint8_t txbuff[8];

    pos_tmp = float_to_uint(pgroup->dmoutput.position, Const_DMmotor_P_MIN, Const_DMmotor_P_MAX, 16);
    kp_tmp = float_to_uint(pgroup->dmoutput.KP, Const_DMmotor_KP_MIN, Const_DMmotor_KP_MAX, 12);
    kd_tmp = float_to_uint(pgroup->dmoutput.KD, Const_DMmotor_KD_MIN, Const_DMmotor_KD_MAX, 12);
	
	switch (pgroup->motor_handle[0]->type){
		case Motor_TYPE_DM4310 :
	        vel_tmp = float_to_uint(pgroup->dmoutput.velocity, -30, 30, 12);
            tor_tmp = float_to_uint(pgroup->dmoutput.torque, -10, 10, 12);
	        break;
        case Motor_TYPE_DM4340 :
	        vel_tmp = float_to_uint(pgroup->dmoutput.velocity, -30, 30, 12);
            tor_tmp = float_to_uint(pgroup->dmoutput.torque, -28, 28, 12);
	        break;
        case Motor_TYPE_DM8009 :
	        vel_tmp = float_to_uint(pgroup->dmoutput.velocity, -45, 45, 12);
            tor_tmp = float_to_uint(pgroup->dmoutput.torque, -54, 54, 12);
            break;
	    default:
            break;
	}
   
    txbuff[0] = (pos_tmp >> 8);
    txbuff[1] = pos_tmp;
    txbuff[2] = (vel_tmp >> 4);
    txbuff[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
    txbuff[4] = kp_tmp;
    txbuff[5] = (kd_tmp >> 4);
    txbuff[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
    txbuff[7] = tor_tmp;

    Can_SendMessage(pgroup->can_handle, &(pgroup->can_header), txbuff);
}
 
 void Motor_DM_Basic_Output(Motor_MotorGroupTypeDef *pgroup , Motor_DMBasicCtrlEnum basic)
 {
    if (pgroup == NULL) return;
    uint8_t txbuff[8];
    txbuff[0] = 0xFF;
    txbuff[1] = 0xFF;
    txbuff[2] = 0xFF;
    txbuff[3] = 0xFF;
    txbuff[4] = 0xFF;
    txbuff[5] = 0xFF;
    txbuff[6] = 0xFF;
	switch (basic)
	{
	case Motor_Enable:
	    txbuff[7] = 0xFC;
		break;

	case Motor_Disable:
	    txbuff[7] = 0xFD;
		break;

	case Motor_SaveInitpos:
        txbuff[7] = 0xFE;
		break;

	case Motor_Clearerr :
        txbuff[7] = 0xFB;
        break;
    }

  	HAL_Delay(500);
    Can_SendMessage(pgroup->can_handle, &(pgroup->can_header), txbuff);	
 }

void Motor_DM_Basic_Ctrl(Motor_DMBasicCtrlEnum basic)
{
    Motor_DM_Basic_Output(&Motor_Yaw_Front_Motors,basic);
    Motor_DM_Basic_Output(&Motor_Yaw_Left_Motors ,basic);
    Motor_DM_Basic_Output(&Motor_Yaw_Right_Motors,basic);
}

 void DM4310_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    
    if (pmotor == NULL) return;
    if (len != 8) return;
             
    pmotor->encoder.angle  = uint_to_float((rxbuff[1]<<8)|rxbuff[2], Const_DMmotor_P_MIN,Const_DMmotor_P_MAX, 16)*180/PI;
    pmotor->encoder.speed  = (uint_to_float((rxbuff[3]<<4)|(rxbuff[4]>>4), -30, 30, 12));  
    pmotor->encoder.torque = uint_to_float(((rxbuff[4]&0xF)<<8)|rxbuff[5], -10, 10, 12);
    pmotor->encoder.temp = rxbuff[7]; 
    pmotor->init = 1; 
    pmotor->encoder.last_angle = pmotor->init == 1 ? pmotor->encoder.angle : (rxbuff[0] << 8 | rxbuff[1]);
    int diff = pmotor->encoder.angle - pmotor->encoder.last_angle;      
    if (diff < -4096)           
        pmotor->encoder.round_count++;
    else if (diff > 4096)       
        pmotor->encoder.round_count--;
    // Calculate the shaft angle because the reduction ratio needs to be divided by 36
    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 10.0f + 
                                       (float)pmotor->encoder.angle / 8192.0f * 10.0f;
    if (pmotor->encoder.round_count > 10000) {
        pmotor->encoder.consequent_angle -= 10 * pmotor->encoder.round_count; 
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.round_count < -10000) {
        pmotor->encoder.consequent_angle += 10 * pmotor->encoder.round_count; 
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.limited_angle < pmotor->encoder.init_offset - 180 && pmotor->encoder.init_offset >= 180)
        pmotor->encoder.limited_angle += 360;
    else if (pmotor->encoder.limited_angle > pmotor->encoder.init_offset + 180 && pmotor->encoder.init_offset < 180)
        pmotor->encoder.limited_angle -= 360;
 
    pmotor->type = Motor_TYPE_DM4310;
    pmotor->update_dt = DWT_GetDeltaT(&pmotor->last_update_tick);
    
} 
 void DM4340_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    
    if (pmotor == NULL) return;
    if (len != 8) return;
             
    pmotor->encoder.angle  = uint_to_float((rxbuff[1]<<8)|rxbuff[2], Const_DMmotor_P_MIN,Const_DMmotor_P_MAX, 16)*180/PI;
    pmotor->encoder.speed  = (uint_to_float((rxbuff[3]<<4)|(rxbuff[4]>>4), -30, 30, 12));  
    pmotor->encoder.torque = uint_to_float(((rxbuff[4]&0xF)<<8)|rxbuff[5], -28, 28, 12);
    pmotor->encoder.temp = rxbuff[7]; 
    pmotor->init = 1; 
    pmotor->encoder.last_angle = pmotor->init == 1 ? pmotor->encoder.angle : (rxbuff[0] << 8 | rxbuff[1]);

    int diff = pmotor->encoder.angle - pmotor->encoder.last_angle;      
    if (diff < -4096)           
        pmotor->encoder.round_count++;
    else if (diff > 4096)       
        pmotor->encoder.round_count--;
    // Calculate the shaft angle because the reduction ratio needs to be divided by 36
    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 10.0f + 
                                       (float)pmotor->encoder.angle / 8192.0f * 10.0f;
    if (pmotor->encoder.round_count > 10000) {
        pmotor->encoder.consequent_angle -= 10 * pmotor->encoder.round_count; 
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.round_count < -10000) {
        pmotor->encoder.consequent_angle += 10 * pmotor->encoder.round_count; 
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.limited_angle < pmotor->encoder.init_offset - 180 && pmotor->encoder.init_offset >= 180)
        pmotor->encoder.limited_angle += 360;
    else if (pmotor->encoder.limited_angle > pmotor->encoder.init_offset + 180 && pmotor->encoder.init_offset < 180)
        pmotor->encoder.limited_angle -= 360;
 
    pmotor->type = Motor_TYPE_DM4340;
    pmotor->update_dt = DWT_GetDeltaT(&pmotor->last_update_tick);
	pmotor->watchdog = 0;
    
} 
 void DM8009_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    
    if (pmotor == NULL) return;
    if (len != 8) return;
             
    pmotor->encoder.angle  = uint_to_float((rxbuff[1]<<8)|rxbuff[2], Const_DMmotor_P_MIN,Const_DMmotor_P_MAX, 16)*180/PI;
    pmotor->encoder.speed  = (uint_to_float((rxbuff[3]<<4)|(rxbuff[4]>>4), -45, 45, 12));  
    pmotor->encoder.torque = uint_to_float(((rxbuff[4]&0xF)<<8)|rxbuff[5], -54, 54, 12);
    pmotor->encoder.temp = rxbuff[7]; 
    pmotor->init = 1; 
    pmotor->encoder.last_angle = pmotor->init == 1 ? pmotor->encoder.angle : (rxbuff[0] << 8 | rxbuff[1]);
    int diff = pmotor->encoder.angle - pmotor->encoder.last_angle;      
    if (diff < -4096)           
        pmotor->encoder.round_count++;
    else if (diff > 4096)       
        pmotor->encoder.round_count--;
    // Calculate the shaft angle because the reduction ratio needs to be divided by 36
    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 10.0f + 
                                       (float)pmotor->encoder.angle / 8192.0f * 10.0f;
    if (pmotor->encoder.round_count > 10000) {
        pmotor->encoder.consequent_angle -= 10 * pmotor->encoder.round_count; 
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.round_count < -10000) {
        pmotor->encoder.consequent_angle += 10 * pmotor->encoder.round_count; 
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.limited_angle < pmotor->encoder.init_offset - 180 && pmotor->encoder.init_offset >= 180)
        pmotor->encoder.limited_angle += 360;
    else if (pmotor->encoder.limited_angle > pmotor->encoder.init_offset + 180 && pmotor->encoder.init_offset < 180)
        pmotor->encoder.limited_angle -= 360;
 
    pmotor->type = Motor_TYPE_DM8009;
    pmotor->update_dt = DWT_GetDeltaT(&pmotor->last_update_tick);
    
} 
 
void ECA4310_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
 
 if (pmotor == NULL) return;
 if (len != 8) return;

 pmotor->encoder.last_angle = (!pmotor->init) ? (float)((int16_t)((uint16_t)(rxbuff[0] << 8 | (uint16_t)rxbuff[1]))) / 100.0f + 180.0f : pmotor->encoder.angle;
 pmotor->encoder.angle  = (float)((int16_t)(((uint16_t)rxbuff[0] << 8) | (uint16_t)rxbuff[1])) / 100.0f + 180.0f;
 pmotor->encoder.speed  = (float)((int16_t)(((uint16_t)rxbuff[2] << 8) | (uint16_t)rxbuff[3])) * 0.010472f;
 pmotor->encoder.current = (float)((int16_t)(((uint16_t)rxbuff[4] << 8) | (uint16_t)rxbuff[5])) / 100.0f;
 pmotor->encoder.torque = pmotor->encoder.current * 1.4f;
 pmotor->encoder.temp  = ((float)((int16_t)(rxbuff[6])) - 50.0f) / 2.0f; 
 pmotor->encoder.error_code = rxbuff[7];
 pmotor->init = 1;

 if ((pmotor->encoder.last_angle > 358.0f) && (pmotor->encoder.angle < 2.0f)) 
   pmotor->encoder.round_count++;
 else if ((pmotor->encoder.last_angle < 2.0f) && (pmotor->encoder.angle > 358.0f)) 
   pmotor->encoder.round_count--;

 // Calculate continuous angle
 pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 360.0f + (float)pmotor->encoder.angle + pmotor->encoder.init_offset;
 pmotor->encoder.limited_angle = (float)pmotor->encoder.angle;
 
 if (pmotor->encoder.limited_angle < pmotor->encoder.init_offset - 180 && pmotor->encoder.init_offset >= 180)
  pmotor->encoder.limited_angle += 360;
 else if (pmotor->encoder.limited_angle > pmotor->encoder.init_offset + 180 && pmotor->encoder.init_offset < 180)
  pmotor->encoder.limited_angle -= 360;

 pmotor->type = Motor_TYPE_ECA4310;
 pmotor->update_dt = DWT_GetDeltaT(&pmotor->last_update_tick);
 
}

void A1_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    if (pmotor == NULL) return;
    if (len != 78) return;
    if ((rxbuff[0] != 0XFE) || (rxbuff[1] != 0XEE)) return;

    
    // Assign a value to the previous angle and get the latest angle
    pmotor->encoder.last_angle = pmotor->encoder.angle;
		pmotor->encoder.angle   = (float)((int32_t)((uint32_t)rxbuff[30] | (uint32_t)rxbuff[31] << 8 | (uint32_t)rxbuff[32] << 16 | (uint32_t)rxbuff[33] << 24)) * 2.0f * 180.0f / 16384.0f / 9.1f;
    // 输出轴速度
		pmotor->encoder.speed   = (float)((int16_t)((uint16_t)rxbuff[14] | (uint16_t)rxbuff[15] << 8) / 128.0f) / 9.1f;
    // 电机力矩，不过当成电流来考虑，A1电机力矩常数为0.87Nm/A
		pmotor->encoder.current = (float)((int16_t)((uint16_t)rxbuff[12] | (uint16_t)rxbuff[13] << 8) / 256.0f);
    pmotor->encoder.temp    = (float)((uint32_t)rxbuff[6]);
    pmotor->encoder.limited_angle = pmotor->encoder.angle;
    pmotor->encoder.consequent_angle = pmotor->encoder.angle;
    pmotor->type = Motor_TYPE_A1;
    pmotor->watchdog = 0;  // 清零看门狗，表示收到电机数据
}
extern uint64_t output_state;
void Go1_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
	
    if (pmotor == NULL) return;
    if (len != 16) return;
    if ((rxbuff[0] != 0XFD) || (rxbuff[1] != 0XEE)) return; 
//  	if(pmotor == &Motor_Pitch_Front_Motor){output_state =2;}
//	  if(pmotor == &Motor_Pitch_Left_Motor ){output_state =3;}
//	  if(pmotor == &Motor_Pitch_Right_Motor){output_state =1;}
    // Assign a value to the previous angle and get the latest angle
    pmotor->encoder.last_angle = pmotor->encoder.angle;
    pmotor->encoder.angle   = (float)((int32_t)((uint32_t)rxbuff[7] | (uint32_t)rxbuff[8] << 8 | (uint32_t)rxbuff[9] << 16 | (uint32_t)rxbuff[10] << 24)) * 180.0f / 16384.0f / 6.33f;
    pmotor->encoder.speed   = (float)((int16_t)((uint16_t)rxbuff[5] | (uint16_t)rxbuff[6] << 8) / 128.0f * PI) / 6.33f;
    pmotor->encoder.torque  = (float)((int16_t)((uint16_t)rxbuff[3] | (uint16_t)rxbuff[4] << 8) / 256.0f * 6.33f);
    pmotor->encoder.temp    = (float)((uint32_t)rxbuff[11]);
		pmotor->encoder.error_code = rxbuff[12];
    pmotor->encoder.limited_angle = pmotor->encoder.angle;
    pmotor->encoder.consequent_angle = pmotor->encoder.angle;
    pmotor->type = Motor_TYPE_Go1;
   // pmotor->update_dt = DWT_GetDeltaT(&pmotor->last_update_tick);
		pmotor->watchdog = 0;    
}

uint32_t crc32_core(uint32_t* ptr, uint32_t len) {
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++) {
        xbit = (uint32_t)1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++){
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}
const uint16_t crc_ccitt_table[256] = {
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c)
{
	return (crc >> 8) ^ crc_ccitt_table[(crc ^ c) & 0xff];
}

/**
 *	crc_ccitt - recompute the CRC (CRC-CCITT variant) for the data
 *	buffer
 *	@crc: previous CRC value
 *	@buffer: data pointer
 *	@len: number of bytes in the buffer
 */
uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, uint32_t len)
{
	while (len--)
		crc = crc_ccitt_byte(crc, *buffer++);
	return crc;
}

void rm3508_encoder_callback(Motor_MotorTypeDef* motor, uint8_t* rxdata, uint32_t len)
{
        if(motor == NULL) return;
        if(len != 8) return;

        motor->encoder.last_speed = motor->encoder.speed;
        motor->encoder.last_angle = motor->init == 1 ? motor->encoder.angle : (rxdata[0] << 8 | rxdata[1]);
        motor->encoder.angle      = (float)((uint16_t)((uint16_t)rxdata[0] << 8 | (uint16_t)rxdata[1]));
        motor->encoder.speed      = (float)((int16_t)((uint16_t)rxdata[2] << 8 | (uint16_t)rxdata[3]));
        motor->encoder.current    = (float)((int16_t)((uint16_t)rxdata[4] << 8 | (uint16_t)rxdata[5]));
        motor->encoder.temp       = 0;
				motor->encoder.init_offset = motor->init == 0 ? motor->encoder.angle / 8192.0f / (3591.0f / 187.0f) * 360.0f : motor->encoder.init_offset;
				motor->encoder.round_count_reduction = motor->encoder.round_count / (3591.0f / 187.0f);
				motor->encoder.cousequence_angle_reduction = motor->encoder.consequence_angle / (3591.0f / 187.0f) - motor->encoder.init_offset;
				motor->encoder.consequence_angle_reduction_fil = Filter_Aver(motor->encoder.cousequence_angle_reduction, &motor->encoder.angle_fil_window);
				motor->encoder.consequence_speed = motor->encoder.speed / 60 * 2 * PI * 0.348f / (3591.0f / 187.0f);
				motor->init = 1;
	
        motor->encoder.accelerate = (motor->encoder.speed - motor->encoder.last_speed) / motor->update_dt;
        int diff = motor->encoder.angle - motor->encoder.last_angle;

        if(diff < -4096)
        {
                motor->encoder.round_count++;
        }
        else if(diff > 4096)
        {
                motor->encoder.round_count--;
        }

        // Calculate continuous angle
        motor->encoder.consequence_angle = (float)(motor->encoder.angle / 8192.0f * 360.0f) + (float)(motor->encoder.round_count * 360.0f);
        
        if(motor->encoder.round_count > 10000)
        {
                motor->encoder.consequence_angle -= 10 * motor->encoder.round_count;
                motor->encoder.round_count = 0;
        }
        else if(motor->encoder.round_count < -10000)
        {
                motor->encoder.consequence_angle += 10 * motor->encoder.round_count;
                motor->encoder.round_count = 0;
        }

        if(motor->encoder.limit_angle < motor->encoder.init_offset - 180 && motor->encoder.init_offset >= 180)
        {
              motor->encoder.limit_angle += 360;
        }
        else if(motor->encoder.limit_angle > motor->encoder.init_offset + 180 && motor->encoder.init_offset < 180)
        {
                motor->encoder.limit_angle -= 360;
        }

        motor->type = Motor_TYPE_RM3508;
        motor->update_dt = DWT_GetDeltaT(&motor->last_update_tick);


}

void lk5005_encoder_callback(Motor_MotorTypeDef* motor, uint8_t* rxdata, uint32_t len)
{
	if(motor == NULL) return;
        if(len != 8) return;
	//�����
	if(rxdata[0] == 0xA3)
	{
		motor->encoder.last_angle = motor->init == 1 ? motor->encoder.angle : ((uint16_t)((uint16_t)((uint16_t)rxdata[6] | (uint16_t)rxdata[7] << 8))) * 360.0f / 32767.0f;
		motor->encoder.last_speed = motor->encoder.speed;
		motor->encoder.angle 	  = ((uint16_t)((uint16_t)((uint16_t)rxdata[6] | (uint16_t)rxdata[7] << 8))) * 360.0f / 32767.0f;
		motor->encoder.speed	  = deg2rad(((int16_t)((uint16_t)rxdata[4] | (uint16_t)rxdata[5] << 8)));
		motor->encoder.current	  = ((int16_t)((uint16_t)((uint16_t)rxdata[2] | (uint16_t)rxdata[3] << 8))) / 24.0f;
		motor->encoder.temp		  = (int8_t)rxdata[1];
		
		motor->type = Motor_TYPE_LK5005;
        motor->update_dt = DWT_GetDeltaT(&motor->last_update_tick);
	}
}

void Dji6020_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    if (pmotor == NULL) return;
    if (len != 8) return;

    uint16_t reverse_angle = (uint16_t)((uint16_t)rxbuff[0] << 8 | (uint16_t)rxbuff[1]);
    reverse_angle = (reverse_angle == 0) ? 0 : (8192 - reverse_angle);

    pmotor->encoder.last_angle = pmotor->init == 1 ? pmotor->encoder.angle : ((float)(reverse_angle) / 8192.0f * 360.0f);
    pmotor->encoder.angle   = (float)(reverse_angle) / 8192.0f * 360.0f;
    pmotor->encoder.speed   = -(float)((int16_t)((uint16_t)rxbuff[2] << 8 | (uint16_t)rxbuff[3])) * 0.1047f;
    pmotor->encoder.current = -(float)((int16_t)((uint16_t)rxbuff[4] << 8 | (uint16_t)rxbuff[5])) / 16384.0f * 3.0f;
    pmotor->encoder.torque = pmotor->encoder.current * 0.741f;
    pmotor->encoder.temp = (float)(rxbuff[6]); 
    pmotor->init = 1; 

    // Calculate angle difference and number of cycles
    if ((pmotor->encoder.last_angle > 358.0f) && (pmotor->encoder.angle < 2.0f)) 
        pmotor->encoder.round_count++;
    else if ((pmotor->encoder.last_angle < 2.0f) && (pmotor->encoder.angle > 358.0f)) 
        pmotor->encoder.round_count--;
        
    // Calculate continuous angle
    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 360.0f + (float)pmotor->encoder.angle + pmotor->encoder.init_offset;
    float temp_limited_angle = (float)pmotor->encoder.angle - pmotor->encoder.init_offset;
    while (temp_limited_angle < -180.0f) temp_limited_angle += 360.0f;
    while (temp_limited_angle > 180.0f) temp_limited_angle -= 360.0f;
    pmotor->encoder.limited_angle = temp_limited_angle;
    
    pmotor->type = Motor_TYPE_Dji6020;
    pmotor->update_dt = DWT_GetDeltaT(&pmotor->last_update_tick); 
    
}

// 在文件顶部声明
/*volatile uint32_t uart6_idle_count = 0;

void Uart_ReceiveHandler(UART_HandleTypeDef *huart) {
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        
        // 调试：统计 UART6 IDLE 中断次数
        if (huart->Instance == USART6) {
            uart6_idle_count++;
        }
        
        Uart_RxIdleCallback(huart);
    }
}*/
