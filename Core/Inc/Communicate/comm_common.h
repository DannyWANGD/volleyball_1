/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : protocol_common.h
 *  Description  : This file contains Bus communication control function
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:03:07
 *  LastEditTime : 2023-10-04 00:33:47
 */


#ifndef COMM_COMMON_H
#define COMM_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "main.h"

#define COMM_SEND_BUFF_LEN  64
#define COMM_RECEIVE_BUFF_LEN 64

typedef enum {
    Comm_STATE_NULL      = 0,
    Comm_STATE_CONNECTED = 1,
    Comm_STATE_LOST      = 2,
    Comm_STATE_ERROR     = 3,
    Comm_STATE_PENDING   = 4
} Comm_CommStateEnum;


typedef struct {
    Comm_CommStateEnum state;

    uint8_t usb_watchBuff[COMM_RECEIVE_BUFF_LEN];
    uint8_t usb_sendBuff[COMM_SEND_BUFF_LEN];

    uint32_t last_rx_tick;
    float rx_dt;

    uint32_t last_tx_tick;
    float tx_dt;

    uint32_t last_gim_rx_tick;
    float gim_rx_dt;
	
	  float rx_len;
} Comm_DataTypeDef;


extern Comm_DataTypeDef Comm_Data;


void Comm_InitComm(void);
Comm_DataTypeDef* Comm_GetBusDataPtr(void);
uint8_t Comm_IsCommOffline(void);
uint8_t Comm_IsGimCommOffline(void);
void Comm_SendBlockError(void);
void Comm_ResetCommData(void);
void Comm_SendCommData(void);
void Comm_DecodeData(uint8_t buff[], uint32_t rxdatalen);
void Comm_GimDecode(uint32_t stdid, uint8_t rxdata[]);
void Comm_UartCallBack(UART_HandleTypeDef* huart);
void Comm_FDCANCallBack(CAN_HandleTypeDef* phcan, uint32_t stdid, uint8_t rxdata[], uint32_t len);

#endif

#ifdef __cplusplus
}
#endif
