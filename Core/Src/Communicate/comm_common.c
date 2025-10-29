/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : protocol_common.c
 *  Description  : This file contains Bus communication control function
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:02:53
 *  LastEditTime : 2023-10-04 01:05:20
 */


#include "comm_common.h"
#include "comm_receive.h"
#include "comm_transmit.h"

#include "lib_buff.h"
#include "stdlib.h"
#include "sys_dwt.h"
#include "sys_const.h"
#include "alg_crc.h"
#include "util_usb.h"
#include "periph_motor.h"
#include "util_uart.h"

Comm_DataTypeDef Comm_Data;


/**
  * @brief      Inter bus communication initialization
  * @param      NULL
  * @retval     NULL
  */
void Comm_InitComm() {
    Comm_DataTypeDef *buscomm = Comm_GetBusDataPtr();
    Comm_ResetCommData();
	  Uart_InitUartDMA(Const_Communicate_UART_HANDLER);
    Uart_ReceiveDMA(Const_Communicate_UART_HANDLER, buscomm->usb_watchBuff, COMM_RECEIVE_BUFF_LEN);    
}


/**
  * @brief      Gets the pointer to the bus communication data object
  * @param      NULL
  * @retval     Pointer to bus communication data object
  */
Comm_DataTypeDef* Comm_GetBusDataPtr() {
    return &Comm_Data;
}


/**
  * @brief      Check whether the dual bus communication is offline
  * @param      NULL
  * @retval     NULL
  */
uint8_t Comm_IsCommOffline() {
    Comm_DataTypeDef *buscomm = Comm_GetBusDataPtr();
    if (DWT_GetDeltaTWithOutUpdate(&buscomm->last_rx_tick) > Const_Comm_OFFLINE_TIME) {
        buscomm->state = Comm_STATE_LOST;
    }
    return buscomm->state != Comm_STATE_CONNECTED;
}


/**
  * @brief      Check whether the dual bus communication is offline
  * @param      NULL
  * @retval     NULL
  */
uint8_t Comm_IsGimCommOffline() {
    Comm_DataTypeDef *buscomm = Comm_GetBusDataPtr();

    return (DWT_GetDeltaTWithOutUpdate(&buscomm->last_gim_rx_tick) > Const_Comm_OFFLINE_TIME) ? 1 : 0;
}


/**
  * @brief      Comm send block error handler
  * @param      NULL
  * @retval     NULL
  */
void Comm_SendBlockError() {

}


/**
  * @brief      Reset inter bus communication data object
  * @param      NULL
  * @retval     NULL
  */
void Comm_ResetCommData() {
    Comm_DataTypeDef *buscomm = Comm_GetBusDataPtr();
    buscomm->rx_dt = DWT_GetDeltaT(&buscomm->last_rx_tick);
    buscomm->tx_dt = DWT_GetDeltaT(&buscomm->last_tx_tick);
}


/**
  * @brief      Data sending function of serial port in inter bus communication
  * @param      NULL
  * @retval     NULL
  */
void Comm_SendCommData() {
    Comm_DataTypeDef *buscomm = Comm_GetBusDataPtr();

   // buscomm->tx_dt = DWT_GetDeltaT(&buscomm->last_tx_tick);
    uint32_t len;   
    // to nuc
    len = 4;

        buscomm->usb_sendBuff[0] = 0X5A;
        buscomm->usb_sendBuff[1] = 0XA5;
        for (int i = 0; i < Const_Comm_Transmit_BUFF_SIZE; i++) {
            if (CommCmd_Send[i].bus_func != NULL) {
                len += CommCmd_Send[i].bus_func(buscomm->usb_sendBuff + len);
            }
        }
        buscomm->usb_sendBuff[2] = (uint8_t)(len & 0xff);
        buscomm->usb_sendBuff[3] = (uint8_t)((len & 0xff00) >> 8);
        buscomm->usb_sendBuff[len] = 0X7A;
        buscomm->usb_sendBuff[len + 1 ] = 0XA7;
       // Usb_SendBuff(buscomm->usb_sendBuff, 256);
	   // 	Uart_SendMessageDMA(Const_Communicate_UART_HANDLER, buscomm->usb_sendBuff, len + 2);
        buscomm->state = Comm_STATE_CONNECTED;
}


static uint8_t Comm_MergeAndverify(uint8_t buff[], uint32_t rxdatalen) {
    Comm_DataTypeDef *buscomm = Comm_GetBusDataPtr();
    
 
    if ((buff[0] == 0x5A) && (buff[1] == 0xA5)) {
    
        memcpy(buscomm->usb_watchBuff, buff, rxdatalen);
        return 1;
    }  
    else {
        return 0;
    }
    
}


/**
  * @brief      Data decoding function of serial port in inter bus communication
  * @param      buff: Data buffer
  * @param      rxdatalen: data length
  * @retval     NULL
  */
void Comm_DecodeData(uint8_t buff[], uint32_t rxdatalen) {
    Comm_DataTypeDef *buscomm = Comm_GetBusDataPtr();

    if (Comm_MergeAndverify(buff, rxdatalen) == 0) return; 
    buscomm->rx_dt = DWT_GetDeltaT(&buscomm->last_rx_tick);
    uint32_t len = 4;

        for (int i = 0; i < Const_Comm_Receive_BUFF_SIZE; i++) {
            if (CommCmd_Receive[i].bus_func != NULL) {
                len += CommCmd_Receive[i].bus_func(buscomm->usb_watchBuff + len);
            }
        } 
}

void Comm_CANCallBack(CAN_HandleTypeDef* phcan, uint32_t stdid, uint8_t rxdata[], uint32_t len) {
    if (len != 8) return;
  //  Comm_GimDecode(stdid, rxdata);
}
void Communicate_RXCallback(UART_HandleTypeDef* huart) {
    Comm_DataTypeDef* comm = Comm_GetBusDataPtr();

    /* clear DMA transfer complete flag */
    __HAL_DMA_DISABLE(huart->hdmarx);

    /* handle uart data from DMA */
    int rxdatalen = COMM_RECEIVE_BUFF_LEN - Uart_DMACurrentDataCounter(huart->hdmarx->Instance);
    comm->rx_len = rxdatalen;
	
	  Comm_DecodeData(comm->usb_watchBuff,rxdatalen);
	
    /* restart dma transmission */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, COMM_RECEIVE_BUFF_LEN);
    __HAL_DMA_ENABLE(huart->hdmarx);
}


