#ifndef APP_COMMUNICATE_H
#define APP_COMMUNICATE_H

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

#define Const_BC_RX_BUFF_LEN  64
#define Const_BC_TX_BUFF_LEN 1024

typedef struct {
    uint8_t BC_RxData[Const_BC_RX_BUFF_LEN];
    uint8_t BC_TxData[Const_BC_TX_BUFF_LEN];
    uint32_t rx_len;
} BC_DataTypeDef;

void BC_Task(void const * argument);
void BC_Init(void);
BC_DataTypeDef* BC_GetBCDataPtr(void);
void BC_RXCallback(UART_HandleTypeDef* huart);

void Comm_Task(void const * argument);


#endif