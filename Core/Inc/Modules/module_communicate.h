#ifndef PROTOCOL_TRANSMIT_H
#define PROTOCOL_TRANSMIT_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "main.h"

#define Const_Protocol_Transmit_BUFF_SIZE 2
#define Const_Protocol_Receive_BUFF_SIZE 2


#define PROTOCOL_SEND_BUFF_LEN  64
#define PROTOCOL_RECEIVE_BUFF_LEN 64

#define Const_Com_RX_BUFF_LEN  64
#define Const_Com_TX_BUFF_LEN 1024

typedef enum {
    Protocol_STATE_NULL      = 0,
    Protocol_STATE_CONNECTED = 1,
    Protocol_STATE_LOST      = 2,
    Protocol_STATE_ERROR     = 3,
    Protocol_STATE_PENDING   = 4
} Protocol_CommStateEnum;


typedef struct {
    Protocol_CommStateEnum state;

    uint8_t usb_watchBuff[PROTOCOL_RECEIVE_BUFF_LEN];
    uint8_t usb_sendBuff[PROTOCOL_SEND_BUFF_LEN];
	  uint8_t uart_watchBuff[PROTOCOL_RECEIVE_BUFF_LEN];

    uint32_t last_rx_tick;
    float rx_dt;

    uint32_t last_tx_tick;
    float tx_dt;

    uint32_t last_gim_rx_tick;
    float gim_rx_dt;
} Protocol_DataTypeDef;

extern Protocol_DataTypeDef Protocol_Data;


static uint32_t _set_Gimbal_Data_(uint8_t *buff) ;

typedef struct {
    uint32_t (*bus_func)(uint8_t *buff);
} Protocol_SendEntry;

extern Protocol_SendEntry ProtocolCmd_Send[Const_Protocol_Transmit_BUFF_SIZE];

typedef struct {
    uint32_t (*bus_func)(uint8_t *buff);
} Protocol_ReceiveEntry;


extern Protocol_ReceiveEntry ProtocolCmd_Receive[Const_Protocol_Receive_BUFF_SIZE];

void Protocol_InitProtocol(void);
Protocol_DataTypeDef* Protocol_GetBusDataPtr(void);
void Protocol_ResetProtocolData();
void Protocol_SendProtocolData(void) ;
void Protocol_DecodeData(uint8_t buff[], uint32_t rxdatalen);
void Communicate_RXCallback(UART_HandleTypeDef* huart);
static uint8_t Protocol_UartVerify(uint8_t buff[], uint32_t rxdatalen);
#endif

#ifdef __cplusplus
}
#endif

