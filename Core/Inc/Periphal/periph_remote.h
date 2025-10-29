/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : periph_remote.h
 *  Description  : This file contains RoboMaster remote relevant function
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-04-06 10:56:47
 */


#ifndef REMOTE_PERIPH_H
#define REMOTE_PERIPH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "util_uart.h"
#include "stdlib.h"

#define Const_Remote_RX_BUFF_LEN             		54
#define Const_Remote_RX_FRAME_LEN                   18
#define Const_Remote_CHANNEL_VALUE_OFFSET           1024
#define Const_Remote_CHANNEL_ERROR_LIMIT            700

typedef enum {
    Remote_STATE_NULL       = 0,
    Remote_STATE_CONNECTED  = 1,
    Remote_STATE_LOST       = 2,
    Remote_STATE_ERROR      = 3,
    Remote_STATE_PENDING    = 4
} Remote_RemoteStateEnum;

typedef enum {
    Remote_SWITCH_NULL      = 0,
    Remote_SWITCH_UP        = 1,
    Remote_SWITCH_DOWN      = 2,
    Remote_SWITCH_MIDDLE    = 3
} Remote_SwitchStateEnum;

typedef struct {
    uint8_t w, a, s, d, shift, ctrl, q, e, r, f, g, z, x, c, v, b;
} Remote_KeyboardTypeDef;

// Remote_RemoteDataTypeDef 结构体定义了遥控器的数据格式
// 包含遥控器的通道数据、开关状态、鼠标数据、键盘数据以及遥控器的状态信息
typedef struct {
    struct {
        int16_t                 ch[5];  //Channel 0 channel 1 channel 2 channel 3 paddle wheel
        Remote_SwitchStateEnum  s[2];   //Switch 0 switch 1
    } remote;

    struct {
        int16_t  x;//mouse x
        int16_t  y;//mouse y
        int16_t  z;//mouse z
        uint8_t  l;//mouse Left key
        uint8_t  r;//mouse Right key
    } mouse;

    Remote_KeyboardTypeDef key;
    Remote_RemoteStateEnum state;

    uint8_t Remote_RxData[Const_Remote_RX_BUFF_LEN];

    float update_dt;
    uint32_t last_update_tick;
} Remote_RemoteDataTypeDef;

Remote_RemoteDataTypeDef* Remote_GetRemoteDataPtr(void);
void Remote_InitRemote(void);
Remote_SwitchStateEnum Remote_ToSwitchState(uint8_t sw);
uint8_t Remote_IsRemoteOffline(void);
void Remote_RXCallback(UART_HandleTypeDef* huart);
uint8_t Remote_IsRemoteError(void);
void Remote_DecodeKeyboardData(Remote_KeyboardTypeDef* key, uint16_t v);
void Remote_DecodeRemoteData(uint8_t* buff, int rxdatalen);
void Remote_ResetRemoteData(void);
int16_t Remote_CancelChannelOffset(uint16_t ch);

#endif

#ifdef __cplusplus
}
#endif
