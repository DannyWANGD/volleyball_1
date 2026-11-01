/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : protocol_receive.h
 *  Description  : This file is for receive communication
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:10:43
 *  LastEditTime : 2023-08-09 00:24:32
 */


#ifndef COMM_RECEIVE_H
#define COMM_RECEIVE_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "main.h"
#include "periph_remote.h"
#define Const_Comm_Receive_BUFF_SIZE 4
// #define Const_Comm_Receive_BUFF_SIZE 4
#define SBUS_DATANUM 16
#define SBUS_RAWDATA 25
#define ROCKER_NUM 4
#define LEVER_NUM 4

typedef struct {
    // Function pointer for handling received data
    // 声明了一个函数指针 bus_func，它指向的函数接受一个 uint8_t * 参数并返回 uint32_t
    uint32_t (*bus_func)(uint8_t *buff);
} Comm_ReceiveEntry;


extern Comm_ReceiveEntry CommCmd_Receive[Const_Comm_Receive_BUFF_SIZE];
//void init_average_filter(void);

typedef enum {
  Remote_DisConnect = 0,
  Remote_Connect = 1,

} Remote_ConnectMode;

typedef enum {
  Lever_up = 0,
  Lever_middle = 1,
  Lever_down = 2

} Remote_LeverMode;

typedef struct {
  uint16_t ch[SBUS_DATANUM];
  uint8_t rx_buff[SBUS_RAWDATA];
  uint8_t state;

} Remote_RawDataTypeDef;


typedef struct {
    Remote_SwitchStateEnum s2_last_state;
    Remote_SwitchStateEnum s3_last_state;
} Remote_SwitchStateHistoryTypeDef;

typedef struct {
  int16_t ch[ROCKER_NUM];
  Remote_LeverMode s[LEVER_NUM];
  Remote_RawDataTypeDef rawdata;
  Remote_ConnectMode connect;
  Remote_SwitchStateHistoryTypeDef switch_history;
	
  int16_t spd_value;
  uint16_t init;

} Remote_DataTypeDef;


extern Comm_ReceiveEntry CommCmd_Receive[Const_Comm_Receive_BUFF_SIZE];
Remote_DataTypeDef *Remote_GetRemotePtr(void);
int16_t Remote_RockerOffset(uint16_t data);
Remote_LeverMode Remote_LeverOffset(uint16_t data);



#endif

#ifdef __cplusplus
}
#endif
