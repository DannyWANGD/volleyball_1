#if defined(__GNUC__) || defined(__clang__) || defined(_MSC_VER)
#pragma once
#endif

#ifndef __PERIPH_DT35_H__
#define __PERIPH_DT35_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "util_uart.h"
#include "usart.h"

#define DT35_RAWDATA   17
#define DT35_DATALEN   13

typedef struct {
  float left_data, leftfront_data, rightfront_data;
	float distance;
	float w_angle;
  uint8_t init;
	uint8_t rx_len;
  uint8_t rxdata[DT35_RAWDATA];
	uint8_t state;
	
	float update_dt;
  uint32_t last_update_tick;
} DT35_DataTypeDef;

extern DT35_DataTypeDef DT35_Data;

DT35_DataTypeDef* DT35_GetPlatformPtr(void);
void DT35_Init(void);
void DT35_RXCallback(UART_HandleTypeDef *huart);
void DT35_DecodeData(uint8_t buf[], uint8_t len);

#ifdef __cplusplus
}
#endif

#endif
