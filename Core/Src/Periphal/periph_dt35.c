#include "periph_dt35.h"
#include "util_uart.h"
#include "usart.h"
#include "sys_const.h"
#include "sys_dwt.h"
#include "main.h"
#include "alg_math.h"
#include "lib_buff.h"
DT35_DataTypeDef DT35_Data;

DT35_DataTypeDef* DT35_GetBusDataPtr() {
    return &DT35_Data;
}

void DT35_Init(void)
{
 DT35_DataTypeDef *DT35 = &DT35_Data;  

  Uart_InitUartDMA(Const_DT35_UART_HANDLER);
  Uart_ReceiveDMA(Const_DT35_UART_HANDLER, DT35->rxdata, DT35_RAWDATA);

  DT35->left_data = 0;
  DT35->leftfront_data = 0;
  DT35->rightfront_data = 0;
	DT35->distance = 0;
	DT35->w_angle = 0;
  DT35->rx_len = 0;

  DT35->init = 1;
	DT35->update_dt = 0;
  DT35->last_update_tick = DWT_GetTimeline_us();
}

void DT35_DecodeData(uint8_t buf[], uint8_t len)
{
	 DT35_DataTypeDef *DT35 = DT35_GetBusDataPtr(); 
 //cm
    uint32_t temp;
    temp = (uint32_t)DT35->rxdata[3] << 24 | (uint32_t)DT35->rxdata[4] << 16 |
           (uint32_t)DT35->rxdata[5] << 8 | (uint32_t)DT35->rxdata[6];
    DT35->left_data = (*((float *)(&temp))) / 10.0f +1.5f;

    temp = (DT35->rxdata[7] << 24) | (DT35->rxdata[8] << 16) |
           (DT35->rxdata[9] << 8) | DT35->rxdata[10];
    DT35->rightfront_data = (*(float *)(&temp)) / 10.0f;

    temp = (uint32_t)DT35->rxdata[11] << 24 | (uint32_t)DT35->rxdata[12] << 16 |
	
           (uint32_t)DT35->rxdata[13] << 8 | (uint32_t)DT35->rxdata[14];
    DT35->leftfront_data = (*((float *)(&temp))) / 10.0f ;
	  
	  temp = (uint32_t)DT35->rxdata[15] << 24 | (uint32_t)DT35->rxdata[16] << 16 |
	
           (uint32_t)DT35->rxdata[17] << 8 | (uint32_t)DT35->rxdata[18];
    //DT35->rightfront_data = (*((float *)(&temp))) / 10.0f ;
	
	  if( (DT35->leftfront_data<400.0f) && (DT35->rightfront_data<400.0f)){
		DT35->state = 1;
		}else 
    {DT35->state = 0;}
		
		DT35->distance = (DT35->rightfront_data + DT35->leftfront_data)/2.0f + 30.0f;
		DT35->w_angle = rad2deg(atanf( (DT35->rightfront_data - DT35->leftfront_data)/31.458f ));
  
}

void DT35_RXCallback(UART_HandleTypeDef *huart)
{
	 DT35_DataTypeDef *DT35 = &DT35_Data; 
  /* clear DMA transfer complete flag */
  __HAL_DMA_DISABLE(huart->hdmarx);

  /* handle uart data from DMA */
  uint8_t rxdatalen = DT35_RAWDATA - Uart_DMACurrentDataCounter(huart->hdmarx->Instance);
	DT35->rx_len = rxdatalen;
  DT35_DecodeData(DT35->rxdata, rxdatalen);

  /* restart dma transmission */
  __HAL_DMA_SET_COUNTER(huart->hdmarx, DT35_RAWDATA);
  __HAL_DMA_ENABLE(huart->hdmarx);
}