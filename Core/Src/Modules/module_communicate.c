 #include "module_communicate.h"
  #include "module_platform.h"
 #include "cmsis_os.h"
 #include "periph_remote.h"
 #include "lib_buff.h"
 #include "util_uart.h"
 #include "periph_motor.h"
 #include "util_usb.h"
 #include "sys_dwt.h"
 #include "sys_const.h"
 #include "app_remote.h"
 #include "app_gimbal.h"
 
static uint32_t _send_Remote_data(uint8_t *buff);
static uint32_t _send_Platform_data(uint8_t *buff);
static uint32_t _send_Gimbal_data(uint8_t *buff);


Protocol_DataTypeDef Protocol_Data;

static uint32_t _set_Platform_Data_(uint8_t *buff) ;
static uint32_t _set_Gimbal_Data_(uint8_t *buff) ;


void Protocol_InitProtocol() {

   Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    Protocol_ResetProtocolData();
    Uart_InitUartDMA(Const_Communicate_UART_HANDLER);
    Uart_ReceiveDMA(Const_Communicate_UART_HANDLER, buscomm->uart_watchBuff, Const_Com_RX_BUFF_LEN);
}

Protocol_DataTypeDef* Protocol_GetBusDataPtr() {
    return &Protocol_Data;
}

uint8_t Protocol_IsProtocolOffline() {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    if (DWT_GetDeltaTWithOutUpdate(&buscomm->last_rx_tick) > Const_Comm_OFFLINE_TIME) {
        buscomm->state = Protocol_STATE_LOST;
    }
    return buscomm->state != Protocol_STATE_CONNECTED;
}

void Protocol_ResetProtocolData() {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    buscomm->rx_dt = DWT_GetDeltaT(&buscomm->last_rx_tick);
    buscomm->tx_dt = DWT_GetDeltaT(&buscomm->last_tx_tick);
}



void Protocol_SendProtocolData() {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();

    buscomm->tx_dt = DWT_GetDeltaT(&buscomm->last_tx_tick);
    uint32_t len;   
    // to nuc
    len = 4;
        buscomm->usb_sendBuff[0] = 0X5A;
        buscomm->usb_sendBuff[1] = 0XA5;
        for (int i = 0; i < Const_Protocol_Transmit_BUFF_SIZE; i++) {
            if (ProtocolCmd_Send[i].bus_func != NULL) {
                len += ProtocolCmd_Send[i].bus_func(buscomm->usb_sendBuff + len);
            }
        }
        buscomm->usb_sendBuff[2] = (uint8_t)(len & 0xff);
        buscomm->usb_sendBuff[3] = (uint8_t)((len & 0xff00) >> 8);
        buscomm->usb_sendBuff[len] = 0X7A;
        buscomm->usb_sendBuff[len + 1 ] = 0XA7;
		//Uart_SendMessage(Const_Communicate_UART_HANDLER, buscomm->usb_sendBuff, len + 2, 10);
        // Usb_SendBuff(buscomm->usb_sendBuff, 64);
    }

static uint8_t Protocol_MergeAndverify(uint8_t buff[], uint32_t rxdatalen) {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    static uint64_t pack_flag = 0;
    
    //if (rxdatalen != 16) return 0;

    if ((buff[0] == 0x5A) && (buff[1] == 0xA5) ) {
			memcpy(buscomm->uart_watchBuff , buff, rxdatalen);
            return 0;
        }
        else {
            return 1;
        } 
	return 0;
}

uint8_t lkn[100];

void Protocol_DecodeData(uint8_t buff[], uint32_t rxdatalen) {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();

    Protocol_MergeAndverify(buff, rxdatalen); 
    buscomm->rx_dt = DWT_GetDeltaT(&buscomm->last_rx_tick);
    uint32_t len = 4;
    {
        for (int i = 0; i < Const_Protocol_Receive_BUFF_SIZE; i++) {
            if (ProtocolCmd_Receive[i].bus_func != NULL) {
				        len += ProtocolCmd_Receive[i].bus_func(buscomm->uart_watchBuff + len);
            }
        }
    }
}

Protocol_SendEntry ProtocolCmd_Send[Const_Protocol_Transmit_BUFF_SIZE] = {

 //   { _send_Gimbal_data},
	  { _send_Remote_data },
   { _send_Platform_data},
};
Protocol_ReceiveEntry ProtocolCmd_Receive[Const_Protocol_Receive_BUFF_SIZE] = {
     {&_set_Platform_Data_        },
	 {&_set_Gimbal_Data_        }
};
 static uint32_t _send_Remote_data(uint8_t *buff) {
    Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr();

    buff[0] = (uint8_t)(rc->remote.s[0]) | ((uint8_t)(rc->remote.s[1]) << 4);
    buff[1] = (uint8_t)(rc->remote.ch[0] >> 8);
    buff[2] = (uint8_t)(rc->remote.ch[0]);
    buff[3] = (uint8_t)(rc->remote.ch[1] >> 8);
    buff[4] = (uint8_t)(rc->remote.ch[1]);
    buff[5] = (uint8_t)(rc->remote.ch[2] >> 8);
    buff[6] = (uint8_t)(rc->remote.ch[2]);
    buff[7] = (uint8_t)(rc->remote.ch[3] >> 8);
    buff[8] = (uint8_t)(rc->remote.ch[3]);
    buff[9] = (uint8_t)(rc->remote.ch[4] >> 8);
    buff[10] =(uint8_t)(rc->remote.ch[4]);

    return 11;
}
static uint32_t _send_Platform_data(uint8_t *buff) {
    
Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
 
     float2buff( Platform->fdb.front_pitch, buff);
     float2buff( Platform->fdb.left_pitch, buff+4);
     float2buff( Platform->fdb.right_pitch, buff+8);
     float2buff( Platform->fdb.front_yaw, buff+12);
     float2buff( Platform->fdb.left_yaw, buff+16);
     float2buff( Platform->fdb.right_yaw, buff+20);
     
    return 24;
}

static uint32_t _send_Gimbal_data(uint8_t *buff) {
    
Gimbal_DataTypeDef *gimbal = Gimbal_GetDataPtr();

  
     float2buff( gimbal->Yaw_fdb, buff);
     float2buff( gimbal->Pitch_fdb, buff+4);


    return 8;
}

static uint32_t _set_Platform_Data_(uint8_t *buff) {
	
   Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr(); 
	 Platform_DataTypeDef *Platform = Platform_GetPlatformPtr();
	

if(rc->remote.s[0] == Remote_SWITCH_DOWN && rc->remote.s[1] == Remote_SWITCH_UP ){
	//Platform_Set_Cal_LTO_PC(buff2float(buff),buff2float(buff+4),buff2float(buff+8),buff2float(buff+12));
}

return 16;

}

float yaw, pitch;
static uint32_t _set_Gimbal_Data_(uint8_t *buff) 
{
	
	Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr(); 
	Gimbal_DataTypeDef *gimbal = Gimbal_GetDataPtr();
	
	gimbal->Pole_Yaw_ref = buff2float(buff);
	gimbal->Pole_Pitch_ref = buff2float(buff + 4);

	Gimbal_SetPos();
	
	return 8;
}

void Communicate_RXCallback(UART_HandleTypeDef* huart) {
    Protocol_DataTypeDef *com = Protocol_GetBusDataPtr();

    /* clear DMA transfer complete flag */
    __HAL_DMA_DISABLE(huart->hdmarx);

    /* handle uart data from DMA */
    int rxdatalen = Const_Com_RX_BUFF_LEN - Uart_DMACurrentDataCounter(huart->hdmarx->Instance);
    
	
	Protocol_DecodeData(com->uart_watchBuff,rxdatalen);
	
    /* restart dma transmission */
    __HAL_DMA_SET_COUNTER(huart->hdmarx, Const_Com_RX_BUFF_LEN);
    __HAL_DMA_ENABLE(huart->hdmarx);
}


