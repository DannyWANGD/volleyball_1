/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : periph_remote.c
 *  Description  : This file contains RoboMaster remote relevant function
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:07
 *  LastEditTime : 2023-10-28 18:25:09
 */


#include "periph_remote.h"
#include "sys_dwt.h"
#include "sys_const.h"

Remote_RemoteDataTypeDef Remote_RemoteData;

/**
  * @brief      Initialize remote control
  * @param      NULL
  * @retval     NULL
  */
void Remote_InitRemote() {
    Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr();

    Remote_ResetRemoteData();
    rc->update_dt = 0;
	  rc->last_update_tick = 0;
    Uart_InitUartDMA(Const_Remote_UART_HANDLER);
    Uart_ReceiveDMA(Const_Remote_UART_HANDLER, rc->Remote_RxData, Const_Remote_RX_BUFF_LEN);
}


/**
  * @brief      Gets the pointer of the remote control object
  * @param      NULL
  * @retval     Pointer to remote control object
  */
Remote_RemoteDataTypeDef* Remote_GetRemoteDataPtr() {
    return &Remote_RemoteData;
}


/**
  * @brief      Switch position state
  * @param      sw: Original switch value
  * @retval     Switch position status
  */
Remote_SwitchStateEnum Remote_ToSwitchState(uint8_t sw) {
    return (Remote_SwitchStateEnum) sw;
}


/**
  * @brief      Judge whether the remote control is offline
  * @param      rc: pointer to remote control object
  * @retval     Offline or not (1 is yes, 0 is no)
  */
uint8_t Remote_IsRemoteOffline() {
    Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr();

    if (DWT_GetDeltaTWithOutUpdate(&rc->last_update_tick) > Const_Remote_REMOTE_OFFLINE_TIME)
        rc->state = Remote_STATE_LOST;
    return (rc->state != Remote_STATE_CONNECTED);
}


/**
  * @brief      Judge whether the remote control data is wrong
  * @param      rc: pointer to remote control object
  * @retval     Error or not (1 is yes, 0 is no)
  */
uint8_t Remote_IsRemoteError() {
    Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr();
    const uint8_t REMOTE_OK      = 0;
    const uint8_t REMOTE_ERROR   = 1;

    for (int i = 0; i < 5; ++i)
        if (abs(rc->remote.ch[i]) > Const_Remote_CHANNEL_ERROR_LIMIT) {
            return REMOTE_ERROR;
        }
    for (int i = 0; i < 2; ++i)
        if (rc->remote.s[i] == Remote_SWITCH_NULL) {
            return REMOTE_ERROR;
        }
    return REMOTE_OK;
}





/**
  * @brief      Remote control decoding function
  * @param      rc: The pointer points to the remote control data object
  * @param      buff: data buff
  * @param      rxdatalen: Data length
  * @retval     NULL
  */
void Remote_DecodeRemoteData(uint8_t* buff, int rxdatalen) {
    Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr();
    if (rxdatalen != Const_Remote_RX_FRAME_LEN) return;
    
    rc->update_dt       = DWT_GetDeltaT(&rc->last_update_tick);   
    rc->remote.ch[0]    = Remote_CancelChannelOffset(((uint16_t)buff[0] | (uint16_t)buff[1] << 8) & 0x07FF);
    rc->remote.ch[1]    = Remote_CancelChannelOffset(((uint16_t)buff[1] >> 3 | (uint16_t)buff[2] << 5) & 0x07FF);
    rc->remote.ch[2]    = Remote_CancelChannelOffset(((uint16_t)buff[2] >> 6 | (uint16_t)buff[3] << 2 | (uint16_t)buff[4] << 10) & 0x07FF);
    rc->remote.ch[3]    = Remote_CancelChannelOffset(((uint16_t)buff[4] >> 1 | (uint16_t)buff[5] << 7) & 0x07FF);
    rc->remote.s[0]     = Remote_ToSwitchState((buff[5] >> 6) & 0x03);
    rc->remote.s[1]     = Remote_ToSwitchState((buff[5] >> 4) & 0x03);
   
    /*buff[16],buff[17] Is Dial wheel*/
    rc->remote.ch[4]    = Remote_CancelChannelOffset(((uint16_t)buff[16] | (uint16_t)buff[17] << 8) & 0x07FF);

    if (rc->remote.ch[4] == -1024) 
        rc->remote.ch[4] = 0;

    if (Remote_IsRemoteError()) {
        rc->state       = Remote_STATE_ERROR;
        Remote_ResetRemoteData();
        return;
    }
    rc->state           = Remote_STATE_CONNECTED;
}


/**
  * @brief      Initialize remote control data
  * @param      rc: Pointer to remote control object
  * @retval     NULL
  */
void Remote_ResetRemoteData() {
    Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr();
    for (int i = 0; i < 5; ++i)
        rc->remote.ch[i] = 0;
    for (int i = 0; i < 2; ++i)
        rc->remote.s[i] = Remote_ToSwitchState(0);
    
}


/**
  * @brief      Remove remote control offset
  * @param      ch: Original channel value
  * @retval     True value
  */
int16_t Remote_CancelChannelOffset(uint16_t ch) {
    return (int16_t) ch - Const_Remote_CHANNEL_VALUE_OFFSET;
}


/**
  * @brief      Remote control receiving callback function
  * @param      huart: Pointer to UART handle
  * @retval     NULL
  */
void Remote_RXCallback(UART_HandleTypeDef* huart) {
    Remote_RemoteDataTypeDef *rc = Remote_GetRemoteDataPtr();
    __HAL_DMA_DISABLE(huart->hdmarx);

    int rxdatalen = Const_Remote_RX_BUFF_LEN - Uart_DMACurrentDataCounter(huart->hdmarx->Instance);
    
    Remote_DecodeRemoteData(rc->Remote_RxData, rxdatalen);


    __HAL_DMA_SET_COUNTER(huart->hdmarx, Const_Remote_RX_BUFF_LEN);
    __HAL_DMA_ENABLE(huart->hdmarx);
}
