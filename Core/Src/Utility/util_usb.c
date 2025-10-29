/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : util_usb.c
 *  Description  : This file contains the functions of USB
 *  LastEditors  : Polaris
 *  Date         : 2022-04-29 14:21:52
 *  LastEditTime : 2023-08-12 13:45:42
 */


#include "util_usb.h"
#include "usbd_cdc_if.h"
#include "module_communicate.h"
#include "util_uart.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdarg.h"
#include "sys_const.h"

static uint8_t usb_buf[512];


/**
 * @brief        : usb printf 
 * @param         [char] *fmt
 * @return        [type]
 */
void Usb_Printf(const char *fmt,...) {
    static va_list ap;
    uint16_t len = 0;

    va_start(ap, fmt);

    len = vsprintf((char *)usb_buf, fmt, ap);

    va_end(ap);

    CDC_Transmit_FS(usb_buf, len);
}


/**
 * @brief        : usb printf 
 * @param         [char] *fmt
 * @return        [type]
 */
void Usb_SendBuff(uint8_t buff[], uint32_t len) {
   CDC_Transmit_FS(buff, len);
}


/**
 * @brief        : usb printf 
 * @param         [char] *fmt
 * @return        [type]
 */
void Usb_ReceiveCallback(uint8_t* Buf, uint32_t *Len) {
    Protocol_DecodeData(Buf, *Len);
    Uart_SendMessage_IT(Const_Remote_UART_HANDLER, Buf, *Len);
}
