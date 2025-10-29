/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : util_usb.h
 *  Description  : This file contains the functions of USB
 *  LastEditors  : Polaris
 *  Date         : 2022-04-29 14:22:07
 *  LastEditTime : 2023-01-27 16:25:16
 */


#ifndef USB_UTIL_H
#define USB_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "main.h"
#include "string.h"



void Usb_SendBuff(uint8_t buff[], uint32_t len);
void Usb_Printf(const char *fmt,...);
void Usb_ReceiveCallback(uint8_t* Buf, uint32_t *Len);

#endif

#ifdef __cplusplus
}
#endif
