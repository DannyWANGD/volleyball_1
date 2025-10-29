/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : lib_gif.h
 *  Description  : This document contains the OLED gif library
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-09-24 01:47:41
 */


#ifndef GIF_LIB_H
#define GIF_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern const unsigned char GIF_ROCKET[16][1024];
extern const unsigned char GIF_FIRE[16][1024];
extern float lcd_cube[8][3];
extern uint8_t lcd_line_id[24];
extern const uint8_t PIC_BEAR[32768];

#ifdef __cplusplus
}
#endif

#endif
