/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : lib_font.h
 *  Description  : TThis document contains the OLED font library
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-10-20 18:44:30
 */


#ifndef FONT_LIB_H
#define FONT_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define LCD_FONT_12_CHAR_WIDTH   6
#define LCD_FONT_12_CHAR_HEIGHT  12
#define LCD_FONT_12_CHAR_SIZE    12

#define LCD_FONT_16_CHAR_WIDTH   8
#define LCD_FONT_16_CHAR_HEIGHT  16
#define LCD_FONT_16_CHAR_SIZE    16

#define LCD_FONT_24_CHAR_WIDTH   12
#define LCD_FONT_24_CHAR_HEIGHT  24
#define LCD_FONT_24_CHAR_SIZE    48

#define LCD_FONT_32_CHAR_WIDTH   16
#define LCD_FONT_32_CHAR_HEIGHT  32
#define LCD_FONT_32_CHAR_SIZE    64

typedef struct {
    uint8_t lcd_font[95][LCD_FONT_32_CHAR_SIZE];
    uint16_t width;
    uint16_t hight;
    uint16_t size;
} LCD_FONTTypeDef;

extern const unsigned char asc2_1206[95][12];
extern const unsigned char OLED_LOGO_RM[128][8];
extern const unsigned char OLED_BEAR[1024];
extern const unsigned char OLED_SUR[1024];
extern const unsigned char OLED_SUB[1024];

extern const LCD_FONTTypeDef LCD_Font[4];

#ifdef __cplusplus
}
#endif

#endif
