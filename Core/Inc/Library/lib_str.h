/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : lib_str.h
 *  Description  : This file contains information about string handling functions
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-08-15 00:58:38
 */


#ifndef STR_LIB_H
#define STR_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdarg.h"

char *Str_Itoa(int value, char *string);
int Str_Atoi(const char *str);
void Str_HexToAscii(uint8_t *src, char *dest, int len);
char *Str_RemoveCRLF(char *psz, uint32_t len);
char *Str_FindSpace(char *buf);

#ifdef __cplusplus
}
#endif

#endif
