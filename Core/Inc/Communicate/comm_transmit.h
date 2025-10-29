/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : protocol_transmit.h
 *  Description  : This file is for transmit communication
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 03:18:52
 *  LastEditTime : 2023-08-15 23:17:31
 */


#ifndef COMM_TRANSMIT_H
#define COMM_TRANSMIT_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "main.h"

#define Const_Comm_Transmit_BUFF_SIZE 2

typedef struct {
    uint32_t (*bus_func)(uint8_t *buff);
} Comm_SendEntry;


extern Comm_SendEntry CommCmd_Send[Const_Comm_Transmit_BUFF_SIZE];



#endif

#ifdef __cplusplus
}
#endif
