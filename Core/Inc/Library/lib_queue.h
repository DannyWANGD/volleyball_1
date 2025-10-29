/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : lib_queue.h
 *  Description  : This file contains the functions of queue 
 *  LastEditors  : Polaris
 *  Date         : 2022-04-29 13:12:20
 *  LastEditTime : 2023-02-01 02:51:20
 */


#ifndef QUEUE_LIB_H
#define QUEUE_LIB_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "main.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

#define QUEUE_ENTER_CRITICAL __disable_irq
#define QUEUE_EXIT_CRITICAL __enable_irq
#define QUEUE_GET_CPU_SR __get_PRIMASK
#define QUEUE_RESTORE_CPU_SR(CPU_SR) __set_PRIMASK(CPU_SR)
#define QUEUE_CPU_SR_TYPE register unsigned long


typedef struct {
  char *p_start_addr;           //!< QUEUE Memory Pool Start Address
  char *p_end_addr;             //!< QUEUE Memory Pool End Address
  int free_num;                 //!< The remain capacity of QUEUE
  int used_num;                 //!< The number of elements in QUEUE
  char *p_read_addr;            //!< QUEUE Data Read Index Pointer
  char *p_write_addr;           //!< QUEUE Data Write Index Pointer
} Queue_CharTypeDef;

typedef struct {
  char *p_start_addr;           //!< QUEUE Memory Pool Start Address
  char *p_end_addr;             //!< QUEUE Memory Pool End Address
  int free_num;                 //!< The remain capacity of QUEUE
  int used_num;                 //!< The number of elements in QUEUE
  int unit_size;                //!< QUEUE Element Size(Unit: Byte)
  char *p_read_addr;            //!< QUEUE Data Read Index Pointer
  char *p_write_addr;           //!< QUEUE Data Write Index Pointer
} Queue_TypeDef;


Queue_CharTypeDef *Queue_CharCreate(int uint_cnt);
void Queue_CharDestroy(Queue_CharTypeDef * p_queue);
int Queue_CharInit(Queue_CharTypeDef * p_queue, void *p_base_addr, int uint_cnt);
int Queue_CharPut(Queue_CharTypeDef * p_queue, char element);
int Queue_CharPuts(Queue_CharTypeDef * p_queue, char *p_source, int len);
int Queue_CharPutsNoProtect(Queue_CharTypeDef * p_queue, char *p_source, int len);
char Queue_CharGet(Queue_CharTypeDef * p_queue);
int Queue_CharGets(Queue_CharTypeDef * p_queue, char *p_dest, int len);
int Queue_CharGetsNoProtect(Queue_CharTypeDef * p_queue, char *p_dest, int len);
char Queue_CharPreread(Queue_CharTypeDef * p_queue, int offset);
int Queue_CharPrereads(Queue_CharTypeDef * p_queue, char *p_dest, int offset, int len);
char Queue_CharIsEmpty(Queue_CharTypeDef * p_queue);
char Queue_CharIsFull(Queue_CharTypeDef * p_queue);
int Queue_CharUsed(Queue_CharTypeDef * p_queue);
int Queue_CharFree(Queue_CharTypeDef * p_queue);
void Queue_CharFlush(Queue_CharTypeDef * p_queue);
int Queue_CharDiscard(Queue_CharTypeDef * p_queue, int len);
Queue_TypeDef *Queue_Create(char unit_size, int unit_cnt);
void Queue_Destory(Queue_TypeDef * p_queue);
int Queue_Init(Queue_TypeDef * p_queue, void *p_base_addr, char unit_size, int unit_cnt);
int Queue_Put(Queue_TypeDef * p_queue, void *p_element);
int Queue_PutNoProtect(Queue_TypeDef * p_queue, void *p_element);
int Queue_Get(Queue_TypeDef * p_queue, void *p_element);
int Queue_GetNoProtect(Queue_TypeDef * p_queue, void *p_element);
int Queue_Preread(Queue_TypeDef * p_queue, char offset, void *p_element);
int Queue_IsEmpty(Queue_TypeDef * p_queue);
int Queue_IsFull(Queue_TypeDef * p_queue);
int Queue_Used(Queue_TypeDef * p_queue);
int Queue_Free(Queue_TypeDef * p_queue);
int Queue_Flush(Queue_TypeDef * p_queue);

#endif

#ifdef __cplusplus
}
#endif
