/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : sys_dwt.h
 *  Description  : Data Watch point and Trace
 *  LastEditors  : Polaris
 *  Date         : 2023-02-10 13:25:31
 *  LastEditTime : 2023-08-13 15:08:40
 */


#ifndef DWT_LIB_H
#define DWT_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct {
    uint32_t s;
    uint32_t ms;
    uint16_t us;
    uint32_t ms_tick;
} DWT_TimeTypeDef;

typedef struct {
    DWT_TimeTypeDef SysTime;
    
    uint32_t CPU_FREQ_Hz, CPU_FREQ_Hz_ms, CPU_FREQ_Hz_us;
    uint32_t CYCCNT_RountCount;
    uint32_t CYCCNT_LAST;
    uint64_t CYCCNT64;
} DWT_DataTypeDef;

void DWT_Init(uint32_t CPU_Freq_mHz);
DWT_DataTypeDef* DWT_GetDWTDataPtr(void);
float DWT_GetDeltaT(uint32_t *cnt_last);
double DWT_GetDeltaT64(uint32_t *cnt_last);
float DWT_GetDeltaTWithOutUpdate(uint32_t *cnt_last);
float DWT_GetTimeline_s(void);
float DWT_GetTimeline_ms(void);
uint64_t DWT_GetTimeline_us(void);
void DWT_Delay(float Delay);
void DWT_Delayms(float Delay);
void DWT_Delayus(float Delay);
void DWT_SysTimeUpdate(void);
void DWT_CNT_Update(void);

#endif

#ifdef __cplusplus
}

#endif
