/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : util_timer.h
 *  Description  : This file contains the functions of Timer
 *  LastEditors  : Polaris
 *  Date         : 2023-08-13 06:18:56
 *  LastEditTime : 2023-08-13 06:53:54
 */


#include "tim.h"


#ifndef UTIL_TIMER_H
#define UTIL_TIMER_H


#ifdef __cplusplus
extern "C" {
#endif 

void Timer_StartTimer(TIM_HandleTypeDef* htim);

#ifdef __cplusplus
}
#endif


#endif