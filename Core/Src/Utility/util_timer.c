/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : util_timer.c
 *  Description  : This file contains the functions of Timer
 *  LastEditors  : Polaris
 *  Date         : 2023-08-13 06:18:48
 *  LastEditTime : 2023-08-24 00:02:22
 */


#include "util_timer.h"
#include "main.h"


/********** VOLATILE USER CODE **********/

void Timer_StartTimer(TIM_HandleTypeDef* htim) {
    HAL_TIM_Base_Start_IT(htim);
}

