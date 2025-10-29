/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : util_adc.h
 *  Description  : This file contains the functions of ADC
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-01-23 05:03:08
 */


#ifndef ADC_UTIL_H
#define ADC_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "adc.h"
#include "alg_filter.h"

#define ADC_OK              0
#define ADC_INIT_ERROR      -1
#define ADC_LOSE_BUFF_ADR   -2
#define ADC_FAIL_SET_PTR    -3
#define ADC_NAME_NULL       -4
#define ADC_UNKNOW_TYPE     -5
#define ADC_NO_DATA_REFRESH -6

typedef enum {
    SINGLE  = 0,
    DIFFER  = 1,
    TEMP    = 2
} Adc_AdcTypeEnum;

typedef struct {
    char *name;
    
    ADC_HandleTypeDef* adc; 
    ADC_ChannelConfTypeDef adc_channel;

    float gain;
    uint32_t *data_ptr;
    float value;
    float filter_result;

    Filter_LowPassParamTypeDef fil_param;
    Filter_LowPassTypeDef fil;
    Adc_AdcTypeEnum type;
} Adc_AdcTypeDef;


int Adc_Init(void);
int Adc_SetDataPtr(void);
int Adc_ChannelInit(const char *nam, ADC_HandleTypeDef* h_adc, uint32_t channel, 
                     Adc_AdcTypeEnum type, float gain, float fil_p, uint32_t rank);
static uint32_t *Adc_GetDmaDstAddress(ADC_HandleTypeDef* adc);
int Adc_RefreshData(void);
float Adc_GetValue(const char *name);
void Adc_ErrorHandler(int ret);
    
#endif

#ifdef __cplusplus
}
#endif
