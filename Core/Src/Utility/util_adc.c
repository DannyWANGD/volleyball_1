/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : util_adc.c
 *  Description  : This file contains the functions of ADC
 *  LastEditors  : Polaris
 *  Date         : 2023-01-19 04:07:35
 *  LastEditTime : 2023-08-24 00:11:24
 */


#include "util_adc.h"
#include "lib_buff.h"
#include "stdlib.h"
#include "string.h"

Adc_AdcTypeDef **adc_buff;
uint8_t channel_num = 0;

/**
 * @brief        : ADC initial
 * @param         [ADC_HandleTypeDef*] adc
 * @param         [uint8_t] channel_num
 * @return        [type]
 */
int Adc_Init() {
    int res = 0;
    res += Adc_ChannelInit("TEMPSENSOR", &hadc1, ADC_CHANNEL_TEMPSENSOR, TEMP, (3.3f / 4096), 0.1, 1);
    res += Adc_ChannelInit("VREFINT", &hadc1, ADC_CHANNEL_VREFINT, SINGLE, (3.3f / 4096), 0.1, 2);
    res += Adc_ChannelInit("VBat", &hadc1, ADC_CHANNEL_VBAT, SINGLE, (3.3f / 4096), 0.1, 3);
    if (res != ADC_OK) return ADC_INIT_ERROR;
    
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)malloc(channel_num * sizeof(uint32_t)), channel_num);	//start Adc DMA,Get the first group data.
    res = Adc_SetDataPtr();
    return res;
}


/** 
  * @brief         Set the sampling data pointer of each channel
  * @retval        res
 */
int Adc_SetDataPtr() {
    if (channel_num == 0) return ADC_FAIL_SET_PTR;

    for (int i = 0; i < channel_num; i++) {
        Adc_AdcTypeDef *cur = adc_buff[i];
        cur->data_ptr = Adc_GetDmaDstAddress(cur->adc);
    }
    
    return ADC_OK;
}


/**
 * @brief        : ADC TypeDef initial
 * @param         [Adc_AdcTypeDef*] s_adc
 * @param         [ADC_HandleTypeDef*] h_adc
 * @param         [uint32_t] channel
 * @param         [Adc_AdcTypeEnum] type
 * @param         [float] gain
 * @return        [type]
 */
int Adc_ChannelInit(const char *name, ADC_HandleTypeDef* h_adc, uint32_t channel, 
                     Adc_AdcTypeEnum type, float gain, float fil_p, uint32_t rank) {
    if (channel_num == 0) {
        adc_buff = (Adc_AdcTypeDef **)malloc(sizeof(Adc_AdcTypeDef *));
    }
    else {
        Adc_AdcTypeDef **new_buff = (Adc_AdcTypeDef **)realloc(adc_buff, (channel_num + 1) * sizeof(Adc_AdcTypeDef *));
        if (new_buff == NULL) return ADC_INIT_ERROR;      

        adc_buff = new_buff;
    }
    channel_num++;
    adc_buff[channel_num - 1] = (Adc_AdcTypeDef *)malloc(sizeof(Adc_AdcTypeDef));
    if (adc_buff[channel_num - 1] == NULL) return ADC_INIT_ERROR;
    
    Adc_AdcTypeDef *cur = adc_buff[channel_num - 1];
    
    cur->adc_channel.Channel = channel;
    cur->adc_channel.Rank = rank;
    cur->adc_channel.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    if (HAL_ADC_ConfigChannel(h_adc, &(cur->adc_channel)) != HAL_OK) {
        Error_Handler();
        return ADC_INIT_ERROR;
    }

    cur->name = malloc(strlen(name));
    if (cur->name == NULL) return ADC_NAME_NULL;
    
    memcpy(cur->name, name, strlen(name));
    
	cur->adc = h_adc;
    cur->gain = gain;
    cur->type = type;

    Filter_LowPassInit(fil_p, &(cur->fil_param));
    return ADC_OK;
}


/**
 * @brief        : Adc Refresh data and restart dma
 * @param         [Adc_AdcTypeDef*] s_adc
 * @return        [type]
 */
int Adc_RefreshData() {
    if (channel_num == 0) return ADC_NO_DATA_REFRESH;
    
    for (int i = 0; i < channel_num; i++) {
        Adc_AdcTypeDef *cur = adc_buff[i];
        uint32_t *adc_temp = cur->data_ptr + (cur->adc_channel.Rank - 1);

		float val_temp = *adc_temp;
			
        if (cur->type == SINGLE) {
            cur->value = val_temp * cur->gain;
        }
        else if (cur->type == DIFFER) {
            cur->value = val_temp * cur->gain / 2.0f;
        }
        else if (cur->type == TEMP) {
            cur->value = (val_temp * cur->gain - 1.43f) / 4.3f + 25.0f;
        }
        else {
            Adc_ErrorHandler(ADC_UNKNOW_TYPE);
            return ADC_UNKNOW_TYPE;
        }

        cur->filter_result = Filter_LowPass(cur->value, &(cur->fil_param), &(cur->fil));
    }
    return ADC_OK;
}


/**
 * @brief        : Get Adc real value
 * @param         [Adc_AdcTypeDef*] s_adc
 * @return        [type]
 */ 
float Adc_GetValue(const char *name) {
    if (channel_num == 0) return 0;

    for (int i = 0; i < channel_num; i++) {
        Adc_AdcTypeDef *cur = adc_buff[i];
        int ret = strncmp(cur->name, name, strlen(name));
        if (ret == 0) return cur->filter_result;
    }
    return 0;
}


/**
 * @brief        : Error Handler
 * @param         [int] ret
 * @return        [type]
 */
void Adc_ErrorHandler(int ret) {
    UNUSED(ret);
    return;
}


/**
 * @brief        : Get ADC DMA dat array ptr 
 * @param         [Adc_AdcTypeDef*] s_adc
 * @return        [type]
 */
static uint32_t *Adc_GetDmaDstAddress(ADC_HandleTypeDef* adc) {
    if (adc->DMA_Handle->Instance->M0AR == NULL) {
        Adc_ErrorHandler(ADC_LOSE_BUFF_ADR);
        return NULL;
    }
    return (uint32_t *)(adc->DMA_Handle->Instance->M0AR);
}
