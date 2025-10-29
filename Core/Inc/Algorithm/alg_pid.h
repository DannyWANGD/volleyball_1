/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : alg_pid.h
 *  Description  : This file contains PID algorithm function
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-02-08 10:51:07
 */


#ifndef PID_ALG_H
#define PID_ALG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "alg_math.h"
#include "alg_filter.h"


typedef enum {
    PID_POSITION = 0u,
    PID_DELTA = 1u
} PID_ModeEnum;

// PID 结构体，包含 PID 控制器的参考值、反馈值、误差、输出等信息
typedef struct {
    float ref;              // 参考值
    float fdb;              // 反馈值
    float err[3];           // 误差
    float err_lim;          // 积分限幅
    float err_fdf[3];       // 前馈信息
    float out_fdf;          // 前馈输出
    float sum;
    float output;

    Filter_LowPassTypeDef d_fil;        // 低通滤波器(把误差差分或测量量通过 d_fil 低通滤波，以减少噪声引起的抖动)
    Filter_LowPassTypeDef delta_fil;    // 低通滤波器(会对Δ值做滤波以平滑控制增量)

    Filter_LowPassTypeDef kf1_fil;      // 前馈滤波器一阶
    Filter_LowPassTypeDef kf2_fil;      // 前馈滤波器二阶
    float err_watch;

} PID_PIDTypeDef;

typedef struct {
    PID_ModeEnum pid_mode;

    // PID
    float kp;                   // pid param
    float ki;
    float kd;
    
    float sum_max;              // sum limit    
    float output_max;           // out limit
    
    Filter_LowPassParamTypeDef d_fil_param;         // Kd filter
    Filter_LowPassParamTypeDef delta_fil_param;     // delta filter ref filter

    // Feedforward
    float kf_1;                                     // feedforward param
    float kf_2;         
    
    Filter_LowPassParamTypeDef kf1_fil_param;      // feed forawrd filter
    Filter_LowPassParamTypeDef kf2_fil_param;
} PID_PIDParamTypeDef;

void PID_InitPIDParam(PID_PIDParamTypeDef* pparam, float kp, float ki, float kd, float sum_max, float output_max, 
                      float kd_fil_param, float delta_fil_param, float kf_1, float kf_2, float kf1_fil_param, 
                      float kf2_fil_param,PID_ModeEnum pid_mode);
float PID_GetPIDRef(PID_PIDTypeDef* pid);
void PID_SetPIDRef(PID_PIDTypeDef* pid, float ref);
void PID_AddPIDRef(PID_PIDTypeDef* pid, float inc);
float PID_GetPIDFdb(PID_PIDTypeDef* pid);
void PID_SetPIDFdb(PID_PIDTypeDef* pid, float fdb);
float PID_GetPIDOutput(PID_PIDTypeDef* pid);
void PID_ClearPID(PID_PIDTypeDef* pid);
void PID_CalcPID(PID_PIDTypeDef* pid, PID_PIDParamTypeDef* para);

#ifdef __cplusplus
}
#endif

#endif
