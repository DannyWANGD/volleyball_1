/*
 *  Project      : Polaris Robot
 * 
 *  FilePath     : alg_math.h
 *  Description  : This file contains the math calculate tools
 *  LastEditors  : Polaris
 *  Date         : 2022-04-16 22:53:30
 *  LastEditTime : 2023-08-15 19:22:37
 */


#ifndef MATH_ALG_H
#define MATH_ALG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "arm_math.h"

#define mat                 arm_matrix_instance_f32
#define Matrix_Init         arm_mat_init_f32
#define Matrix_Add          arm_mat_add_f32
#define Matrix_Subtract     arm_mat_sub_f32
#define Matrix_Multiply     arm_mat_mult_f32
#define Matrix_Transpose    arm_mat_trans_f32
#define Matrix_Inverse      arm_mat_inverse_f32


/**
  * @brief      Limit function
  * @param      input :Limited value
  * @param      max :Max limite value
  * @retval     NULL
  */
#define LimitMax(input, max) {      \
        if (input > max) {          \
            input = max;            \
        }                           \
        else if (input < -max) {    \
            input = -max;           \
        }                           \
    }


/**
  * @brief      Maximum and minimum limiting
  * @param      input :Limited value
  * @param      max :Max limite value
  * @param      min :Min limite value
  * @retval     NULL
  */
#define LimitMaxMin(input, max, min) {      \
        if (input > max) {                  \
            input = max;                    \
        }                                   \
        else if (input < min) {             \
            input = min;                    \
        }                                   \
    }

#define GetMaxandMinRange(input, max, min) {    \
    if (input >= max) {                         \
        max = input;                            \
    }                                           \
    else if (input <= min) {                    \
        min = input;                            \
    }                                           \
}

typedef struct {
    float acc;
    float dec;
} Math_SlopeParamTypeDef;


#define Math_Rad2Angle(rad) ((rad) * 180.0f / PI)
#define Math_Angle2Rad(ang) ((ang) * PI / 180.0f)
float Math_Fal(float e, float alpha, float zeta);
int16_t Math_Fsg(float x, float d);
int16_t Math_Sign(float Input);
void Math_InitSlopeParam(Math_SlopeParamTypeDef* pparam, float acc, float dec);
float Math_CalcSlopeRef(float rawref, float targetref, Math_SlopeParamTypeDef* pparam);
float Math_Differential(float arr[], uint8_t order, float dt);
float Math_InvSqrt(float x);
int sign(double x);
double rad2deg(double rad);
double deg2rad(double deg);
#ifdef __cplusplus
}
#endif

#endif
