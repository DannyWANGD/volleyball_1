/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : lib_music.c
 *  Description  : Music Mapping
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 14:59:00
 *  LastEditTime : 2023-08-03 14:34:28
 */


#include "lib_music.h"

const uint32_t BEEPER_MUSIC[MUSIC_BEEPER_MAXNUM][MUSIC_BEEPER_LENGTH] = {
{HIG_E, HIG_C, HIG_D, HIG_E, HIG_C, HIG_D, HIG_E, MID_E, 
 MID_F, MID_G, HIG_A, HIG_B, HIG_C, HIG_D, HIG_C, HIG_A, 
 HIG_B, HIG_C, MID_C, MID_D, MID_E, MID_F, MID_E, MID_D, 
 MID_E, MID_C, MID_D, MID_E, MID_D, MID_F, MID_E, MID_D, 
 MID_C, MID_B, MID_C, MID_B, MID_A, MID_B, MID_C, MID_D, 
 MID_E, MID_F, 0    , 0    , 0    , 0    , 0    , 0    ,
 HIG_E, HIG_C, HIG_D, HIG_E, HIG_C, HIG_D, HIG_E, MID_E,
 MID_F, MID_G, HIG_A, HIG_B, HIG_C, HIG_D, HIG_C, HIG_A,
 HIG_B, HIG_C, MID_C, MID_D, MID_E, MID_F, MID_E, MID_D,
 MID_E, MID_C, MID_D, MID_E, MID_D, MID_F, MID_E, MID_D,
 MID_C, MID_B, MID_C, MID_B, MID_A, MID_B, MID_C, MID_D,
 MID_E, MID_F, 0    , 0    , 0    , 0    , 0    , 0    ,
 HIG_E, HIG_C, HIG_D, HIG_E, HIG_C, HIG_D, HIG_E, MID_E,
 MID_F, MID_G, HIG_A, HIG_B, HIG_C, HIG_D, HIG_C, HIG_A,
 HIG_B, HIG_C, MID_C, MID_D, MID_E, MID_F, MID_E, MID_D,
 MID_E, MID_C, MID_D, MID_E, MID_D, MID_F, MID_E, MID_D,},
{HIG_E, HIG_E, HIG_E, },
{LOW_C, LOW_C, LOW_D, },
{LOW_C, LOW_C, LOW_D, },
{LOW_C, LOW_C, LOW_D, }
};
