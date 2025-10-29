/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : lib_music.h
 *  Description  : Music Mapping
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 14:59:17
 *  LastEditTime : 2023-01-23 17:05:51
 */
#ifndef MUSIC_LIB_H
#define MUSIC_LIB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define LOW_C   262
#define LOW_D   294
#define LOW_E   330
#define LOW_F   349
#define LOW_G   392
#define LOW_A   440
#define LOW_B   494

#define MID_C   262
#define MID_D   294
#define MID_E   330
#define MID_F   349
#define MID_G   392
#define MID_A   440
#define MID_B   494

#define HIG_C   262
#define HIG_D   294
#define HIG_E   330
#define HIG_F   349
#define HIG_G   392
#define HIG_A   440
#define HIG_B   494

#define MUSIC_BEEPER_MAXNUM 5
#define MUSIC_BEEPER_LENGTH 128
extern const uint32_t BEEPER_MUSIC[MUSIC_BEEPER_MAXNUM][MUSIC_BEEPER_LENGTH];

#ifdef __cplusplus
}
#endif

#endif
