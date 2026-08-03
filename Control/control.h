#ifndef _CONTROL_H
#define _CONTROL_H

//#include "ti_msp_dl_config.h"
#include "stdint.h"
int Position_PID(int reality,int target);
int limit_control(int x,int x_min, int x_max);//正值限幅函数

extern int64_t EncoderA, EncoderB, EncoderC, EncoderD;
extern float Position_KP,Position_KI,Position_KD;

extern int8_t huidu_data[8];

void Control_Init(void);

extern int16_t speed_left,speed_right;



#endif
