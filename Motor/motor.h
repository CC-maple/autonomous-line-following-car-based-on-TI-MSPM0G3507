#ifndef _MOTOR_H
#define _MOTOR_H

//#include "ti_msp_dl_config.h"
#include "ti/driverlib/dl_gpio.h"

#define BIN1_OUT(X)  ( (X) ? (DL_GPIO_setPins(GPIOB,DL_GPIO_PIN_15)) : (DL_GPIO_clearPins(GPIOB,DL_GPIO_PIN_15)) )
#define BIN2_OUT(X)  ( (X) ? (DL_GPIO_setPins(GPIOB,DL_GPIO_PIN_16)) : (DL_GPIO_clearPins(GPIOB,DL_GPIO_PIN_16)) )

#define AIN1_OUT(X)  ( (X) ? (DL_GPIO_setPins(GPIOB,DL_GPIO_PIN_13)) : (DL_GPIO_clearPins(GPIOB,DL_GPIO_PIN_13)) )
#define AIN2_OUT(X)  ( (X) ? (DL_GPIO_setPins(GPIOB,DL_GPIO_PIN_12)) : (DL_GPIO_clearPins(GPIOB,DL_GPIO_PIN_12)) )

#define CIN1_OUT(X)    ( (X) ? (DL_GPIO_setPins(GPIOA,DL_GPIO_PIN_14)) : (DL_GPIO_clearPins(GPIOA,DL_GPIO_PIN_14)) )
#define CIN2_OUT(X)    ( (X) ? (DL_GPIO_setPins(GPIOA,DL_GPIO_PIN_27)) : (DL_GPIO_clearPins(GPIOA,DL_GPIO_PIN_27)) )

#define DIN1_OUT(X)    ( (X) ? (DL_GPIO_setPins(GPIOA,DL_GPIO_PIN_15)) : (DL_GPIO_clearPins(GPIOA,DL_GPIO_PIN_15)) )
#define DIN2_OUT(X)    ( (X) ? (DL_GPIO_setPins(GPIOA,DL_GPIO_PIN_16)) : (DL_GPIO_clearPins(GPIOA,DL_GPIO_PIN_16)) )

void Limit(int *motorA, int *motorB, int *motorC, int *motorD);
void Load(int motor1, int motor2, int motor3, int motor4);

int my_abs(int p);
int get_abs(int x);
void brake(void);//两驱没改

#endif