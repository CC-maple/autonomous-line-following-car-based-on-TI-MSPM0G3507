#ifndef _UART_GYRO_H
#define _UART_GYRO_H

#include <stdint.h>



extern uint8_t data_tx[15];

extern double Angle[3];
extern double T;

void uart_gyro_init();
void uart_gyro_disabled();
void DecodeIMUData(uint8_t chrT[]);

#endif