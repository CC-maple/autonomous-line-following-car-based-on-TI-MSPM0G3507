#ifndef _UART_GYRO_H
#define _UART_GYRO_H

#include <stdint.h>



extern uint8_t data_tx[15];

extern double Angle[3];
extern double T;

void uart_gyro_init(void);
void uart_gyro_disabled(void);
void DecodeIMUData(const uint8_t chrT[]);
void uart_gyro_tick(void);
uint8_t uart_gyro_is_fresh(void);
float uart_gyro_heading_degrees(void);

#endif