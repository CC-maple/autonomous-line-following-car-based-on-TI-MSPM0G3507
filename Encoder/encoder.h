#ifndef _ENCODER_H
#define _ENCODER_H

#include <stdint.h>

extern uint8_t mode;
extern uint8_t begin;
void Encoder_Init(void);
void encoder_read(int64_t *a,int64_t *b, int64_t *c, int64_t *d);

#endif