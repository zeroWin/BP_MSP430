#ifndef _HAL_DELAY_H
#define _HAL_DELAY_H
#include  "typedefs.h"
#include  <msp430f1611.h>


extern void usDelay(int microeconds);
extern void delay_ms(u16 nms);
extern void delay_s(u16 ns);
#endif