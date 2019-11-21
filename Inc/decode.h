//decode.h
#ifndef __DECODE_H
#define __DECODE_H

#include "STM32F4xx.h"
void decode_command(char cmd[100]);

void transmit_slot_config(void);
void transmit_really_fast(void);

void transmit_2x_frame(void);
void dynamic_loop(void);
void transmit_vcore2_dvs1(void);
void transmit_vcore2_dvs2(void);
void transmit_vcore2_dvs3(void);
#endif
