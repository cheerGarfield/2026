#ifndef _TIM1_H
#define _TIM1_H
#include "sys.h"
extern volatile uint32_t usTicks;     // 微秒时基计数器，每个tick为10微秒
void TIM1_Init(u16 arr,u16 psc);
void TIM4_Init(u16 arr,u16 psc);
u32 Run(u16 target_position);
#endif
