#ifndef _TIM2_CAP_H
#define _TIM2_CAP_H
#include "sys.h"
extern u32 period_count,high_level_count;//单个周期计数和高电平计数值
void TIM3_Cap_Init(u16 arr,u16 psc);
float Get_Position(u16 times);

#endif
