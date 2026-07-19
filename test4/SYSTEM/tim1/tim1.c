#include "tim1.h"
volatile uint32_t usTicks = 0;     // 微秒时基计数器，每个tick为10微秒
/**************************************************************************
Function: TIMING_TIM_Init
Input   : TIM_Period,TIM_Prescaler
Output  : none
函数功能：定时中断初始化
入口参数: 预装载值和预分频器 
返回  值：无
**************************************************************************/	 	
//定时器TIM1
void TIM1_Init(u16 arr,u16 psc)
{
	//定时时间：(arr+1)*(psc+1)/CLK
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStructure; 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
		// 设置中断组为0
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);		
		// 设置中断来源
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn ;	
		// 设置主优先级为 1
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;	 
		// 设置抢占优先级为0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	// 开启定时器时钟,即内部时钟CK_INT=72M
	

	/*--------------------时基结构体初始化-------------------------*/
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
	TIM_TimeBaseInitStruct.TIM_Period = arr;              			//设定计数器自动重装值 
	TIM_TimeBaseInitStruct.TIM_Prescaler  = psc;          			//设定预分频器
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;	//TIM向上计数模式
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;        //设置时钟分割
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStruct);      		//初始化定时器
		// 清除计数器中断标志位
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
		// 开启计数器中断
    TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);

	TIM_Cmd(TIM1,ENABLE);                              		//使能定时器

}



