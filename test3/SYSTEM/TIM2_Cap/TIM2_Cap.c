#include "TIM2_Cap.h"

u32 period_count,high_level_count;//单个周期计数和高电平计数值

void TIM3_Cap_Init(u16 arr,u16 psc)
{
	//定义相关结构体
	GPIO_InitTypeDef GPIO_InitStructure; //定义一个GPIO初始化的结构体
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure; //定义一个定时器初始化的结构体
	TIM_ICInitTypeDef  TIM3_ICInitStructure; //定义一个定时器捕获输入初始化的结构体
//	TIM_OCInitTypeDef  TIM_OCInitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure; //定义一个中断优先级初始化的结构体

	//使能相关时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);	//使能TIM3时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB, ENABLE);  //使能GPIOA时钟
	
	//初始化GPIOA2
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6;  //PA6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure); //根据GPIO_InitStructure的参数初始化GPIOA6

	
	//初始化定时器TIM3
	TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //预分频器   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;	
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct的参数初始化定时器TIM3
  
	
	//初始化TIM3输入捕获参数
	TIM3_ICInitStructure.TIM_Channel = TIM_Channel_1; //输入捕获通道1
	TIM3_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
	TIM3_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //
	TIM3_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频为不分频，分频决定几个捕获事件触发中断服务函数
	TIM3_ICInitStructure.TIM_ICFilter = 0x06;//IC1F=0000 配置输入滤波器为不滤波
	TIM_ICInit(TIM3, &TIM3_ICInitStructure);
	

	//中断分组初始化
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //抢占优先级1级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  //从优先级1级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //中断使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
	
	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);	
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
	TIM_Cmd(TIM3, ENABLE); 	//使能定时器3
}

//定时器2中断服务程序
//也可以使用捕获功能
void TIM3_IRQHandler(void)
{ 
	static u8 state = 0;
	static u8 cnt = 0;
	static u16 cnt_period = 0;
	if(cnt_period < 30)
	{
		if(TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)//进入捕获中断
		{
			switch(state)
			{
				case 0://开始计数
					state++;
					TIM3->CNT = 0;
					break;
				case 1:
					
					period_count += (TIM3->CCR1 + cnt*65536);//获取计数值
					if(++cnt_period == 30)
						period_count = period_count/30;
					cnt = 0;
					state = 0;
					break;
				default:break;
			}
		}
		if((TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)&& (state == 1))//溢出中断，且已经捕获到了第一个上升沿，此时开始计数
			cnt++;
	}
	
	else//此时开始计算高电平的时间
	{
		if(TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)//进入捕获中断
		{
			switch(state)
			{
				case 0://获取到上升沿
					state++;
					TIM3->CNT = 0;
					TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Falling); //CC4P=1 设置为下降沿捕获
					break;
				case 1:
					//state++;//
					high_level_count = TIM3->CCR1 + cnt*65536;//获取高电平计数时间
					cnt = 0;
					state = 0;
					TIM_OC1PolarityConfig(TIM3,TIM_ICPolarity_Rising);///设置上升沿
					break;
				default:break;
			}
		}
		if((TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)&& state == 1)//溢出中断，且已经捕获到了第一个上升沿，此时开始计数
			cnt++;
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1|TIM_IT_Update); //清除中断标志位
	
}

//获取角度的位置信息
//计算公式：Position = ton * 4115 / (ton + toff) - 1 
float Get_Position(u16 times)
{
	u16 temp_count = 0;
	float Position_Sum = 0;
	for(temp_count = 0;temp_count < times;temp_count++)
	{
		Position_Sum += ((float)high_level_count/(float)period_count*4115-1);
		__nop();//小延时
	}
	Position_Sum = Position_Sum/times/4115*360;//计算位置信息，数值是0--360
	return Position_Sum;
	
}
