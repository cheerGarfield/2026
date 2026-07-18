#include "control.h"

/* TIM1更新中断: 已清空
   原代码在此调用click()并自增Motor_T_Position,
   与主循环PID输出存在竞态. 按键检测改在主循环中进行.
   保留空中断体(仅清标志), 避免TIM1未处理中断卡死.
*/
void TIM1_UP_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM1,TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM1,TIM_IT_Update);
	}
}
