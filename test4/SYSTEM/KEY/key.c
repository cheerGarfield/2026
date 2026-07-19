#include "key.h"

/**************************************************************************
Function: Key initialization
Input   : none
Output  : none
�������ܣ�������ʼ��
��ڲ�������
����  ֵ���� 
**************************************************************************/
void KEY_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��GPIOBʱ��
  GPIO_InitStructure.GPIO_Pin = KEY_PIN; //KEY��Ӧ����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��ͨ����ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100M
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOB14	
} 
/**************************************************************************
Function: Buttons to scan
Input   : none
Output  : none
�������ܣ�����ɨ��
��ڲ�������
����  ֵ������״̬ 0���޶��� 1������ 
**************************************************************************/
u8 click(void)
{
	//Press the release sign
	//�������ɿ���־
	static u8 flag_key=1;
	
	if(flag_key&&KEY==0)
	{
	 flag_key=0; //The key is pressed //��������
	 return 1;	
	}
	else if(1==KEY)			
		flag_key=1;
	return 0; //No key is pressed //�ް�������
}
/**************************************************************************
Function: Delay function
Input   : none
Output  : none
�������ܣ��ӳٺ���
��ڲ�������
�� �� ֵ����
**************************************************************************/
void Delay_ms(void)
{
   int ii,i;    
   for(ii=0;ii<50;ii++)
   {
	   for(i=0;i<50;i++);
	 }	
}
/**************************************************************************
Function: Buttons to scan
Input   : Double click wait time
Output  : Button status: 0- no action, 1- click, 2- double click
�������ܣ�����ɨ��
��ڲ�����˫���ȴ�ʱ��
����  ֵ������״̬: 0-�޶���, 1-����, 2-˫�� 
**************************************************************************/
u8 click_N_Double (u8 time)
{
		static	u8 flag_key,count_key,double_key;	
		static	u16 count_single,Forever_count;
	
	  if(KEY==0)  Forever_count++;   
    else        Forever_count=0;
	
		if(0==KEY&&0==flag_key)		flag_key=1;	
	  if(0==count_key)
		{
				if(flag_key==1) 
				{
					double_key++;
					count_key=1;	
				}
				if(double_key==2) 
				{
					double_key=0;
					count_single=0;
					return 2; //Double click //˫��
				}
		}
		if(1==KEY)			flag_key=0,count_key=0;
		
		if(1==double_key)
		{
			count_single++;
			if(count_single>time&&Forever_count<time)
			{
			double_key=0;
			count_single=0;	
			return 1; //Click //����
			}
			if(Forever_count>time)
			{
			double_key=0;
			count_single=0;	
			}
		}	
		return 0;
}
/**************************************************************************
Function: Button scan.Because static variables are used, a function with a different name needs to be defined when the keystroke scan function is used multiple times
Input   : none
Output  : Button status: 0- no action, 1- click, 2- double click
�������ܣ�����ɨ�衣��Ϊʹ�õ��˾�̬���������ദ��Ҫʹ�ð���ɨ�躯��ʱ����Ҫ�ٶ���һ����ͬ������
��ڲ�������
�� �� ֵ������״̬: 0-�޶���, 1-����, 2-˫�� 
**************************************************************************/
u8 click_N_Double_MPU6050 (u8 time)
{
		static	u8 flag_key,count_key,double_key;	
		static	u16 count_single,Forever_count;
	
	  if(KEY==0)  Forever_count++;  
    else        Forever_count=0;
		if(0==KEY&&0==flag_key)		flag_key=1;	
	  if(0==count_key)
		{
				if(flag_key==1) 
				{
					double_key++;
					count_key=1;	
				}
				if(double_key==2) 
				{
					double_key=0;
					count_single=0;
					return 2; //Double click //˫��
				}
		}
		if(1==KEY)			flag_key=0,count_key=0;
		
		if(1==double_key)
		{
			count_single++;
			if(count_single>time&&Forever_count<time)
			{
			double_key=0;
			count_single=0;	
			return 1; //Click //����
			}
			if(Forever_count>time)
			{
			double_key=0;
			count_single=0;	
			}
		}	
		return 0;
}
/**************************************************************************
Function: Long according to the test
Input   : none
Output  : Key state 0: no action 1: long press 3s
�������ܣ��������
��ڲ�������
����  ֵ������״̬ 0���޶��� 1������3s
**************************************************************************/
u8 Long_Press(void)
{
	static u16 Long_Press_count,Long_Press;

	if(Long_Press==0&&KEY==0)  Long_Press_count++; 
	else                       Long_Press_count=0;

	if(Long_Press_count>15)	//3 seconds //3��	
	{
		Long_Press=1;	
		Long_Press_count=0;
		return 1;
	}				
	 if(Long_Press==1) //Long press position 1 //������־λ��1
	{
			Long_Press=0;
	}
	return 0;
}

//����ɨ�躯��
//��ڲ�����ִ�иú���������Ƶ��,�ӳ��˲���ʱ��
//����ֵ��long_click��double_click��single_click��key_stateless
//����ֵ���ٵ�����ֻɨ�谴����KEY_Scan(3000,200)
//       �ڵ����������������Ƚ϶࣬KEY_Scan(200,0)
u8 KEY_Scan(u16 Frequency,u16 filter_times)
{
    static u16 time_core;//��ʱ����
    static u16 long_press_time;//����ʶ��
    static u8 press_flag=0;//�������±��
    static u8 check_once=0;//�Ƿ��Ѿ�ʶ��1�α��
    static u16 delay_mini_1;
    static u16 delay_mini_2;
	
    float Count_time = (((float)(1.0f/(float)Frequency))*1000.0f);//�����1��Ҫ���ٸ�����

    if(check_once)//�����ʶ����������б���
    {
        press_flag=0;//�����1��ʶ�𣬱������
        time_core=0;//�����1��ʶ��ʱ������
        long_press_time=0;//�����1��ʶ��ʱ������
        delay_mini_1=0;
        delay_mini_2=0;
    }
    if(check_once&&KEY==1) check_once=0; //���ɨ��󰴼�������������һ��ɨ��

    if(KEY==0&&check_once==0)//����ɨ��
    {
        press_flag=1;//��Ǳ�����1��
		
        if(++delay_mini_1>filter_times)
        {
            delay_mini_1=0;
            long_press_time++;		
        }
    }

    if(long_press_time>(u16)(600.0f/Count_time))// ����1��
    {	
        check_once=1;//����ѱ�ʶ��
        return long_click; //����
    }

    //����������1���ֵ���󣬿����ں���ʱ
    if(press_flag&&KEY==1)
    {
        if(++delay_mini_2>filter_times)
        {
            delay_mini_2=0;
            time_core++; 
        }
    }		
	
    if(press_flag&&(time_core>(u16)(50.0f/Count_time)&&time_core<(u16)(500.0f/Count_time)))//50~700ms�ڱ��ٴΰ���
    {
        if(KEY==0) //����ٴΰ���
        {
            check_once=1;//����ѱ�ʶ��
            return double_click; //���Ϊ˫��
        }
    }
    else if(press_flag&&time_core>(u16)(500.0f/Count_time))
    {
        check_once=1;//����ѱ�ʶ��
        return single_click; //800ms��û�����£����ǵ���
    }

    return key_stateless;
}


