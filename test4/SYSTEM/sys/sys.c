#include "sys.h" 
/**************************************************************************
Function: Set the vector table offset address
Input   : Base site 	 Offsets
Output  : none
�������ܣ�����������ƫ�Ƶ�ַ
��ڲ�����NVIC_VectTab:��ַ		Offset:ƫ����	
����  ֵ����
**************************************************************************/ 		 
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset)	 
{ 	   	 
	SCB->VTOR = NVIC_VectTab|(Offset & (u32)0x1FFFFF80);//����NVIC��������ƫ�ƼĴ���
																											//���ڱ�ʶ����������CODE��������RAM��
}
/**************************************************************************
Function: Set NVIC group
Input   : NVIC_Group
Output  : none
�������ܣ������жϷ���
��ڲ�����NVIC_Group:NVIC���� 0~4 �ܹ�5�� 	
����  ֵ����
**************************************************************************/ 
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group)	 
{ 
	u32 temp,temp1;	  
	temp1=(~NVIC_Group)&0x07;//ȡ����λ
	temp1<<=8;
	temp=SCB->AIRCR;  //��ȡ��ǰ������
	temp&=0X0000F8FF; //�����ǰ����
	temp|=0X05FA0000; //д��Կ��
	temp|=temp1;	   
	SCB->AIRCR=temp;  //���÷���	    	  				   
}
/**************************************************************************
Function: Set NVIC group
Input   : NVIC_PreemptionPriority	NVIC_SubPriority	NVIC_Channel	NVIC_Group
Output  : none
�������ܣ������жϷ���
��ڲ�������ռ���ȼ�	��Ӧ���ȼ� �жϱ��	�жϷ����
����  ֵ����
* �жϷ��� | ����
 * --------+--------------------------------------
 * ��0     | 0λ��ռ���ȼ�,4λ��Ӧ���ȼ�
 * ��1     | 1λ��ռ���ȼ�,3λ��Ӧ���ȼ�
 * ��2     | 2λ��ռ���ȼ�,2λ��Ӧ���ȼ�
 * ��3     | 3λ��ռ���ȼ�,1λ��Ӧ���ȼ�
 * ��4     | 4λ��ռ���ȼ�,0λ��Ӧ���ȼ�
 *NVIC_SubPriority��NVIC_PreemptionPriority��ԭ����,��ֵԽС,Խ����
**************************************************************************/   
void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group)	 
{ 																									//ע�����ȼ����ܳ����趨����ķ�Χ!����������벻���Ĵ���
	u32 temp;	
	MY_NVIC_PriorityGroupConfig(NVIC_Group);      		//���÷���
	temp=NVIC_PreemptionPriority<<(4-NVIC_Group);	  
	temp|=NVIC_SubPriority&(0x0f>>NVIC_Group);
	temp&=0xf;//ȡ����λ  
	NVIC->ISER[NVIC_Channel/32]|=(1<<NVIC_Channel%32);//ʹ���ж�λ(Ҫ����Ļ�,�෴������OK) 
	NVIC->IP[NVIC_Channel]|=temp<<4;									//������Ӧ���ȼ����������ȼ�   	    	  				   
} 
/**************************************************************************
Function: External interrupt function configuration
Input   : GPIOx��General-purpose input/output	 BITx��The port needed enable
					TRIM��Trigger mode
Output  : none
�������ܣ��ⲿ�жϺ�������
��ڲ�����GPIOx:0~6,����GPIOA~G��
					BITx:��Ҫʹ�ܵ�λ;		TRIM:����ģʽ,1,������;2,�Ͻ���;3�������ƽ����
����  ֵ����
**************************************************************************/
//�ú���һ��ֻ������1��IO��,���IO��,���ε���
//�ú������Զ�������Ӧ�ж�,�Լ�������     	    
void Ex_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM) 
{
	u8 EXTADDR;
	u8 EXTOFFSET;
	EXTADDR=BITx/4;//�õ��жϼĴ�����ı��
	EXTOFFSET=(BITx%4)*4; 
	RCC->APB2ENR|=0x01;//ʹ��io����ʱ��			 
	AFIO->EXTICR[EXTADDR]&=~(0x000F<<EXTOFFSET);//���ԭ�����ã�����
	AFIO->EXTICR[EXTADDR]|=GPIOx<<EXTOFFSET;//EXTI.BITxӳ�䵽GPIOx.BITx 
	//�Զ�����
	EXTI->IMR|=1<<BITx;//  ����line BITx�ϵ��ж�
	//EXTI->EMR|=1<<BITx;//������line BITx�ϵ��¼� (������������,��Ӳ�����ǿ��Ե�,���������������ʱ���޷������ж�!)
 	if(TRIM&0x01)EXTI->FTSR|=1<<BITx;//line BITx���¼��½��ش���
	if(TRIM&0x02)EXTI->RTSR|=1<<BITx;//line BITx���¼��������ش���
} 	  
/**************************************************************************
Function: Reset all clock registers
Input   : none
Output  : none
�������ܣ�������ʱ�ӼĴ�����λ	
��ڲ�������
����  ֵ����
**************************************************************************/
//����������ִ���������踴λ!�����������𴮿ڲ�����.		    
//������ʱ�ӼĴ�����λ			  
void MYRCC_DeInit(void)
{	
 	RCC->APB1RSTR = 0x00000000;//��λ����			 
	RCC->APB2RSTR = 0x00000000; 
	  
  RCC->AHBENR = 0x00000014;  //˯��ģʽ�����SRAMʱ��ʹ��.�����ر�.	  
  RCC->APB2ENR = 0x00000000; //����ʱ�ӹر�.			   
  RCC->APB1ENR = 0x00000000;   
	RCC->CR |= 0x00000001;     //ʹ���ڲ�����ʱ��HSION	 															 
	RCC->CFGR &= 0xF8FF0000;   //��λSW[1:0],HPRE[3:0],PPRE1[2:0],PPRE2[2:0],ADCPRE[1:0],MCO[2:0]					 
	RCC->CR &= 0xFEF6FFFF;     //��λHSEON,CSSON,PLLON
	RCC->CR &= 0xFFFBFFFF;     //��λHSEBYP	   	  
	RCC->CFGR &= 0xFF80FFFF;   //��λPLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE 
	RCC->CIR = 0x00000000;     //�ر������ж�		 
	//����������				  
#ifdef  VECT_TAB_RAM
	MY_NVIC_SetVectorTable(0x20000000, 0x0);
#else   
	MY_NVIC_SetVectorTable(0x08000000,0x0);
#endif
}
//THUMBָ�֧�ֻ������
//�������·���ʵ��ִ�л��ָ��WFI  
__asm void WFI_SET(void)
{
	WFI;		  
}
//�ر������ж�
__asm void INTX_DISABLE(void)
{
	CPSID I;		  
}
//���������ж�
__asm void INTX_ENABLE(void)
{
	CPSIE I;		  
}
//����ջ����ַ
//addr:ջ����ַ
__asm void MSR_MSP(u32 addr) 
{
    MSR MSP, r0 			//set Main Stack value
    BX r14
}

/**************************************************************************
Function: Go to standby mode
Input   : none
Output  : none
�������ܣ��������ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/  	  
void Sys_Standby(void)
{
	SCB->SCR|=1<<2;							//ʹ��SLEEPDEEPλ (SYS->CTRL)	   
  RCC->APB1ENR|=1<<28;     	//ʹ�ܵ�Դʱ��	    
 	PWR->CSR|=1<<8;          		//����WKUP���ڻ���
	PWR->CR|=1<<2;           		//���Wake-up ��־
	PWR->CR|=1<<1;          		//PDDS��λ		  
	WFI_SET();				 					//ִ��WFIָ��		 
}	     
/**************************************************************************
Function: System soft reset
Input   : none
Output  : none
�������ܣ�ϵͳ����λ
��ڲ�������
����  ֵ����
**************************************************************************/  
void Sys_Soft_Reset(void)
{   
	SCB->AIRCR =0X05FA0000|(u32)0x04;	  
} 		 
/**************************************************************************
Function: Set JTAG mode
Input   : mode:JTAG, swd mode settings��00��all enable��01��enable SWD��10��Full shutdown
Output  : none
�������ܣ�����JTAGģʽ
��ڲ�����mode:jtag,swdģʽ����;00,ȫʹ��;01,ʹ��SWD;10,ȫ�ر�;	
����  ֵ����
**************************************************************************/
//#define JTAG_SWD_DISABLE   0X02
//#define SWD_ENABLE         0X01
//#define JTAG_SWD_ENABLE    0X00	
void JTAG_Set(u8 mode)
{
	u32 temp;
	temp=mode;
	temp<<=25;
	RCC->APB2ENR|=1<<0;     //��������ʱ��	   
	AFIO->MAPR&=0XF8FFFFFF; //���MAPR��[26:24]
	AFIO->MAPR|=temp;       //����jtagģʽ
} 
/**************************************************************************
Function: System clock initialization function
Input   : pll��Selected frequency multiplication��Starting at 2, the maximum value is 16
Output  : none
�������ܣ�ϵͳʱ�ӳ�ʼ������
��ڲ�����pll:ѡ��ı�Ƶ������2��ʼ�����ֵΪ16		
����  ֵ����
**************************************************************************/ 
void Stm32_Clock_Init(u8 PLL)
{
	unsigned char temp=0;   
	MYRCC_DeInit();		  		//��λ������������
 	RCC->CR|=0x00010000;  	//�ⲿ����ʱ��ʹ��HSEON
	while(!(RCC->CR>>17));	//�ȴ��ⲿʱ�Ӿ���
	RCC->CFGR=0X00000400; 	//APB1=DIV2;APB2=DIV1;AHB=DIV1;
	PLL-=2;									//����2����λ
	RCC->CFGR|=PLL<<18;   	//����PLLֵ 2~16
	RCC->CFGR|=1<<16;	 			//PLLSRC ON 
	FLASH->ACR|=0x32;	  		//FLASH 2����ʱ����

	RCC->CR|=0x01000000;  	//PLLON
	while(!(RCC->CR>>25));	//�ȴ�PLL����
	RCC->CFGR|=0x00000002;	//PLL��Ϊϵͳʱ��	 
	while(temp!=0x02)     	//�ȴ�PLL��Ϊϵͳʱ�����óɹ�
	{   
		temp=RCC->CFGR>>2;
		temp&=0x03;
	}    
}		    

