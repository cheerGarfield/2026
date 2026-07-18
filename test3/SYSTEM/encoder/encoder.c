#include "encoder.h"

u16 Angle_Data;//用于更新编码器角度
void Encoder_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
	NVIC_InitTypeDef NVIC_InitStruct;	
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//使能定时器4的时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);//使能PB端口时钟
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;	//端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //上拉输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);					      //根据设定参数初始化GPIOB
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; // 预分频器 
  TIM_TimeBaseStructure.TIM_Period = 65535; //设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//选择时钟分频：不分频
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;////TIM向上计数  
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);//使用编码器模式3
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 10;
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
  
  NVIC_InitStruct.NVIC_IRQChannel = TIM4_IRQn;  		//定时器4中断
   NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;  			//使能IRQ通道
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;	//抢占优先级1 
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;       	//响应优先级3
	NVIC_Init(&NVIC_InitStruct);
  
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);//清除TIM的更新标志位
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
 
  TIM_SetCounter(TIM4,0);
  TIM_Cmd(TIM4, ENABLE); 
}


int32_t Read_Encoder(void)
{
	int32_t Encoder_TIM;
	Encoder_TIM = (short)TIM4->CNT;
//	TIM4->CNT = 0;
	return Encoder_TIM;
}

void TIM4_IRQHandler(void)
{
	if(TIM_GetFlagStatus(TIM4,TIM_FLAG_Update)==SET)
	{
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update); 	//清除中断标志位  
	}
	
}

//下面是软件SPI
#define ENC_SPI_GPIO  GPIOA
#define ENC_SPI_SCK   GPIO_Pin_3
#define ENC_SPI_MISO  GPIO_Pin_1//单片机的MISO引脚接编码器的MOSI
#define ENC_SPI_MOSI  GPIO_Pin_0//单片机的MOSI引脚接编码器的MISO
#define ENC_SPI_CS    GPIO_Pin_2
//MT6816 寄存器地址
#define ANGLE_VALUE_H           (uint16_t)0x83
#define ANGLE_VALUE_L           (uint16_t)0x84
/************************************************
函数功能：初始化SPI的接口
入口参数：无
返回值  ：无
************************************************/
void Spi_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = ENC_SPI_SCK|ENC_SPI_MOSI;
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(ENC_SPI_GPIO,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = ENC_SPI_CS;
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
	GPIO_Init(ENC_SPI_GPIO,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = ENC_SPI_MISO;
	GPIO_InitStructure.GPIO_Mode= GPIO_Mode_IN_FLOATING;
	GPIO_Init(ENC_SPI_GPIO,&GPIO_InitStructure);
	
}

/*****************************************************
函数功能：向SPI发送数据和读取spi的数据
入口参数：发送的数据
返回值  ：读取的编码器数据
********************************************************/
u16 Spi_Transmit_Receive(u16 TxData)
{
	u16 recv_data=0;
	int i,j;
	for(i=0;i<8;i++)//先发送数据
	{		
		if(TxData & 0x80)
			GPIO_SetBits(ENC_SPI_GPIO,ENC_SPI_MOSI);
		else
			GPIO_ResetBits(ENC_SPI_GPIO,ENC_SPI_MOSI);
		GPIO_ResetBits(ENC_SPI_GPIO,ENC_SPI_SCK);
		delay_us(1);
		GPIO_SetBits(ENC_SPI_GPIO,ENC_SPI_SCK);
		TxData <<= 1;
	}
	for(j=0;j<8;j++)//再读数据
	{
		recv_data <<= 1;
		GPIO_ResetBits(ENC_SPI_GPIO,ENC_SPI_SCK);
		delay_us(1);
		GPIO_SetBits(ENC_SPI_GPIO,ENC_SPI_SCK);
		if(GPIO_ReadInputDataBit(ENC_SPI_GPIO,ENC_SPI_MISO)==1)
             recv_data +=1;			
	}
	return recv_data;
}

float Get_Machinery_Angle(void)
{
	float position=0;
	u16 data_r[3];
	GPIO_ResetBits(ENC_SPI_GPIO,ENC_SPI_CS);
	delay_us(1);
	data_r[0]=Spi_Transmit_Receive(ANGLE_VALUE_H);
	GPIO_SetBits(ENC_SPI_GPIO,ENC_SPI_CS);
	delay_us(1);
	GPIO_ResetBits(ENC_SPI_GPIO,ENC_SPI_CS);
	delay_us(1);
	data_r[1]=Spi_Transmit_Receive(ANGLE_VALUE_L);
	GPIO_SetBits(ENC_SPI_GPIO,ENC_SPI_CS);
	delay_us(1);
	data_r[2] = ((data_r[0]<<6)|(data_r[1]>>2)); //合成寄存器完整数据
	position = data_r[2]/16384.0*360.0;//
	return position;
}

