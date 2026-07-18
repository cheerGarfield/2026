/***********************************************
  WHEELTEC F1 STM32F103 UART driver
  UART1(PA9/PA10): K230 vision module
  UART2(PA2/PA3):  F32C gimbal motor
***********************************************/
#include "sys.h"
#include "usart.h"
u8 Send_Data[12];
extern int Time_count;
RECEIVE_DATA Receive_Data;
#define CONTROL_DELAY		1000
#if SYSTEM_SUPPORT_OS
#include "includes.h"
#endif

/* printf redirected to UART1 disabled (conflicts with K230 protocol) */
#if 0
#pragma import(__use_no_semihosting)
struct __FILE
{
	int handle;
};
FILE __stdout;
_sys_exit(int x)
{
	x = x;
}
int fputc(int ch, FILE *f)
{
	while((USART1->SR&0X40)==0);
	USART1->DR = (u8) ch;
	return ch;
}
#endif

/**************************************************************************
Function: Serial port 1 initialization (K230)
**************************************************************************/
void uart_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);

	/* USART1_TX   GPIOA.9 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART1_RX   GPIOA.10 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Usart1 NVIC */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);
}

u8 usart2_receive_data[10];

/**************************************************************************
Function: Serial port 2 initialization (F32C motor, DMA receive)
**************************************************************************/
void uart2_init(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);

	/* USART2_TX   GPIOA.2 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART2_RX   GPIOA.3 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

    DMA_DeInit(DMA1_Channel6);
	DMA_InitStructure.DMA_BufferSize = 10;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)usart2_receive_data;
	DMA_InitStructure.DMA_MemoryDataSize = 	DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART2->DR);
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_Init(DMA1_Channel6,&DMA_InitStructure);
	DMA_ClearFlag(DMA1_FLAG_TC6);
	DMA_Cmd(DMA1_Channel6,ENABLE);

    /* Usart2 NVIC */
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
    USART_Cmd(USART2, ENABLE);
}

/**************************************************************************
Function: UART1 send single byte
**************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);
}

/**************************************************************************
Function: UART1 send data buffer
**************************************************************************/
void USART1_SEND(u8* data,u8 len)
{
     u8 i = 0;
	for(i=0; i<len; i++)
	{
		usart1_send(data[i]);
	}
}

/**************************************************************************
Function: UART2 send single byte
**************************************************************************/
void usart2_send(u8 data)
{
	USART2->DR = data;
	while((USART2->SR&0x40)==0);
}

/**************************************************************************
Function: UART2 send data buffer
**************************************************************************/
void USART2_SEND(u8* data,u8 len)
{
     u8 i = 0;
	for(i=0; i<len; i++)
	{
		usart2_send(data[i]);
	}
}

/**************************************************************************
Function: BCC checksum (XOR of first Count_Number bytes)
**************************************************************************/
u8 BCC_Sum1(u8* usart_data,unsigned char Count_Number)
{
    unsigned char crc_sum = 0,k;
	for(k=0;k<Count_Number;k++)
	{
		crc_sum=crc_sum^usart_data[k];
	}
	return crc_sum;
}

/***************************************************************************
Function: UART1 RX interrupt (K230 frame byte-by-byte)
***************************************************************************/
extern void k230_rx_byte(u8 b);

void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		u8 b = USART_ReceiveData(USART1);
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		k230_rx_byte(b);
	}
}

/***************************************************************************
Function: UART2 RX interrupt (F32C motor feedback, DMA + IDLE)
  Response frame: 7A ID type 00 00 angleH angleL BCC 7B (9 bytes)
  type=0x02: mechanical angle (single-turn 0~3599)
***************************************************************************/
extern volatile int Motor1_Current_Position;

void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2,USART_IT_IDLE)!=RESET)
	{
		USART_ReceiveData(USART2);  /* read DR to clear IDLE flag */
		DMA_Cmd(DMA1_Channel6,DISABLE);
		/* BCC check: [7] == XOR([0]~[6]) */
		if(usart2_receive_data[7]==BCC_Sum1(usart2_receive_data,7))
		{
			/* motor1 mechanical angle feedback (type=0x02) */
			if(usart2_receive_data[1]==0x01 && usart2_receive_data[2]==0x02)
			{
				Motor1_Current_Position = (usart2_receive_data[5]<<8) | usart2_receive_data[6];
			}
		}
		DMA_SetCurrDataCounter(DMA1_Channel6,10);
	    DMA_Cmd(DMA1_Channel6,ENABLE);
	}
}
