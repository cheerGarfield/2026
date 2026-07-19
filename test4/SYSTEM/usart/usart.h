/***********************************************
公司：轮趣科技(东莞)有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2022-09-01

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2022-09-01

All rights reserved
***********************************************/
#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 
#define RECEIVE_DATA_SIZE 11             //接收的数组大小
#define FRAME_HEADER      0X7B          //帧头
#define FRAME_tail        0X7D          //帧尾
typedef struct _RECEIVE_DARA_
{
	u8 buffer[RECEIVE_DATA_SIZE];
	unsigned char PC_buffer[12];
}RECEIVE_DATA;
extern u8 Send_Data[12];

void uart_init(u32 bound);
void usart1_send(u8 data);
u8 CRC_Sum(unsigned char Count_Number);
float Speed_transition(u8 High,u8 Low);
u32 Position_transition(u8 High,u8 Low);
float SPeed_transition(u8 High,u8 Low);
float Angle_transition(u8 High,u8 Low);
float Baud_transition(u8 High,u8 middle,u8 Low);
void uart3_init(u32 bound);
void data_transition(void);
void USART1_SEND(u8* data,u8 len);
u8 CRC_Sum1(unsigned char Count_Number);
void USART2_SEND(u8* data,u8 len);
u8 BCC_Sum1(u8* usart_data,unsigned char Count_Number);
void uart2_init(u32 bound);
void usart2_send(u8 data);
#endif


