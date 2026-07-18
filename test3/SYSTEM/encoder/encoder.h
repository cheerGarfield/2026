#ifndef _ENCODER_H
#define _ENCODER_H
#include "sys.h"



extern u16 AngleData;


float Get_Machinery_Angle(void);
u16 SPI_ReadData(void);
u16 SPI_WriteReadData(u16 TxData);
void time_of_round(void);
u16 Spi_Transmit_Receive(u16 data);
void Encoder_Init(void);
int32_t Read_Encoder(void);
void Spi_Init(void);
#endif
