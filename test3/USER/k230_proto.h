#ifndef __K230_PROTO_H
#define __K230_PROTO_H

#include "sys.h"

/* ==================== K230帧协议 ====================
   K230 -> STM32 (14字节, 主动上报, 30ms/帧):
     AA 55 FLAGS D0_H D0_L D1_H D1_L D2_H D2_L D3_H D3_L ERR_H ERR_L XOR
   校验: byte[13] = byte[2]^...^byte[12]

   STM32 不再向 K230 发送指令（K230自带SCAN→ALIGN→AIM状态机）
*/

#define K230_FRAME_LEN    14
#define K230_HEADER1      0xAA
#define K230_HEADER2      0x55

/* flags 字段低3位:
     bit0(0x01): 有靶纸
     bit1(0x02): 有激光光斑
     flags=0x00 SCAN / 0x01 ALIGN / 0x03 AIM
*/

/* 解析后的K230帧数据
   ALIGN: data0=靶心X, data1=靶心Y (其余0)
   AIM  : data0=靶心X, data1=靶心Y, data2=光斑X, data3=光斑Y, error=偏差px
   坐标系640x480, 中心(320,240)
*/
typedef struct {
    u8  flags;
    int data0;
    int data1;
    int data2;
    int data3;
    int error;
} K230_Frame_t;

extern volatile u8  k230_frame_ready;
extern volatile u32 k230_frame_count;   /* 诊断: 累计接收到的帧数 */
extern K230_Frame_t k230_frame;

void k230_proto_init(void);
void k230_rx_byte(u8 b);
void k230_parse_frame(void);

#endif
