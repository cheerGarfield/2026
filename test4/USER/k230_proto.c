#include "k230_proto.h"
#include <string.h>

volatile u8  k230_frame_ready = 0;
volatile u32 k230_frame_count = 0;   /* 诊断: 累计接收到的完整帧数 */
K230_Frame_t k230_frame;

/* ==================== 接收缓冲区 ==================== */
static u8  rx_buf[K230_FRAME_LEN];
static u8  rx_idx  = 0;
static u8  rx_state = 0;   /* 0=等0xAA, 1=等0x55, 2=收数据 */

/* ==================== 初始化 ==================== */
void k230_proto_init(void)
{
    rx_state = 0;
    rx_idx   = 0;
    k230_frame_ready = 0;
    memset((void *)&k230_frame, 0, sizeof(K230_Frame_t));
}

/* ==================== 接收单字节（由USART1中断调用） ==================== */
void k230_rx_byte(u8 b)
{
    u8 calc, i;

    switch (rx_state) {
    case 0:
        if (b == K230_HEADER1) {
            rx_buf[0] = b;
            rx_state = 1;
        }
        break;

    case 1:
        if (b == K230_HEADER2) {
            rx_buf[1] = b;
            rx_idx = 2;
            rx_state = 2;
        } else {
            rx_state = 0;
        }
        break;

    case 2:
        rx_buf[rx_idx++] = b;
        if (rx_idx >= K230_FRAME_LEN) {
            rx_state = 0;

            /* XOR校验: 字节2~12异或应等于字节13 */
            calc = 0;
            for (i = 2; i < 13; i++) {
                calc ^= rx_buf[i];
            }
            if (calc == rx_buf[13]) {
                k230_parse_frame();
            }
        }
        break;
    }
}

/* ==================== 解析完整帧 ==================== */
void k230_parse_frame(void)
{
    k230_frame.flags = rx_buf[2] & 0x07;

    /* 大端序解包 */
    k230_frame.data0 = ((int)rx_buf[3] << 8) | rx_buf[4];
    k230_frame.data1 = ((int)rx_buf[5] << 8) | rx_buf[6];
    k230_frame.data2 = ((int)rx_buf[7] << 8) | rx_buf[8];
    k230_frame.data3 = ((int)rx_buf[9] << 8) | rx_buf[10];
    k230_frame.error = ((int)rx_buf[11] << 8) | rx_buf[12];

    k230_frame_ready = 1;
    k230_frame_count++;             /* 诊断: 累计接收到的帧数 */
}
