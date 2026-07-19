#include "motor.h"
#include "usart.h"
#include "delay.h"

/* ==================== 全局变量 ==================== */
volatile int Motor1_Current_Position = 0;   /* 单圈机械角度反馈 0~3599, 由USART2中断更新 */
int phys_pos = 0;                   /* X轴物理位置(0.1°) -1799~+1799 */
volatile int Motor2_Current_Position = 0;   /* Y轴单圈机械角度反馈 0~3599 */
int phys_pos2 = 0;                  /* Y轴物理位置(0.1°) -1799~+1799 */

/* ==================== 路径规划器状态 ==================== */
#define WP_QUEUE_SIZE  8
static int wp_queue[WP_QUEUE_SIZE];
static int wp_head, wp_tail;
static int wp_current;
static u8  wp_active;

/* ==================== 扫描器状态 ==================== */
typedef struct {
    int center;      /* 扫描中心(物理位置0.1°) */
    int amp;         /* 当前振幅(0.1°) */
    int amp_step;    /* 振幅增长步长, 0=不增长 */
    int max_amp;     /* 最大振幅 */
    int dir;         /* +1/-1 */
    int current;     /* 当前端点目标(物理位置0.1°) */
    u8  active;
} scan_state_t;
static scan_state_t scan_s;

/* ==================== 工具函数 ==================== */
/* 物理位置(0.1°,可负) → 单圈角度(0~3599) */
int phys_to_single(int phys)
{
    int s = phys % 3600;
    if (s < 0) s += 3600;
    return s;
}

static int clamp_phys(int phys)
{
    if (phys >  PHYS_LIMIT) phys =  PHYS_LIMIT;
    if (phys < -PHYS_LIMIT) phys = -PHYS_LIMIT;
    return phys;
}

/* 判断电机是否到达目标(物理位置), 考虑单圈环绕 */
u8 motor_reached(int target_phys)
{
    int target_single = phys_to_single(target_phys);
    int diff = Motor1_Current_Position - target_single;
    if (diff > 1800)       diff -= 3600;
    else if (diff < -1800) diff += 3600;
    if (diff < 0) diff = -diff;
    return (u8)(diff < ARRIVE_THRESH);
}

/* ==================== 基础控制 ==================== */

/* 发送单圈位置指令: 7A ID 03 dataH dataL BCC 7B (7字节) */
void motor_send_position(int single_angle)
{
    u8 frame[7];
    if (single_angle < 0)    single_angle = 0;
    if (single_angle > 3599) single_angle = 3599;
    frame[0] = 0x7A;
    frame[1] = MOTOR_ID;
    frame[2] = 0x03;                    /* 单圈绝对位置 */
    frame[3] = (u8)((single_angle >> 8) & 0xFF);
    frame[4] = (u8)(single_angle & 0xFF);
    frame[5] = BCC_Sum1(frame, 5);
    frame[6] = 0x7B;
    USART2_SEND(frame, 7);
    delay_ms(2);                          /* F32C DMA空闲中断需帧间≥1ms间隔 */
}

/* 物理位置(0.1°) → 单圈发送 */
void motor_send_phys(int phys)
{
    phys = clamp_phys(phys);
    motor_send_position(phys_to_single(phys));
}

/* 设置控制模式: 0x0002单圈T / 0x0004单圈直通 */
void motor_set_mode(u16 mode)
{
    u8 frame[7];
    frame[0] = 0x7A;
    frame[1] = MOTOR_ID;
    frame[2] = 0x00;                    /* 模式选择 */
    frame[3] = (u8)((mode >> 8) & 0xFF);
    frame[4] = (u8)(mode & 0xFF);
    frame[5] = BCC_Sum1(frame, 5);
    frame[6] = 0x7B;
    USART2_SEND(frame, 7);
    delay_ms(2);                          /* F32C DMA空闲中断需帧间≥1ms间隔 */
}

/* 设置速度(RPM): 7A ID 01 dataH dataL BCC 7B */
void motor_set_speed(u8 speed)
{
    u8 frame[7];
    frame[0] = 0x7A;
    frame[1] = MOTOR_ID;
    frame[2] = 0x01;                    /* 速度 */
    frame[3] = 0;
    frame[4] = speed;
    frame[5] = BCC_Sum1(frame, 5);
    frame[6] = 0x7B;
    USART2_SEND(frame, 7);
    delay_ms(2);                          /* F32C DMA空闲中断需帧间≥1ms间隔 */
}

/* 请求机械角度反馈: 7A ID 0E 02 BCC 7B (6字节) */
void motor_request_feedback(void)
{
    u8 frame[6];
    frame[0] = 0x7A;
    frame[1] = MOTOR_ID;
    frame[2] = 0x0E;                    /* 数据反馈 */
    frame[3] = 0x02;                    /* 机械角度 */
    frame[4] = BCC_Sum1(frame, 4);
    frame[5] = 0x7B;
    USART2_SEND(frame, 6);
    delay_ms(2);                          /* F32C DMA空闲中断需帧间≥1ms间隔 */
}

/* ==================== 初始化序列(上电) ==================== */
/* 单独发送使能命令(防上电时序问题, 可在任意时刻重新使能) */
void motor_enable(void)
{
    u8 frame[5];
    frame[0] = 0x7A;
    frame[1] = MOTOR_ID;
    frame[2] = 0x06;                    /* 使能 */
    frame[3] = BCC_Sum1(frame, 3);
    frame[4] = 0x7B;
    USART2_SEND(frame, 5);
    delay_ms(2);
}

void motor_init(void)
{
    /* 1.使能电机 */
    motor_enable();

    /* 2.单圈T型模式: 7A 01 00 00 02 BCC 7B */
    motor_set_mode(MOTOR_MODE_SINGLE_T);
    delay_ms(2);

    /* 3.设速度 */
    motor_set_speed(SPEED_SCAN);
    delay_ms(10);

    /* 4.回零 */
    phys_pos = 0;
    motor_send_phys(0);
    delay_ms(500);

    /* 5.首次反馈 */
    motor_request_feedback();
    delay_ms(10);

    /* 清空规划器/扫描器 */
    wp_active = 0;
    scan_s.active = 0;
}

/* ==================== 路径规划器 ==================== */

/* 同区段关键点入队: 从from到to, 每KEYPOINT_STEP一个waypoint */
static void enqueue_segment(int from, int to)
{
    int step, p, next;
    step = (to > from) ? KEYPOINT_STEP : -KEYPOINT_STEP;
    p = from;
    while (p != to) {
        next = p + step;
        if (step > 0) { if (next > to) next = to; }
        else          { if (next < to) next = to; }
        if (wp_tail < WP_QUEUE_SIZE) {
            wp_queue[wp_tail++] = next;
        }
        p = next;
    }
}

/* 设定目标, 自动规划关键点路径(跨区先回0) */
void planner_goto(int target_phys)
{
    int cur;

    target_phys = clamp_phys(target_phys);
    cur = phys_pos;
    wp_head = 0;
    wp_tail = 0;

    /* 跨区: 先经当前区关键点到0, 再经目标区关键点到target */
    if ((cur > 0 && target_phys < 0) || (cur < 0 && target_phys > 0)) {
        enqueue_segment(cur, 0);
        cur = 0;
        enqueue_segment(0, target_phys);
    } else {
        enqueue_segment(cur, target_phys);
    }

    if (wp_tail == 0) {
        wp_active = 0;
        return;
    }
    wp_active = 1;
    wp_current = wp_queue[0];
    motor_send_phys(wp_current);
}

/* 每30ms调用, 推进waypoint. 返回1=到达最终目标 */
u8 planner_tick(void)
{
    if (!wp_active) return 1;
    if (motor_reached(wp_current)) {
        phys_pos = wp_current;
        wp_head++;
        if (wp_head >= wp_tail) {
            wp_active = 0;
            return 1;
        }
        wp_current = wp_queue[wp_head];
        motor_send_phys(wp_current);
    }
    return 0;
}

u8 planner_is_active(void) { return wp_active; }
void planner_stop(void)     { wp_active = 0; }

/* ==================== 扫描器(三角波找靶) ==================== */

void scan_start(int center, int start_amp, int dir, int amp_step, int max_amp)
{
    scan_s.center    = center;
    scan_s.amp       = start_amp;
    scan_s.amp_step  = amp_step;
    scan_s.max_amp   = max_amp;
    scan_s.dir       = (dir >= 0) ? 1 : -1;
    scan_s.current   = clamp_phys(center + scan_s.dir * start_amp);
    scan_s.active    = 1;
    motor_send_phys(scan_s.current);
}

void scan_tick(void)
{
    if (!scan_s.active) return;
    if (motor_reached(scan_s.current)) {
        phys_pos = scan_s.current;
        /* 到端点, 反向 */
        scan_s.dir = -scan_s.dir;
        /* 扩大振幅 */
        if (scan_s.amp_step > 0) {
            scan_s.amp += scan_s.amp_step;
            if (scan_s.amp > scan_s.max_amp) scan_s.amp = scan_s.max_amp;
        }
        scan_s.current = clamp_phys(scan_s.center + scan_s.dir * scan_s.amp);
        motor_send_phys(scan_s.current);
    }
}

void scan_stop(void) { scan_s.active = 0; }
int  scan_get_dir(void) { return scan_s.dir; }

/* ==================== Y轴电机2基础控制(新增) ==================== */

/* 判断Y轴电机是否到达目标(物理位置), 考虑单圈环绕 */
u8 motor2_reached(int target_phys)
{
    int target_single = phys_to_single(target_phys);
    int diff = Motor2_Current_Position - target_single;
    if (diff > 1800)       diff -= 3600;
    else if (diff < -1800) diff += 3600;
    if (diff < 0) diff = -diff;
    return (u8)(diff < ARRIVE_THRESH);
}

/* Y轴发送单圈位置指令: 7A 02 03 dataH dataL BCC 7B */
void motor2_send_position(int single_angle)
{
    u8 frame[7];
    if (single_angle < 0)    single_angle = 0;
    if (single_angle > 3599) single_angle = 3599;
    frame[0] = 0x7A;
    frame[1] = MOTOR2_ID;
    frame[2] = 0x03;
    frame[3] = (u8)((single_angle >> 8) & 0xFF);
    frame[4] = (u8)(single_angle & 0xFF);
    frame[5] = BCC_Sum1(frame, 5);
    frame[6] = 0x7B;
    USART2_SEND(frame, 7);
    delay_ms(2);
}

/* Y轴物理位置(0.1°) → 单圈发送 */
void motor2_send_phys(int phys)
{
    phys = clamp_phys(phys);
    motor2_send_position(phys_to_single(phys));
}

/* Y轴设置控制模式 */
void motor2_set_mode(u16 mode)
{
    u8 frame[7];
    frame[0] = 0x7A;
    frame[1] = MOTOR2_ID;
    frame[2] = 0x00;
    frame[3] = (u8)((mode >> 8) & 0xFF);
    frame[4] = (u8)(mode & 0xFF);
    frame[5] = BCC_Sum1(frame, 5);
    frame[6] = 0x7B;
    USART2_SEND(frame, 7);
    delay_ms(2);
}

/* Y轴设置速度 */
void motor2_set_speed(u8 speed)
{
    u8 frame[7];
    frame[0] = 0x7A;
    frame[1] = MOTOR2_ID;
    frame[2] = 0x01;
    frame[3] = 0;
    frame[4] = speed;
    frame[5] = BCC_Sum1(frame, 5);
    frame[6] = 0x7B;
    USART2_SEND(frame, 7);
    delay_ms(2);
}

/* Y轴请求机械角度反馈 */
void motor2_request_feedback(void)
{
    u8 frame[6];
    frame[0] = 0x7A;
    frame[1] = MOTOR2_ID;
    frame[2] = 0x0E;
    frame[3] = 0x02;
    frame[4] = BCC_Sum1(frame, 4);
    frame[5] = 0x7B;
    USART2_SEND(frame, 6);
    delay_ms(2);
}

/* Y轴使能 */
void motor2_enable(void)
{
    u8 frame[5];
    frame[0] = 0x7A;
    frame[1] = MOTOR2_ID;
    frame[2] = 0x06;
    frame[3] = BCC_Sum1(frame, 3);
    frame[4] = 0x7B;
    USART2_SEND(frame, 5);
    delay_ms(2);
}

/* Y轴初始化序列(上电) */
void motor2_init(void)
{
    /* 1.使能 */
    motor2_enable();
    /* 2.单圈T型模式 */
    motor2_set_mode(MOTOR_MODE_SINGLE_T);
    delay_ms(2);
    /* 3.设速度 */
    motor2_set_speed(SPEED_SCAN);
    delay_ms(10);
    /* 4.回零 */
    phys_pos2 = 0;
    motor2_send_phys(0);
    delay_ms(500);
    /* 5.首次反馈 */
    motor2_request_feedback();
    delay_ms(10);
}
