#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"

/* ==================== F32C单圈控制模块 ====================
   水平电机(ID=1), 单圈绝对位置控制 0~359.9° (发送×10 → 0~3599)
   物理范围 ±179.9°(相对零点), 避开单圈180°方向歧义点
*/

#define MOTOR_ID             1     /* 水平轴 */

/* 控制模式 */
#define MOTOR_MODE_SINGLE_T   0x0002   /* 单圈T型轨迹(平滑起停) */
#define MOTOR_MODE_SINGLE_DIR 0x0004   /* 单圈直通(即时响应) */

/* 物理范围与路径规划参数 (0.1°单位) */
#define PHYS_LIMIT        1799     /* ±179.9° 避开180°方向歧义 */
#define KEYPOINT_STEP     1200     /* 关键点间隔120° */
#define ARRIVE_THRESH     50       /* 到达判定阈值 5° */

/* 速度 (RPM, 1RPM=6°/s) */
#define SPEED_SCAN        4        /* 找靶 ~24°/s */
#define SPEED_RETURN     30        /* 回零 ~180°/s */
#define SPEED_TRACK      50        /* 跟踪 ~300°/s */

/* 全局变量 */
extern volatile int Motor1_Current_Position;  /* 单圈机械角度反馈 0~3599 (USART2中断更新) */
extern int phys_pos;                  /* 物理位置(0.1°) -1799~+1799 */

/* ====== 基础控制 ====== */
void motor_init(void);
void motor_enable(void);          /* 单独发送使能命令(防上电时序问题) */
void motor_set_mode(u16 mode);
void motor_set_speed(u8 speed);
void motor_send_position(int single_angle);  /* 单圈角度0~3599 */
void motor_send_phys(int phys);              /* 物理位置(0.1°), 内部转单圈 */
void motor_request_feedback(void);
u8   motor_reached(int target_phys);         /* 判断是否到达目标(物理位置) */
int  phys_to_single(int phys);               /* 物理位置→单圈角度 */

/* ====== 路径规划器(用于回零/跨区移动) ====== */
void planner_goto(int target_phys);   /* 设定目标,自动规划关键点路径 */
u8   planner_tick(void);              /* 每30ms调用,推进waypoint. 返回1=到达 */
u8   planner_is_active(void);
void planner_stop(void);

/* ====== 扫描器(找靶三角波) ====== */
void scan_start(int center, int start_amp, int dir, int amp_step, int max_amp);
void scan_tick(void);                 /* 每30ms调用,推进扫描 */
void scan_stop(void);
int  scan_get_dir(void);              /* 当前方向 +1/-1 (OLED显示用) */

#endif
