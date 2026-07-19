#ifndef __MOTOR_H
#define __MOTOR_H
#include "sys.h"

/* ==================== F32C二自由度云台控制模块 ====================
   X轴(水平)电机 ID=1, Y轴(俯仰)电机 ID=2
   单圈绝对位置控制 0~359.9° (发送×10 → 0~3599)
   物理范围 ±179.9°(相对零点), 避开单圈180°方向歧义点
*/

#define MOTOR_ID             1     /* X轴 水平 */
#define MOTOR2_ID           2     /* Y轴 俯仰 */

/* 控制模式 */
#define MOTOR_MODE_SINGLE_T   0x0002   /* 单圈T型轨迹(平滑起停) */
#define MOTOR_MODE_SINGLE_DIR 0x0004   /* 单圈直通(即时响应) */

/* 物理范围与路径规划参数 (0.1°单位) */
#define PHYS_LIMIT        1799     /* ±179.9° 避开180°方向歧义 */
#define KEYPOINT_STEP     1200     /* 关键点间隔120° */
#define ARRIVE_THRESH     50       /* 到达判定阈值 5° */

/* Y轴俯仰固定仰角(FIND模式): 355° = -5° = phys_pos2 = -50 */
#define PITCH_FIND_ANGLE   3550    /* 355° (单圈角度, 用于motor2_send_position) */
#define PITCH_FIND_PHYS    (-50)   /* 355° 对应phys_pos2值 (-5°) */

/* 速度 (RPM, 1RPM=6°/s) */
#define SPEED_SCAN        4        /* 找靶 ~24°/s */
#define SPEED_RETURN     30        /* 回零 ~180°/s */
#define SPEED_TRACK      50        /* 跟踪 ~300°/s */

/* ====== 全局变量 ====== */
/* X轴(水平)电机1 */
extern volatile int Motor1_Current_Position;  /* 单圈机械角度反馈 0~3599 */
extern int phys_pos;                  /* 物理位置(0.1°) -1799~+1799 */
/* Y轴(俯仰)电机2 */
extern volatile int Motor2_Current_Position;  /* 单圈机械角度反馈 0~3599 */
extern int phys_pos2;                 /* 物理位置(0.1°) -1799~+1799 */

/* ====== X轴电机1基础控制 ====== */
void motor_init(void);
void motor_enable(void);
void motor_set_mode(u16 mode);
void motor_set_speed(u8 speed);
void motor_send_position(int single_angle);
void motor_send_phys(int phys);
void motor_request_feedback(void);
u8   motor_reached(int target_phys);
int  phys_to_single(int phys);

/* ====== Y轴电机2基础控制(新增) ====== */
void motor2_init(void);
void motor2_enable(void);
void motor2_set_mode(u16 mode);
void motor2_set_speed(u8 speed);
void motor2_send_position(int single_angle);
void motor2_send_phys(int phys);
void motor2_request_feedback(void);
u8   motor2_reached(int target_phys);

/* ====== 路径规划器(X轴回零/跨区移动) ====== */
void planner_goto(int target_phys);
u8   planner_tick(void);
u8   planner_is_active(void);
void planner_stop(void);

/* ====== 扫描器(X轴找靶三角波) ====== */
void scan_start(int center, int start_amp, int dir, int amp_step, int max_amp);
void scan_tick(void);
void scan_stop(void);
int  scan_get_dir(void);

#endif
