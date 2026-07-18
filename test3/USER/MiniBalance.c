/***************************************************************
  2025电赛E题 — STM32固件 (单水平轴激光对靶)
  四态状态机: STANDBY → FIND → ALIGN → AIM
  F32C单圈绝对位置控制, K230只接收不发送

  硬件:
    UART1(PA9/PA10)  ↔ K230视觉模块
    UART2(PA2/PA3)   ↔ F32C水平电机(ID=1)
    PA5 按键, OLED(PB3/PB4/PB5/PA15)
***************************************************************/
#include "sys.h"
#include "pid.h"
#include "k230_proto.h"
#include "motor.h"

/* ==================== 兼容性全局变量(sys.h extern) ==================== */
int Encoder_cnt, Encoder_pr;
int Motor1_Speed = 50, Motor2_Speed = 50;
int motor1_Current_Speed, motor2_Current_Speed;
int Motor1_T_Position = 0, Motor2_T_Position = 0;
int Motor2_Current_Position;
/* Motor1_Current_Position 在 motor.c 中定义 */

/* ==================== PID ====================
   注意: ALIGN/AIM使用增量式结构 phys_pos += out,
   位置累加本身就是积分运算, 若PID含Ki会导致双重积分
   (phys_pos = Kp*∫e + Ki*∫∫e), 必然超调振荡.
   因此 Ki 必须为 0, 仅用PD控制.

   参数标定(靶宽30cm @ 距离>1m):
     靶视角≈17°(半宽8.5°), K230 FOV≈60° → 1像素≈0.19°
     靶心最大偏差≈±45像素, 对应±8.5°
     每拍最大移动1.5°(PID_MAX_OUT=15), 约6拍收敛, 温和不超调 */
PID_t pid_azimuth;
#define KP_Z         0.5f    /* 温和增益: 45像素→22.5→限幅15→1.5°/拍 */
#define KI_Z         0.0f    /* 增量式结构禁用积分项, 否则双重积分振荡 */
#define KD_Z         0.2f    /* 适度阻尼抑超调 */
#define PID_MAX_OUT  15      /* 每拍最多1.5°, 远小于靶半宽8.5°, 防超调 */
#define ERR_DEADBAND 3

/* ==================== 工作模式 ==================== */
enum {
    MODE_STANDBY = 0,
    MODE_FIND,        /* 找靶(初次/二次由find_secondary区分) */
    MODE_ALIGN,       /* 对靶找光 */
    MODE_AIM,         /* 瞄准 */
    MODE_RETURNING    /* 按键回零中 */
};
static u8 mode = MODE_STANDBY;
static u8 find_secondary = 0;   /* 0=初次找靶, 1=二次找靶 */
static u32 loop_cnt = 0;

/* ==================== OLED显示(模式 + K230接收诊断) ==================== */
static void update_oled(void)
{
    const u8 *str;
    u32 cnt;
    u8  fl;
    switch (mode) {
        case MODE_STANDBY:   str = (const u8 *)"STANDBY"; break;
        case MODE_FIND:      str = find_secondary ? (const u8 *)"<FIND" : (const u8 *)"FIND>"; break;
        case MODE_ALIGN:     str = (const u8 *)"ALIGN";   break;
        case MODE_AIM:       str = (const u8 *)"AIM";     break;
        case MODE_RETURNING: str = (const u8 *)"RETURN";  break;
        default:             str = (const u8 *)"??????";  break;
    }
    fl  = k230_frame.flags;
    cnt = k230_frame_count;

    OLED_Clear();               /* 清空 GRAM 并推送空屏 */
    OLED_ShowString(0, 0, str); /* 第1行: 模式字符串 */
    /* 第2行: K230接收诊断
       N=累计接收帧数(每秒应+约30), F=最近flags(0=SCAN/无靶 1=ALIGN 3=AIM) */
    OLED_ShowString(0,  16, (const u8 *)"N");
    OLED_ShowNumber(12, 16, cnt, 5, 12);
    OLED_ShowString(66, 16, (const u8 *)"F");
    OLED_ShowNumber(78, 16, fl,  1, 12);
    OLED_Refresh_Gram();        /* 仅推送 GRAM, 不追加电机信息(避免覆盖模式字符串) */
}

/* ==================== D0历史(二次找靶方向判定) ==================== */
#define D0_HIST_LEN 3
static int d0_hist[D0_HIST_LEN];
static u8  d0_hist_idx = 0;

static void push_d0(int d0)
{
    d0_hist[d0_hist_idx] = d0;
    d0_hist_idx = (d0_hist_idx + 1) % D0_HIST_LEN;
}

/* 靶丢失前最后3帧D0均值: >320(靶在右)→需右转
 * 实测F32C方向反向: phys_pos减=电机右转, 故dir=-1朝靶方向 */
static int determine_scan_dir(void)
{
    int sum = d0_hist[0] + d0_hist[1] + d0_hist[2];
    int avg = sum / D0_HIST_LEN;
    return (avg > 320) ? -1 : 1;
}

/* ==================== 模式切换函数 ==================== */
static void enter_standby(void)
{
    mode = MODE_STANDBY;
    motor_set_mode(MOTOR_MODE_SINGLE_T);
    motor_set_speed(SPEED_SCAN);
    PID_Reset(&pid_azimuth);
    update_oled();
}

static void enter_find_initial(void)
{
    mode = MODE_FIND;
    find_secondary = 0;
    motor_enable();                          /* 重新使能, 防上电时序问题 */
    motor_set_mode(MOTOR_MODE_SINGLE_T);
    motor_set_speed(SPEED_SCAN);
    PID_Reset(&pid_azimuth);
    /* 初次: center=0, amp=80°, 不增长, dir=+1 */
    scan_start(0, 800, 1, 0, 800);
    update_oled();
}

static void enter_find_secondary(void)
{
    int dir;
    mode = MODE_FIND;
    find_secondary = 1;
    motor_set_mode(MOTOR_MODE_SINGLE_T);
    motor_set_speed(SPEED_SCAN);
    PID_Reset(&pid_azimuth);
    dir = determine_scan_dir();
    /* 二次: center=当前位置, amp=20°起, 逐步扩大, max=179.9°(避免180°歧义) */
    scan_start(phys_pos, 200, dir, 200, PHYS_LIMIT);
    update_oled();
}

static void enter_align(void)
{
    int cur_single;
    /* 关键修复: FIND扫描中途切换时, phys_pos还是上次端点旧值,
     * 而电机实际位置可能在扫描中途. 必须用反馈值同步, 否则
     * ALIGN第一帧motor_send_phys(旧phys_pos+PID_out)会让电机
     * 从实际位置"跳"到旧基准+PID输出, 方向可能正好与靶反向 */
    cur_single = Motor1_Current_Position;
    phys_pos = (cur_single > 1800) ? (cur_single - 3600) : cur_single;
    if (phys_pos >  PHYS_LIMIT) phys_pos =  PHYS_LIMIT;
    if (phys_pos < -PHYS_LIMIT) phys_pos = -PHYS_LIMIT;

    mode = MODE_ALIGN;
    motor_set_mode(MOTOR_MODE_SINGLE_DIR);  /* 直通, 响应快 */
    motor_set_speed(SPEED_TRACK);
    PID_Reset(&pid_azimuth);
    update_oled();
}

static void enter_aim(void)
{
    mode = MODE_AIM;
    /* 保持直通模式 */
    motor_set_speed(SPEED_TRACK);
    PID_Reset(&pid_azimuth);
    update_oled();
}

/* 按键: STANDBY→启动初次找靶 / 其他模式→急停回零 */
static void handle_button(void)
{
    /* 待机模式: 按键启动初次找靶 */
    if (mode == MODE_STANDBY) {
        enter_find_initial();
        return;
    }
    /* 其他模式: 急停回零 */
    scan_stop();
    planner_stop();
    PID_Reset(&pid_azimuth);
    motor_set_mode(MOTOR_MODE_SINGLE_T);
    motor_set_speed(SPEED_RETURN);
    planner_goto(0);
    mode = MODE_RETURNING;
    update_oled();
}

/* ==================== K230帧处理(模式相关) ==================== */
static void process_k230_frame(void)
{
    u8  flags    = k230_frame.flags;
    u8  has_rect  = flags & 0x01;
    u8  has_laser = flags & 0x02;
    int d0 = k230_frame.data0;
    int d2 = k230_frame.data2;
    int e;
    float out;

    /* 更新D0历史(仅靶可见时) */
    if (has_rect) {
        push_d0(d0);
    }

    if (mode == MODE_STANDBY) {
        /* 待机: 只收帧, 不切模式 (要按按键才进找靶) */
        return;
    }

    if (mode == MODE_RETURNING) {
        /* 回零中: 不处理模式切换 */
        return;
    }

    if (mode == MODE_FIND) {
        /* 找靶: 看到靶→ALIGN */
        if (has_rect) {
            scan_stop();
            enter_align();
        }
        return;
    }

    if (mode == MODE_ALIGN) {
        /* 对靶找光 */
        if (!has_rect) {
            /* 靶丢 → 二次找靶 */
            enter_find_secondary();
        } else if (has_laser) {
            /* 激光出现 → AIM */
            enter_aim();
        } else {
            /* 有靶无激光: PID把靶拉到画面中心(320)
             * 实测F32C方向反向: phys_pos增=电机左转(非右转)
             * D0>320(靶在右) -> 需右转 -> phys_pos减 -> e=320-D0<0 */
            e = 320 - d0;
            if (e > -ERR_DEADBAND && e < ERR_DEADBAND) e = 0;
            out = PID_Update(&pid_azimuth, (float)e);
            phys_pos += (int)out;
            if (phys_pos >  PHYS_LIMIT) phys_pos =  PHYS_LIMIT;
            if (phys_pos < -PHYS_LIMIT) phys_pos = -PHYS_LIMIT;
            motor_send_phys(phys_pos);
        }
        return;
    }

    if (mode == MODE_AIM) {
        /* 瞄准: 持续运行抗底座扰动 */
        if (!has_rect) {
            /* 靶丢 → 二次找靶 */
            enter_find_secondary();
        } else if (!has_laser) {
            /* 激光丢 → 对靶找光 */
            enter_align();
        } else {
            /* 有靶有激光: PID让光斑D2趋近靶心D0
             * 实测F32C方向反向: phys_pos增=电机左转(非右转)
             * D2>D0(光斑偏右) -> 需左转 -> phys_pos增 -> e=D2-D0>0 */
            e = d2 - d0;
            if (e > -ERR_DEADBAND && e < ERR_DEADBAND) e = 0;
            out = PID_Update(&pid_azimuth, (float)e);
            phys_pos += (int)out;
            if (phys_pos >  PHYS_LIMIT) phys_pos =  PHYS_LIMIT;
            if (phys_pos < -PHYS_LIMIT) phys_pos = -PHYS_LIMIT;
            motor_send_phys(phys_pos);
        }
        return;
    }
}

/* ==================== 主程序 ==================== */
int main(void)
{
    /* ----- 系统初始化 ----- */
    MY_NVIC_PriorityGroupConfig(2);
    delay_init();
    JTAG_Set(JTAG_SWD_DISABLE);
    JTAG_Set(SWD_ENABLE);
    uart_init(115200);          /* UART1: K230 */
    uart2_init(115200);         /* UART2: F32C电机 */
    OLED_Init();
    KEY_Init();
    usart2_send(0x00);          /* 防首字节丢失 */
    delay_ms(1500);             /* 等电机上电就绪 */

    /* ----- PID初始化 ----- */
    PID_Init(&pid_azimuth, KP_Z, KI_Z, KD_Z, PID_MAX_OUT);

    /* ----- K230协议初始化 ----- */
    k230_proto_init();

    /* ===== F32C电机初始化(使能/单圈T型/速度/回零) ===== */
    motor_init();

    /* ----- 显示启动信息 ----- */
    OLED_Clear();
    OLED_ShowString(0, 0, (const u8 *)"2025E Ready");
    OLED_Refresh_Gram();

    mode = MODE_STANDBY;
    update_oled();
    loop_cnt = 0;

    /* ==================== 主循环 ==================== */
    while (1)
    {
        loop_cnt++;

        /* ---- 1. 处理K230帧 ---- */
        if (k230_frame_ready) {
            k230_frame_ready = 0;
            process_k230_frame();
        }

        /* ---- 2. 推进路径/扫描 ---- */
        if (mode == MODE_RETURNING) {
            if (planner_tick()) {
                /* 到达0° → 待机 */
                enter_standby();
            }
        } else if (mode == MODE_FIND) {
            scan_tick();
        }
        /* ALIGN/AIM: 每帧PID已在process_k230_frame中处理 */

        /* ---- 3. 请求电机反馈(~150ms) ---- */
        if (loop_cnt % 30 == 0) {
            motor_request_feedback();
        }

        /* ---- 4. 待机重发0°(~1s) ---- */
        if (mode == MODE_STANDBY && (loop_cnt % 200 == 0)) {
            motor_send_phys(0);
        }

        /* ---- 5. 按键检测 ---- */
        if (click() == 1) {
            handle_button();
            delay_ms(200);  /* 消抖 */
        }

        /* ---- 6. OLED刷新(~1s) ---- */
    if (loop_cnt % 200 == 0) {
        update_oled();
    }

        if (loop_cnt > 60000) loop_cnt = 0;

        delay_ms(5);
    }
}
