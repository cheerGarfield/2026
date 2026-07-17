'''
实验名称：2025电赛E题 阶段5 - 蓝紫激光光斑检测 + 画圆轨迹
功能：在阶段4双向通信基础上，实现发挥部分(3)的画圆轨迹生成
硬件：K230 + ST7701触摸屏 + 摄像头 + 蓝紫激光笔(405nm, VCC+GND直驱)
改动：
  - 蓝紫激光笔(405nm) VCC+GND直驱，接电即亮，无需GPIO控制
  - 光斑检测改为蓝紫LAB阈值，彻底解决与靶上红色圆混淆问题
  - 阈值UI改为"黑色门限"和"光斑门限(蓝紫)"
  - 保留CIRCLE画圆模式、矩形检测、双向通信、模式切换
  - 保留长按触摸屏进入阈值调节、自动AIM↔SCAN切换
格式参考：k230-master/14_脱机调阈值.py
'''

import time
import os
import math
from media.sensor import *
from media.display import *
from media.media import *
from machine import TOUCH
from ybUtils.YbUart import YbUart

sensor = None
uart = None

# ==================== 配置参数 ====================
black = (0, 95)                # 黑色二值化阈值

# 蓝紫激光光斑灰度阈值（亮度检测，比LAB颜色检测鲁棒得多）
# 激光光斑在画面上是最亮的小点，用灰度阈值直接找亮斑即可
# 只要调这一个值：调太高漏检光斑，调太低会把反光也当光斑
SPOT_GRAY_MIN = 180            # 光斑最低灰度值(0~255)，越大越严格
SPOT_GRAY_MAX = 255            # 通常保持255不变

# ===== 画圆参数（基于矩形尺寸推算，无需检测红色圆） =====
# 靶纸黑色胶带外框物理尺寸（cm），从矩形检测结果推算像素/cm比例
RECT_WIDTH_CM = 25.0          # 靶纸黑色外框物理宽度(cm) — 圆偏小则调小此值
RECT_HEIGHT_CM = 17.5         # 靶纸黑色外框物理高度(cm) — 圆偏小则调小此值
CIRCLE_RADIUS_CM = 6.0        # 目标红色圆物理半径(cm)
CIRCLE_PHASE_OFFSET_DEG = 0   # 画圆起始相位偏移(度)

# 阈值持久化文件路径
THRESHOLD_FILE = "/sdcard/thresholds.txt"

# 矩形滤波参数
MIN_AREA = 5000
MAX_AREA = 100000
MAX_ANGLE_ERROR = 45
AVG_ANGLE_ERROR = 30
MIN_BLACK_RATIO = 0.25

FIND_RECTS_THRESHOLD = 8000
ERODE_TIMES = 1
DILATE_TIMES = 2

AREA_TOLERANCE = 0.4

MIN_DETECT_FRAMES = 3
MIN_LOST_FRAMES = 6

# 光斑检测参数（蓝紫激光在UV纸上的光斑，比红圆小得多）
SPOT_MIN_PIXELS = 3
SPOT_MAX_PIXELS = 800        # 蓝紫光斑面积远小于红色圆，不会被红色圆干扰
SPOT_AREA_THRESHOLD = 3

LONG_PRESS_THRESHOLD = 20

SCREEN_W = 640
SCREEN_H = 480

CUT_ROI = (160, 120, 320, 240)

# 自动模式切换参数
AUTO_SCAN_LOST_FRAMES = 15
AUTO_SCAN_FOUND_FRAMES = 3

# UART配置
UART_BAUDRATE = 115200
UART_SEND_INTERVAL_MS = 30

# ==================== 工作模式 ====================
MODE_AIM    = 0
MODE_SCAN   = 1
MODE_CIRCLE = 2

CMD_SET_MODE_AIM    = 0x01
CMD_SET_MODE_SCAN   = 0x02
CMD_SET_MODE_CIRCLE = 0x03
CMD_LASER_ON        = 0x10
CMD_LASER_OFF       = 0x11
CMD_SET_CIRCLE_PROG = 0x20


# ==================== 阈值持久化 ====================
def save_thresholds():
    """保存当前阈值到文件"""
    try:
        f = open(THRESHOLD_FILE, "w")
        f.write("black=%d,%d\n" % black)
        f.write("spot_gray=%d\n" % SPOT_GRAY_MIN)
        f.close()
        print("阈值已保存: %s" % THRESHOLD_FILE)
    except Exception as e:
        print("阈值保存失败:", e)


def load_thresholds():
    """从文件加载阈值，成功返回True"""
    global black, SPOT_GRAY_MIN
    try:
        f = open(THRESHOLD_FILE, "r")
        for line in f:
            line = line.strip()
            if line.startswith("black="):
                parts = line[6:].split(",")
                black = (int(parts[0]), int(parts[1]))
            elif line.startswith("spot_gray="):
                SPOT_GRAY_MIN = int(line[10:])
        f.close()
        print("阈值已加载: black=%s spot_gray=%d" % (black, SPOT_GRAY_MIN))
        return True
    except Exception as e:
        print("阈值文件不存在 (%s)，使用默认值" % e)
        return False


# ==================== 工具函数 ====================
def vector_angle_diff(v1, v2):
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    det = v1[0]*v2[1] - v1[1]*v2[0]
    angle = math.atan2(det, dot) * (180 / math.pi)
    return abs(angle)


def get_line_intersection(line1, line2):
    (x1, y1), (x2, y2) = line1
    (x3, y3), (x4, y4) = line2
    A1 = y2 - y1
    B1 = x1 - x2
    C1 = A1 * x1 + B1 * y1
    A2 = y4 - y3
    B2 = x3 - x4
    C2 = A2 * x3 + B2 * y3
    det = A1 * B2 - A2 * B1
    if det == 0:
        return ((x1 + x3) / 2, (y1 + y3) / 2)
    x = (B2 * C1 - B1 * C2) / det
    y = (A1 * C2 - A2 * C1) / det
    return (x, y)


def shoe_lace_area(corners):
    area = 0
    n = len(corners)
    for i in range(n):
        x1, y1 = corners[i]
        x2, y2 = corners[(i + 1) % n]
        area += (x1 * y2 - x2 * y1)
    return abs(area) / 2


def calculate_angle_errors(corners):
    errors = []
    max_err = 0
    for i in range(4):
        p0 = corners[(i - 1) % 4]
        p1 = corners[i]
        p2 = corners[(i + 1) % 4]
        vec1 = (p0[0] - p1[0], p0[1] - p1[1])
        vec2 = (p2[0] - p1[0], p2[1] - p1[1])
        angle_diff = vector_angle_diff(vec1, vec2)
        err = abs(angle_diff - 90)
        errors.append(err)
        if err > max_err:
            max_err = err
    avg_err = sum(errors) / len(errors)
    return max_err, avg_err


def check_center_black_ratio(img_binary, cx, cy, rect_w, rect_h):
    half_w = max(3, min(10, rect_w // 4))
    half_h = max(3, min(10, rect_h // 4))
    x_start = max(0, cx - half_w)
    x_end = min(img_binary.width() - 1, cx + half_w)
    y_start = max(0, cy - half_h)
    y_end = min(img_binary.height() - 1, cy + half_h)
    valid = 0
    total = 0
    step = 2
    for y in range(y_start, y_end, step):
        for x in range(x_start, x_end, step):
            pixel = img_binary.get_pixel(x, y)
            if isinstance(pixel, tuple):
                pixel = pixel[0]
            if pixel == 0:
                valid += 1
            total += 1
    if total == 0:
        return 0.0
    return valid / total


# ==================== 光斑检测（灰度亮度法） ====================
def detect_laser_spot(img, rect_corners=None):
    """
    检测激光光斑：灰度亮度法。
    激光光斑是画面中最亮的点，直接用灰度阈值找，不依赖颜色。
    比LAB颜色检测可靠得多，不受激光波长和摄像机色彩响应影响。
    
    如果提供了矩形角点rect_corners，仅在矩形区域内搜索（排除环境反光干扰）。
    """
    # 转为灰度图
    gray = img.to_grayscale(copy=False)
    
    # 如果有矩形区域，限定ROI搜索（排除矩形外的反光）
    if rect_corners is not None and len(rect_corners) == 4:
        xs = [p[0] for p in rect_corners]
        ys = [p[1] for p in rect_corners]
        rx, ry = min(xs), min(ys)
        rw, rh = max(xs) - rx, max(ys) - ry
        if rw > 10 and rh > 10:
            gray = gray.copy(roi=(max(0,rx), max(0,ry), rw, rh))
    
    # 找灰度阈值内的亮斑
    blobs = gray.find_blobs([(SPOT_GRAY_MIN, SPOT_GRAY_MAX)],
                            pixels_threshold=SPOT_MIN_PIXELS,
                            area_threshold=SPOT_AREA_THRESHOLD,
                            x_stride=1, y_stride=1,
                            margin=False)
    if not blobs:
        return None
    
    # 取最亮的（面积最大的通常就是光斑）
    best = max(blobs, key=lambda b: b.pixels())
    if best.pixels() > SPOT_MAX_PIXELS:
        return None
    
    # 如果做了ROI裁切，坐标需要偏移回原图
    if rect_corners is not None and len(rect_corners) == 4:
        xs = [p[0] for p in rect_corners]
        ys = [p[1] for p in rect_corners]
        rx, ry = min(xs), min(ys)
        rw, rh = max(xs) - rx, max(ys) - ry
        if rw > 10 and rh > 10:
            # 构造一个新的blob-like对象，坐标偏移回原图
            class SpotResult:
                pass
            result = SpotResult()
            result._x = best.x() + rx
            result._y = best.y() + ry
            result._w = best.w()
            result._h = best.h()
            result._pixels = best.pixels()
            result.x = lambda: result._x
            result.y = lambda: result._y
            result.w = lambda: result._w
            result.h = lambda: result._h
            result.pixels = lambda: result._pixels
            return result
    
    return best


# ==================== 圆形计算（基于矩形+物理尺寸推算） ====================
def calc_circle_radius_px(rect_w_px, rect_h_px):
    """从矩形像素尺寸推算6cm圆的像素半径"""
    if rect_w_px < 10 or rect_h_px < 10:
        return 50  # 兜底值
    scale_x = rect_w_px / RECT_WIDTH_CM   # 像素/cm
    scale_y = rect_h_px / RECT_HEIGHT_CM
    scale = (scale_x + scale_y) / 2.0
    return int(CIRCLE_RADIUS_CM * scale)


def calc_circle_point(cx, cy, radius_px, progress_pct):
    """根据进度(0~100%)计算圆上目标点"""
    angle_deg = (progress_pct / 100.0) * 360.0 + CIRCLE_PHASE_OFFSET_DEG
    angle_rad = math.radians(angle_deg)
    tx = int(cx + radius_px * math.cos(angle_rad))
    ty = int(cy + radius_px * math.sin(angle_rad))
    return tx, ty


# ==================== UART 发送帧 ====================
def pack_frame(data0, data1, data2, data3, error, flags):
    data = bytearray(14)
    data[0] = 0xAA
    data[1] = 0x55
    data[2] = flags & 0x07
    data[3] = (data0 >> 8) & 0xFF
    data[4] = data0 & 0xFF
    data[5] = (data1 >> 8) & 0xFF
    data[6] = data1 & 0xFF
    data[7] = (data2 >> 8) & 0xFF
    data[8] = data2 & 0xFF
    data[9] = (data3 >> 8) & 0xFF
    data[10] = data3 & 0xFF
    data[11] = (error >> 8) & 0xFF
    data[12] = error & 0xFF
    checksum = 0
    for i in range(2, 13):
        checksum ^= data[i]
    data[13] = checksum & 0xFF
    return bytes(data)


# ==================== UART 接收 ====================
RX_WAIT_H1 = 0
RX_WAIT_H2 = 1
RX_WAIT_DATA = 2


def parse_cmd_frame():
    global rx_state, rx_buf, rx_idx, rx_call_count
    raw = uart.read()
    rx_call_count += 1
    if raw is None or len(raw) == 0:
        return None, None
    for b in raw:
        if isinstance(b, str):
            b = ord(b)
        if rx_state == RX_WAIT_H1:
            if b == 0xBB:
                rx_state = RX_WAIT_H2
        elif rx_state == RX_WAIT_H2:
            if b == 0x66:
                rx_buf[0] = 0xBB
                rx_buf[1] = 0x66
                rx_idx = 2
                rx_state = RX_WAIT_DATA
            else:
                rx_state = RX_WAIT_H1
        elif rx_state == RX_WAIT_DATA:
            rx_buf[rx_idx] = b
            rx_idx += 1
            if rx_idx >= 5:
                rx_state = RX_WAIT_H1
                calc = rx_buf[2] ^ rx_buf[3]
                if calc == rx_buf[4]:
                    print("[UART] 帧 cmd=0x{:02X} p={}".format(rx_buf[2], rx_buf[3]))
                    return rx_buf[2], rx_buf[3]
    return None, None


rx_state = RX_WAIT_H1
rx_buf = bytearray(5)
rx_idx = 0
rx_call_count = 0


# ==================== 阈值UI ====================
def show_ui_to_screen(canvas):
    canvas.compress_for_ide()
    Display.show_image(canvas, x=(SCREEN_W - canvas.width()) // 2, y=(SCREEN_H - canvas.height()) // 2)


def run_threshold_ui(tp):
    global black, SPOT_GRAY_MIN

    # mode: 0=黑色, 1=光斑灰度
    mode = 0
    # mode0用2个值(Min,Max), mode1用1个值(灰度下限)
    vals = list(black) if mode == 0 else [SPOT_GRAY_MIN]

    btn_color = (150, 150, 150)
    txt_color = (0, 0, 0)

    black_labels = ["Min", "Max"]
    spot_labels  = ["GrayMin"]    # 只有一个灰度下限

    row_ys = [60, 115, 170, 225, 280, 335]
    BTN_L_X, BTN_W, BTN_H = 5, 110, 50
    BTN_R_X = 525

    ui = image.Image(SCREEN_W, SCREEN_H, image.RGB565)

    titles = ["黑色门限", "光斑灰度(亮度)"]

    def draw_static():
        ui.draw_rectangle(0, 0, SCREEN_W, SCREEN_H, color=(255, 255, 255), fill=True)
        ui.draw_string_advanced(220, 8, 30, titles[mode], color=(0, 0, 0))
        ui.draw_rectangle(80, 425, 160, 45, color=btn_color, fill=True)
        ui.draw_string_advanced(120, 432, 30, "切换", color=txt_color)
        ui.draw_rectangle(400, 425, 160, 45, color=btn_color, fill=True)
        ui.draw_string_advanced(450, 432, 30, "返回", color=txt_color)

    draw_static()
    show_ui_to_screen(ui)
    print("阈值UI启动, mode:%d (%s)" % (mode, titles[mode]))

    while True:
        os.exitpoint()
        img_ = sensor.snapshot(chn=CAM_CHN_ID_0)
        img_ = img_.copy(roi=CUT_ROI)

        if mode == 0:
            img_ = img_.to_grayscale()
            img_ = img_.binary([(vals[0], vals[1])])
            img_ = img_.to_rgb565()
        else:
            # mode 1: 光斑灰度 → 转灰度后用单阈值二值化
            img_ = img_.to_grayscale()
            img_ = img_.binary([(vals[0], 255)])
            img_ = img_.to_rgb565()

        ui.draw_image(img_, (SCREEN_W - img_.width()) // 2, (SCREEN_H - img_.height()) // 2)

        labels = black_labels if mode == 0 else spot_labels
        n = len(labels)
        for i in range(n):
            y = row_ys[i]
            ui.draw_rectangle(BTN_L_X, y, BTN_W, BTN_H, color=btn_color, fill=True)
            ui.draw_string_advanced(BTN_L_X + 5, y + 2, 18, labels[i], color=txt_color)
            ui.draw_string_advanced(BTN_L_X + 5, y + 24, 18, "{} -".format(vals[i]), color=txt_color)
            ui.draw_rectangle(BTN_R_X, y, BTN_W, BTN_H, color=btn_color, fill=True)
            ui.draw_string_advanced(BTN_R_X + 5, y + 2, 18, labels[i], color=txt_color)
            ui.draw_string_advanced(BTN_R_X + 5, y + 24, 18, "{} +".format(vals[i]), color=txt_color)

        show_ui_to_screen(ui)

        points = tp.read()
        if len(points) > 0:
            tx, ty = points[0].x, points[0].y
            handled = False
            n = len(labels)
            for i in range(n):
                y = row_ys[i]
                if BTN_L_X <= tx <= BTN_L_X + BTN_W and y <= ty <= y + BTN_H:
                    vals[i] -= 1
                    if mode == 0:
                        vals[i] = max(0, vals[i])
                    else:
                        vals[i] = max(0, vals[i])       # 灰度值下限0
                    handled = True
                    time.sleep_ms(120)
                    break
                if BTN_R_X <= tx <= BTN_R_X + BTN_W and y <= ty <= y + BTN_H:
                    vals[i] += 1
                    if mode == 0:
                        vals[i] = min(255, vals[i])
                    else:
                        vals[i] = min(255, vals[i])     # 灰度值上限255
                    handled = True
                    time.sleep_ms(120)
                    break

            if not handled:
                if 80 <= tx <= 240 and 425 <= ty <= 470:
                    if mode == 0:
                        black = tuple(vals)
                    else:
                        SPOT_GRAY_MIN = vals[0]
                    mode = (mode + 1) % 2
                    vals = list(black) if mode == 0 else [SPOT_GRAY_MIN]
                    draw_static()
                    time.sleep_ms(300)
                elif 400 <= tx <= 560 and 425 <= ty <= 470:
                    if mode == 0:
                        black = tuple(vals)
                    else:
                        SPOT_GRAY_MIN = vals[0]
                    save_thresholds()
                    break


# ==================== 主程序 ====================
try:
    print("2025E 阶段5 蓝紫光斑检测+画圆轨迹 启动")
    print("蓝紫激光(405nm)为VCC+GND直驱，接电即亮")

    # ===== 摄像头初始化 =====
    sensor = Sensor(width=640, height=480)
    sensor.reset()
    sensor.set_framesize(width=640, height=480)
    sensor.set_pixformat(Sensor.RGB565)

    Display.init(Display.ST7701, to_ide=True)
    MediaManager.init()
    sensor.run()

    # ===== 蓝紫激光笔说明 =====
    # 405nm蓝紫激光笔，VCC+GND两根线直驱，接电即亮
    # 无需GPIO控制，由瞄准模块独立电源开关控制通断
    print("蓝紫激光笔(405nm): VCC+GND直驱, 已接电点亮")

    uart = YbUart(baudrate=UART_BAUDRATE)
    print("UART已初始化 (YbUart), 波特率=%d" % UART_BAUDRATE)

    # 加载已保存的阈值，没有则用默认值
    load_thresholds()

    tp = TOUCH(0)
    clock = time.clock()

    # 状态变量
    flag_detected = False
    detect_counter = 0
    lost_counter = 0
    prev_corners = None
    prev_area = 0
    touch_counter = 0
    last_send_time = time.ticks_ms()

    work_mode = MODE_AIM
    laser_force = False
    circle_progress = 0
    aim_lost_frames = 0
    scan_found_frames = 0

    # 画圆模式状态
    circle_radius_px = 50        # 当前像素半径（每帧从矩形重算）
    circle_target_x = 0
    circle_target_y = 0

    # 接收状态机
    rx_state = RX_WAIT_H1
    rx_buf = bytearray(5)
    rx_idx = 0
    rx_call_count = 0

    mode_names = {MODE_AIM: "AIM", MODE_SCAN: "SCAN", MODE_CIRCLE: "CIRCLE"}

    while True:
        clock.tick()
        os.exitpoint()

        # ===== 接收MSPM0指令 =====
        cmd, param = parse_cmd_frame()
        if cmd is not None:
            if cmd == CMD_SET_MODE_AIM:
                work_mode = MODE_AIM
                aim_lost_frames = 0
                scan_found_frames = 0
                print("[RX] 模式: AIM")
            elif cmd == CMD_SET_MODE_SCAN:
                work_mode = MODE_SCAN
                aim_lost_frames = 0
                scan_found_frames = 0
                print("[RX] 模式: SCAN")
            elif cmd == CMD_SET_MODE_CIRCLE:
                work_mode = MODE_CIRCLE
                circle_progress = 0
                aim_lost_frames = 0
                scan_found_frames = 0
                print("[RX] 模式: CIRCLE")
            elif cmd == CMD_LASER_ON:
                laser_force = True
                print("[RX] 强制开激光(蓝紫激光VCC直驱,已常亮)")
            elif cmd == CMD_LASER_OFF:
                laser_force = False
                print("[RX] 强制关激光(蓝紫激光VCC直驱,靠独立开关断电)")
            elif cmd == CMD_SET_CIRCLE_PROG:
                circle_progress = param

        # ===== 长按进调阈值 =====
        points = tp.read()
        if len(points) > 0:
            touch_counter += 1
            if touch_counter > LONG_PRESS_THRESHOLD:
                print("进入阈值调节")
                run_threshold_ui(tp)
                touch_counter = 0
        else:
            touch_counter = max(0, touch_counter - 1)

        # ===== 拍照 =====
        img = sensor.snapshot(chn=CAM_CHN_ID_0)

        # ===== 矩形检测 =====
        img_binary = img.to_grayscale(copy=True)
        img_binary = img_binary.binary([black])
        img_binary.erode(ERODE_TIMES)
        img_binary.dilate(DILATE_TIMES)

        rects = img_binary.find_rects(threshold=FIND_RECTS_THRESHOLD)

        min_area = float('inf')
        min_corners = None

        if rects is not None:
            for rect in rects:
                corners = rect.corners()
                if len(corners) != 4:
                    continue
                area = shoe_lace_area(corners)
                if area < MIN_AREA or area > MAX_AREA:
                    continue
                max_err, avg_err = calculate_angle_errors(corners)
                if max_err > MAX_ANGLE_ERROR or avg_err > AVG_ANGLE_ERROR:
                    continue
                center = get_line_intersection(
                    [corners[0], corners[2]],
                    [corners[1], corners[3]]
                )
                cx, cy = int(center[0]), int(center[1])
                ratio = check_center_black_ratio(img_binary, cx, cy, rect.w(), rect.h())
                if ratio < MIN_BLACK_RATIO:
                    continue
                if prev_area > 0:
                    if abs(area - prev_area) / prev_area < AREA_TOLERANCE:
                        if area < min_area:
                            min_area = area
                            min_corners = corners
                else:
                    if area < min_area:
                        min_area = area
                        min_corners = corners

        # ===== 状态防抖 =====
        current_has_rect = min_corners is not None
        if current_has_rect:
            if not flag_detected:
                detect_counter += 1
                if detect_counter >= MIN_DETECT_FRAMES:
                    flag_detected = True
                    detect_counter = 0
            else:
                lost_counter = 0
        else:
            if flag_detected:
                lost_counter += 1
                if lost_counter >= MIN_LOST_FRAMES:
                    flag_detected = False
                    lost_counter = 0
                    prev_area = 0
            else:
                detect_counter = 0

        if not current_has_rect and prev_corners is not None and flag_detected:
            min_corners = prev_corners

        # ===== 激光控制（蓝紫激光VCC+GND直驱，由独立电源开关控制） =====
        # 激光常亮，无需GPIO切换。激光笔电源由瞄准模块独立开关控制。
        # MSPM0的CMD_LASER_ON/OFF指令仅做日志记录，不影响实际激光状态
        if laser_force:
            pass  # 激光已由独立电源开关控制
        else:
            pass  # 保留接口兼容性，实际无需操作

        # ===== 蓝紫光斑检测（灰度亮度法，矩形内搜索） =====
        # 传入矩形角点以限定搜索区域，排除矩形外的环境反光
        spot_blob = detect_laser_spot(img, min_corners if flag_detected else None)
        dot_x, dot_y = 0, 0
        has_dot = spot_blob is not None
        if has_dot:
            dot_x = int(spot_blob.x() + spot_blob.w() / 2)
            dot_y = int(spot_blob.y() + spot_blob.h() / 2)

        # ===== 计算靶心坐标和偏差 =====
        rect_cx, rect_cy = 0, 0
        error = 0

        if min_corners is not None and flag_detected:
            if current_has_rect:
                prev_corners = min_corners
                prev_area = min_area

            center = get_line_intersection(
                [min_corners[0], min_corners[2]],
                [min_corners[1], min_corners[3]]
            )
            rect_cx, rect_cy = int(center[0]), int(center[1])

            if has_dot:
                error = int(math.sqrt((dot_x - rect_cx) ** 2 + (dot_y - rect_cy) ** 2))

        # ===== 计算圆参数（从矩形尺寸推算） =====
        circle_radius_px = 50  # 默认
        if flag_detected and rect_cx > 0 and min_corners is not None:
            rect_w = int(math.sqrt((min_corners[0][0] - min_corners[1][0])**2 +
                                   (min_corners[0][1] - min_corners[1][1])**2))
            rect_h = int(math.sqrt((min_corners[1][0] - min_corners[2][0])**2 +
                                   (min_corners[1][1] - min_corners[2][1])**2))
            circle_radius_px = calc_circle_radius_px(rect_w, rect_h)

        # ===== 模式特定处理 + 自动切换 =====
        target_found = False

        if work_mode == MODE_AIM:
            if flag_detected:
                aim_lost_frames = 0
            else:
                aim_lost_frames += 1
                if aim_lost_frames >= AUTO_SCAN_LOST_FRAMES:
                    work_mode = MODE_SCAN
                    aim_lost_frames = 0
                    scan_found_frames = 0
                    print("[AUTO] AIM→SCAN")

        elif work_mode == MODE_SCAN:
            if flag_detected and current_has_rect:
                target_found = True
                scan_found_frames += 1
                if scan_found_frames >= AUTO_SCAN_FOUND_FRAMES:
                    work_mode = MODE_AIM
                    scan_found_frames = 0
                    aim_lost_frames = 0
                    print("[AUTO] SCAN→AIM")
            else:
                scan_found_frames = 0

        elif work_mode == MODE_CIRCLE:
            # 从矩形尺寸推算圆周目标点
            if rect_cx > 0:
                circle_target_x, circle_target_y = calc_circle_point(
                    rect_cx, rect_cy, circle_radius_px, circle_progress)
            else:
                circle_target_x, circle_target_y = 0, 0

        # ===== UART发送数据帧 =====
        now = time.ticks_ms()
        if time.ticks_diff(now, last_send_time) >= UART_SEND_INTERVAL_MS:
            last_send_time = now

            flags = 0
            if flag_detected and rect_cx > 0:
                flags |= 0x01
            if has_dot:
                flags |= 0x02
            if target_found:
                flags |= 0x04

            if work_mode == MODE_AIM:
                tx_d0 = rect_cx if (flags & 0x01) else 0
                tx_d1 = rect_cy if (flags & 0x01) else 0
                tx_d2 = dot_x  if (flags & 0x02) else 0
                tx_d3 = dot_y  if (flags & 0x02) else 0
                tx_err = error if (flags & 0x03) == 0x03 else 0
            elif work_mode == MODE_SCAN:
                tx_d0, tx_d1, tx_d2, tx_d3 = 0, 0, 0, 0
                tx_err = 0
            elif work_mode == MODE_CIRCLE:
                # 发送圆上目标点坐标 + 进度
                tx_d0 = circle_target_x if rect_cx > 0 else 0
                tx_d1 = circle_target_y if rect_cx > 0 else 0
                tx_d2 = circle_progress
                tx_d3 = circle_radius_px
                tx_err = 0
            else:
                tx_d0, tx_d1, tx_d2, tx_d3, tx_err = 0, 0, 0, 0, 0

            frame = pack_frame(tx_d0, tx_d1, tx_d2, tx_d3, tx_err, flags)
            uart.send(frame)

        # ===== 绘制矩形 =====
        if min_corners is not None and flag_detected:
            for i in range(4):
                x1, y1 = min_corners[i]
                x2, y2 = min_corners[(i + 1) % 4]
                img.draw_line(x1, y1, x2, y2, color=(0, 255, 0), thickness=2)
            img.draw_circle(rect_cx, rect_cy, 3, color=(255, 255, 0), thickness=1)
            for p in min_corners:
                img.draw_circle(p[0], p[1], 4, color=(255, 0, 0), thickness=1)

        # ===== 绘制推算出的圆 =====
        if flag_detected and rect_cx > 0:
            img.draw_circle(rect_cx, rect_cy, circle_radius_px,
                            color=(0, 160, 255), thickness=2)

        # ===== 绘制画圆目标点 =====
        if work_mode == MODE_CIRCLE and circle_target_x > 0:
            img.draw_cross(circle_target_x, circle_target_y,
                          color=(255, 128, 0), size=12)
            # 画目标点与圆心的连线
            if rect_cx > 0:
                img.draw_line(circle_target_x, circle_target_y, rect_cx, rect_cy,
                              color=(255, 128, 0), thickness=1)

        # ===== 绘制蓝紫光斑 =====
        if has_dot:
            img.draw_rectangle(spot_blob.x(), spot_blob.y(), spot_blob.w(), spot_blob.h(),
                               color=(128, 0, 255), thickness=2)    # 紫色框表示蓝紫光斑
            img.draw_cross(dot_x, dot_y, color=(128, 0, 255), size=10)
            if rect_cx > 0 and rect_cy > 0:
                img.draw_line(dot_x, dot_y, rect_cx, rect_cy,
                              color=(255, 255, 0), thickness=1)
                img.draw_string_advanced(5, 85, 15, "error:{}".format(error), color=(255, 0, 0))

        # ===== 状态文字 =====
        rect_status = "Track" if flag_detected else "Lost"
        spot_status = "Spot" if has_dot else "NoSpot"
        mode_colors = {MODE_AIM: (0, 255, 0), MODE_SCAN: (0, 128, 255), MODE_CIRCLE: (255, 128, 0)}
        mc = mode_colors.get(work_mode, (255, 255, 255))

        img.draw_string_advanced(5, 5, 15, "fps:{:.1f}".format(clock.fps()), color=(255, 0, 0))
        img.draw_string_advanced(5, 25, 15, "mode:{}".format(mode_names[work_mode]), color=mc)
        img.draw_string_advanced(5, 45, 15, "rect:{}".format(rect_status), color=(0, 255, 0))
        img.draw_string_advanced(5, 65, 15, "spot:{}".format(spot_status), color=(128, 0, 255))
        img.draw_string_advanced(5, 105, 15, "R({},{}) D({},{})".format(
            rect_cx, rect_cy, dot_x, dot_y), color=(255, 255, 0))
        if work_mode == MODE_CIRCLE:
            img.draw_string_advanced(5, 125, 15, "C({},{}) r={}px".format(
                circle_target_x, circle_target_y, circle_radius_px), color=(255, 128, 0))
            img.draw_string_advanced(5, 145, 15, "prog:{}% ang:{:.0f}d".format(
                circle_progress, (circle_progress/100.0)*360), color=(255, 128, 0))
        else:
            img.draw_string_advanced(5, 125, 15, "TX:{}".format(mode_names[work_mode]), color=mc)
            img.draw_string_advanced(5, 145, 15, "circle prog:{}%".format(circle_progress), color=(200, 200, 200))
        img.draw_string_advanced(5, 165, 15, "long press to adjust", color=(200, 200, 200))
        img.draw_string_advanced(5, 185, 15, "RX:{}".format(rx_call_count), color=(200, 200, 200))
        if work_mode == MODE_AIM and not flag_detected:
            img.draw_string_advanced(5, 205, 15, "lost:{}/{}".format(aim_lost_frames, AUTO_SCAN_LOST_FRAMES), color=(255, 128, 0))
        if work_mode == MODE_SCAN and target_found:
            img.draw_string_advanced(5, 205, 15, "found:{}/{}".format(scan_found_frames, AUTO_SCAN_FOUND_FRAMES), color=(0, 255, 128))

        if work_mode == MODE_SCAN:
            scan_text = "FOUND!" if target_found else "Searching..."
            sc = (0, 255, 0) if target_found else (255, 255, 0)
            img.draw_string_advanced(200, 220, 25, scan_text, color=sc)

        # ===== 显示 =====
        if img.height() > SCREEN_H or img.width() > SCREEN_W:
            scale = max(img.height() // SCREEN_H, img.width() // SCREEN_W) + 1
            img.midpoint_pool(scale, scale)
        img.compress_for_ide()
        Display.show_image(img, x=(SCREEN_W - img.width()) // 2, y=(SCREEN_H - img.height()) // 2)

except KeyboardInterrupt as e:
    print("用户停止:", e)
except BaseException as e:
    print("异常: ", e)
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    try:
        if uart is not None:
            uart.deinit()
            print("UART已关闭")
    except:
        pass
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()


'''
==================== 联调说明 ====================

1. 蓝紫激光笔(405nm)接线：
   - VCC线 → 瞄准模块独立电源正极（通过开关控制）
   - GND线 → 瞄准模块独立电源负极
   - 接电即亮，无需GPIO控制
   - 激光功率≤10mW，波长405nm，切勿照射人眼和皮肤

2. MSPM0端工作流程：

   send_cmd(CMD_SET_MODE_CIRCLE, 0);   // 切换画圆模式

   while (car_lap_count < 1) {
       float progress = calculate_car_progress();  // 0.0~100.0
       send_cmd(CMD_SET_CIRCLE_PROG, (uint8_t)progress);
       // K230返回目标点坐标 (data0=X, data1=Y)
   }

3. 光斑检测原理：
   - 蓝紫激光(405nm)在UV感光纸上呈亮蓝紫色光斑
   - LAB空间：高L值 + 中性A + 深负B值 → 与红色圆完全分离
   - 红色圆：低L + 高正A + 中性B → 不会被误识别为光斑
   - 阈值可在长按触摸屏→"光斑门限(蓝紫)"中实时调整

4. 进度与角度关系（默认相位偏移0）：
   progress 0%  →  0° (圆右侧)     progress 25% →  90° (圆上方)
   progress 50% →  180° (圆左侧)   progress 75% →  270° (圆下方)

5. 激光控制：
   - CMD_LASER_ON/OFF 指令仅做日志记录
   - 实际激光通断由瞄准模块独立电源开关控制（题目要求）
'''
