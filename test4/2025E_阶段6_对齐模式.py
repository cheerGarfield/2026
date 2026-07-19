'''
实验名称：2025电赛E题 阶段6 对齐模式 — 四态状态机(SCAN→ALIGN→AIM) + cv_lite加速
功能：在SCAN和AIM之间增加ALIGN过渡模式，矩形锁定但激光未识别时，
     发送靶心坐标给MSPM0/STM32控制云台引导激光到靶上
硬件：K230 + ST7701触摸屏 + 摄像头 + 蓝紫激光笔(405nm)

===== 新增：四态状态机 =====

  SCAN ──[rect锁定3帧]──→ ALIGN ──[激光发现3帧]──→ AIM
    ↑                        ↑                        │
    └──[rect丢失15帧]────────└──[rect丢失15帧]─────────┘
                              ↑                        │
                              └──[激光丢失10帧]─────────┘
                                  (rect还在→回ALIGN)

  模式对比:
  ┌────────┬──────────┬──────────┬──────────────────────────────┐
  │  模式  │ 矩形靶纸 │ 激光光斑 │        UART发送内容           │
  ├────────┼──────────┼──────────┼──────────────────────────────┤
  │ SCAN   │    ✗     │    ✗     │ 全0 (队友自行搜索)           │
  │ ALIGN  │    ✓     │    ✗     │ 靶心XY → 队友控制云台引导激光 │
  │ AIM    │    ✓     │    ✓     │ 靶心XY+光斑XY+偏差 → 精确反馈 │
  │ CIRCLE │    ✓     │    ✓     │ 圆周目标点XY+进度+半径         │
  └────────┴──────────┴──────────┴──────────────────────────────┘

===== 激光笔供电 =====
  GPIO42(3.3V) → 1KΩ → S8050基极, 5V → 激光VCC, 激光GND → 集电极 → 发射极 → GND

基于：2025E_阶段5_5_cv_lite加速版
'''

import time
import os
import math
import gc
from media.sensor import *
from media.display import *
from media.media import *
from machine import FPIOA
from machine import Pin
from machine import TOUCH
from ybUtils.YbUart import YbUart

sensor = None
uart = None
laser_pin = None

try:
    import cv_lite
    HAS_CV_LITE = True
except ImportError:
    HAS_CV_LITE = False
    print("[WARN] cv_lite不可用，降级到find_rects")

# ==================== 配置参数 ====================
DETECT_W = 320
DETECT_H = 240
SCREEN_W = 640
SCREEN_H = 480

black = (0, 95)
USE_OTSU = True

SPOT_GRAY_MIN = 180
SPOT_GRAY_MAX = 255

RECT_WIDTH_CM = 25.0
RECT_HEIGHT_CM = 17.5
CIRCLE_RADIUS_CM = 6.0
CIRCLE_PHASE_OFFSET_DEG = 0

THRESHOLD_FILE = "/sdcard/thresholds.txt"

# ===== 矩形滤波参数 =====
MIN_AREA = 1000
MAX_AREA = 30000
AREA_TOLERANCE = 0.4

MIN_DETECT_FRAMES = 3
MIN_LOST_FRAMES = 6

# cv_lite 参数
CVL_CANNY_LO = 50
CVL_CANNY_HI = 150
CVL_APPROX_EPSILON = 0.04
CVL_AREA_MIN_RATIO = 0.008
CVL_MAX_ANGLE_COS = 0.3
CVL_GAUSSIAN_BLUR = 5

# find_rects 降级参数
FIND_RECTS_THRESHOLD = 2000
DILATE_TIMES = 0

# 光斑检测参数
SPOT_MIN_PIXELS = 2
SPOT_MAX_PIXELS = 200
SPOT_AREA_THRESHOLD = 2

LONG_PRESS_THRESHOLD = 20
CUT_ROI = (80, 60, 160, 120)

SKIP_COMPRESS_FOR_IDE = False

# ===== 四态状态机切换阈值 =====
AUTO_SCAN_LOST_FRAMES = 15      # SCAN←ALIGN←AIM: rect连续丢失多少帧触发回退
AUTO_SCAN_FOUND_FRAMES = 3      # SCAN→ALIGN: rect连续发现多少帧进入ALIGN
AUTO_ALIGN_FOUND_FRAMES = 3     # ALIGN→AIM:  激光连续发现多少帧进入AIM
AUTO_AIM_LASER_LOST_FRAMES = 10 # AIM→ALIGN:  激光连续丢失多少帧回退到ALIGN

UART_BAUDRATE = 115200
UART_SEND_INTERVAL_MS = 30

# ==================== 工作模式（四态） ====================
MODE_SCAN   = 0     # 扫描中: 无矩形, 无激光
MODE_ALIGN  = 1     # 对齐中: 矩形✓, 激光✗  → 队友控制云台引导激光
MODE_AIM    = 2     # 瞄准中: 矩形✓, 激光✓  → 精确偏差反馈
MODE_CIRCLE = 3     # 画圆中: 矩形✓, 激光✓  → 圆周轨迹跟踪

CMD_SET_MODE_AIM    = 0x01
CMD_SET_MODE_SCAN   = 0x02
CMD_SET_MODE_CIRCLE = 0x03
CMD_LASER_ON        = 0x10
CMD_LASER_OFF       = 0x11
CMD_SET_CIRCLE_PROG = 0x20


# ==================== 激光控制 ====================
def laser_on():
    if laser_pin is not None:
        laser_pin.value(1)

def laser_off():
    if laser_pin is not None:
        laser_pin.value(0)


# ==================== 阈值持久化 ====================
def save_thresholds():
    try:
        f = open(THRESHOLD_FILE, "w")
        f.write("black=%d,%d\n" % black)
        f.write("spot_gray=%d\n" % SPOT_GRAY_MIN)
        f.write("use_otsu=%d\n" % (1 if USE_OTSU else 0))
        f.close()
    except Exception as e:
        print("阈值保存失败:", e)

def load_thresholds():
    global black, SPOT_GRAY_MIN, USE_OTSU
    try:
        f = open(THRESHOLD_FILE, "r")
        for line in f:
            line = line.strip()
            if line.startswith("black="):
                parts = line[6:].split(",")
                black = (int(parts[0]), int(parts[1]))
            elif line.startswith("spot_gray="):
                SPOT_GRAY_MIN = int(line[10:])
            elif line.startswith("use_otsu="):
                USE_OTSU = (int(line[9:]) == 1)
        f.close()
        return True
    except:
        return False


# ==================== 工具函数 ====================
def get_otsu_threshold(img_gray):
    try:
        hist = img_gray.get_histogram()
        t_val = hist.get_threshold().value()
        return (0, t_val) if t_val > 0 else black
    except:
        return black


def get_line_intersection(line1, line2):
    (x1, y1), (x2, y2) = line1
    (x3, y3), (x4, y4) = line2
    A1, B1 = y2 - y1, x1 - x2
    A2, B2 = y4 - y3, x3 - x4
    det = A1 * B2 - A2 * B1
    if abs(det) < 1e-9:
        return ((x1 + x3) / 2, (y1 + y3) / 2)
    C1 = A1 * x1 + B1 * y1
    C2 = A2 * x3 + B2 * y3
    return ((B2 * C1 - B1 * C2) / det, (A1 * C2 - A2 * C1) / det)


# ==================== cv_lite 矩形检测 ====================
def detect_rects_cvlite(img_gray):
    img_shape = [DETECT_H, DETECT_W]
    img_np = img_gray.to_numpy_ref()

    rects_data = cv_lite.grayscale_find_rectangles(
        img_shape, img_np,
        CVL_CANNY_LO, CVL_CANNY_HI,
        CVL_APPROX_EPSILON,
        CVL_AREA_MIN_RATIO,
        CVL_MAX_ANGLE_COS,
        CVL_GAUSSIAN_BLUR
    )

    if not rects_data or len(rects_data) < 4:
        return None

    results = []
    for i in range(0, len(rects_data), 4):
        if i + 3 >= len(rects_data):
            break
        x, y, w, h = rects_data[i], rects_data[i+1], rects_data[i+2], rects_data[i+3]
        area = w * h
        if area < MIN_AREA or area > MAX_AREA:
            continue
        if h > 0:
            aspect = w / h
            if aspect < 0.5 or aspect > 2.0:
                continue
        cx = x + w // 2
        cy = y + h // 2
        corners = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
        results.append((cx, cy, w, h, corners, area))

    if not results:
        return None
    best = max(results, key=lambda r: r[5])
    return best


# ==================== find_rects 降级检测 ====================
def detect_rects_fallback(img_binary):
    rects = img_binary.find_rects(threshold=FIND_RECTS_THRESHOLD)
    if rects is None:
        return None

    best_area = 0
    best_rect = None

    for rect in rects:
        corners = rect.corners()
        if len(corners) != 4:
            continue
        area = rect.w() * rect.h()
        if area < MIN_AREA or area > MAX_AREA:
            continue
        w, h = rect.w(), rect.h()
        if w > 0 and h > 0:
            aspect = w / h if w > h else h / w
            if aspect > 2.5:
                continue
        if area > best_area:
            best_area = area
            center = get_line_intersection(
                [corners[0], corners[2]], [corners[1], corners[3]])
            best_rect = (int(center[0]), int(center[1]),
                         rect.w(), rect.h(), corners, area)

    return best_rect


# ==================== 光斑检测 ====================
def detect_laser_spot(img, rect_info=None):
    if rect_info is not None:
        rx, ry, rw, rh = rect_info
        if rw > 6 and rh > 6:
            gray = img.to_grayscale(copy=False)
            gray = gray.copy(roi=(max(0,rx), max(0,ry),
                                  min(rw, img.width()-rx),
                                  min(rh, img.height()-ry)))
        else:
            gray = img.to_grayscale(copy=False)
    else:
        gray = img.to_grayscale(copy=False)

    blobs = gray.find_blobs([(SPOT_GRAY_MIN, SPOT_GRAY_MAX)],
                            pixels_threshold=SPOT_MIN_PIXELS,
                            area_threshold=SPOT_AREA_THRESHOLD,
                            x_stride=1, y_stride=1,
                            margin=False)
    if not blobs:
        return None, 0, 0

    best = max(blobs, key=lambda b: b.pixels())
    if best.pixels() > SPOT_MAX_PIXELS:
        return None, 0, 0

    bx, by = best.x(), best.y()
    if rect_info is not None:
        bx += rect_info[0]
        by += rect_info[1]

    cx = int(bx + best.w() / 2)
    cy = int(by + best.h() / 2)
    return best, cx, cy


# ==================== 圆形计算 ====================
def calc_circle_radius_px(rect_w_px, rect_h_px):
    if rect_w_px < 6 or rect_h_px < 6:
        return 30
    scale_x = rect_w_px / RECT_WIDTH_CM
    scale_y = rect_h_px / RECT_HEIGHT_CM
    return int(CIRCLE_RADIUS_CM * (scale_x + scale_y) / 2.0)


def calc_circle_point(cx, cy, radius_px, progress_pct):
    angle_rad = math.radians((progress_pct / 100.0) * 360.0 + CIRCLE_PHASE_OFFSET_DEG)
    return (int(cx + radius_px * math.cos(angle_rad)),
            int(cy + radius_px * math.sin(angle_rad)))


# ==================== UART ====================
def pack_frame(data0, data1, data2, data3, error, flags):
    data = bytearray(14)
    data[0] = 0xAA; data[1] = 0x55
    data[2] = flags & 0x07
    data[3] = (data0 >> 8) & 0xFF; data[4] = data0 & 0xFF
    data[5] = (data1 >> 8) & 0xFF; data[6] = data1 & 0xFF
    data[7] = (data2 >> 8) & 0xFF; data[8] = data2 & 0xFF
    data[9] = (data3 >> 8) & 0xFF; data[10] = data3 & 0xFF
    data[11] = (error >> 8) & 0xFF; data[12] = error & 0xFF
    checksum = 0
    for i in range(2, 13):
        checksum ^= data[i]
    data[13] = checksum & 0xFF
    return bytes(data)


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
                rx_buf[0] = 0xBB; rx_buf[1] = 0x66
                rx_idx = 2; rx_state = RX_WAIT_DATA
            else:
                rx_state = RX_WAIT_H1
        elif rx_state == RX_WAIT_DATA:
            rx_buf[rx_idx] = b; rx_idx += 1
            if rx_idx >= 5:
                rx_state = RX_WAIT_H1
                if (rx_buf[2] ^ rx_buf[3]) == rx_buf[4]:
                    print("[UART] cmd=0x{:02X} p={}".format(rx_buf[2], rx_buf[3]))
                    return rx_buf[2], rx_buf[3]
    return None, None


rx_state = RX_WAIT_H1
rx_buf = bytearray(5)
rx_idx = 0
rx_call_count = 0


# ==================== 阈值UI ====================
def show_ui_to_screen(canvas):
    canvas.compress_for_ide()
    Display.show_image(canvas, x=(SCREEN_W - canvas.width()) // 2,
                       y=(SCREEN_H - canvas.height()) // 2)


def run_threshold_ui(tp):
    global black, SPOT_GRAY_MIN, USE_OTSU
    prev_laser = laser_pin.value()
    laser_on()

    mode = 0
    vals = list(black) if mode != 1 else [SPOT_GRAY_MIN]
    btn_color = (150, 150, 150)
    txt_color = (0, 0, 0)
    black_labels = ["Min", "Max"]
    spot_labels  = ["GrayMin"]
    row_ys = [60, 115, 170, 225, 280, 335]
    BTN_L_X, BTN_W, BTN_H = 5, 110, 50
    BTN_R_X = 525
    ui = image.Image(SCREEN_W, SCREEN_H, image.RGB565)
    titles = ["黑色门限", "光斑灰度", "OTSU自适应"]

    def draw_static():
        ui.draw_rectangle(0, 0, SCREEN_W, SCREEN_H, color=(255,255,255), fill=True)
        ui.draw_string_advanced(220, 8, 30, titles[mode], color=(0,0,0))
        ui.draw_rectangle(80, 425, 160, 45, color=btn_color, fill=True)
        ui.draw_string_advanced(120, 432, 30, "切换", color=txt_color)
        ui.draw_rectangle(400, 425, 160, 45, color=btn_color, fill=True)
        ui.draw_string_advanced(450, 432, 30, "返回", color=txt_color)

    draw_static()
    show_ui_to_screen(ui)

    while True:
        os.exitpoint()
        img_ = sensor.snapshot(chn=CAM_CHN_ID_0)
        img_ = img_.copy(roi=CUT_ROI)
        if mode == 0:
            img_ = img_.to_grayscale(); img_ = img_.binary([(vals[0], vals[1])]); img_ = img_.to_rgb565()
        elif mode == 1:
            img_ = img_.to_grayscale(); img_ = img_.binary([(vals[0], 255)]); img_ = img_.to_rgb565()
        else:
            img_ = img_.to_grayscale()
            otsu = get_otsu_threshold(img_); img_ = img_.binary([otsu]); img_ = img_.to_rgb565()

        ui.draw_image(img_, (SCREEN_W - img_.width()) // 2, (SCREEN_H - img_.height()) // 2)

        if mode == 2:
            ui.draw_rectangle(160, 160, 320, 80, color=btn_color, fill=True)
            ui.draw_string_advanced(200, 175, 28,
                "OTSU: %s" % ("ON" if USE_OTSU else "OFF"), color=txt_color)
        else:
            labels = black_labels if mode == 0 else spot_labels
            for i in range(len(labels)):
                y = row_ys[i]
                ui.draw_rectangle(BTN_L_X, y, BTN_W, BTN_H, color=btn_color, fill=True)
                ui.draw_string_advanced(BTN_L_X+5, y+2, 18, labels[i], color=txt_color)
                ui.draw_string_advanced(BTN_L_X+5, y+24, 18, "{} -".format(vals[i]), color=txt_color)
                ui.draw_rectangle(BTN_R_X, y, BTN_W, BTN_H, color=btn_color, fill=True)
                ui.draw_string_advanced(BTN_R_X+5, y+2, 18, labels[i], color=txt_color)
                ui.draw_string_advanced(BTN_R_X+5, y+24, 18, "{} +".format(vals[i]), color=txt_color)

        show_ui_to_screen(ui)

        points = tp.read()
        if len(points) > 0:
            tx, ty = points[0].x, points[0].y
            handled = False
            if mode == 2:
                if 160 <= tx <= 480 and 160 <= ty <= 240:
                    USE_OTSU = not USE_OTSU
                    handled = True; time.sleep_ms(200)
            else:
                for i in range(len(labels)):
                    y = row_ys[i]
                    if BTN_L_X <= tx <= BTN_L_X+BTN_W and y <= ty <= y+BTN_H:
                        vals[i] = max(0, vals[i]-1); handled = True; time.sleep_ms(120); break
                    if BTN_R_X <= tx <= BTN_R_X+BTN_W and y <= ty <= y+BTN_H:
                        vals[i] = min(255, vals[i]+1); handled = True; time.sleep_ms(120); break
            if not handled:
                if 80 <= tx <= 240 and 425 <= ty <= 470:
                    if mode == 0: black = tuple(vals)
                    elif mode == 1: SPOT_GRAY_MIN = vals[0]
                    mode = (mode + 1) % 3
                    vals = list(black) if mode == 0 else ([SPOT_GRAY_MIN] if mode == 1 else [])
                    draw_static(); time.sleep_ms(300)
                elif 400 <= tx <= 560 and 425 <= ty <= 470:
                    if mode == 0: black = tuple(vals)
                    elif mode == 1: SPOT_GRAY_MIN = vals[0]
                    save_thresholds(); break
        gc.collect()

    laser_pin.value(prev_laser)


# ==================== 主程序 ====================
try:
    print("2025E 阶段6 四态状态机(SCAN→ALIGN→AIM) 启动")
    print("cv_lite: %s  检测:%dx%d  显示:%dx%d" %
          ("可用" if HAS_CV_LITE else "不可用", DETECT_W, DETECT_H, SCREEN_W, SCREEN_H))

    # GPIO激光
    fpioa = FPIOA()
    fpioa.set_function(42, FPIOA.GPIO42)
    laser_pin = Pin(42, Pin.OUT)
    laser_pin.value(0)

    # 摄像头 320×240
    sensor = Sensor(width=DETECT_W, height=DETECT_H)
    sensor.reset()
    sensor.set_framesize(width=DETECT_W, height=DETECT_H)
    sensor.set_pixformat(Sensor.RGB565)
    Display.init(Display.ST7701, to_ide=True)
    MediaManager.init()
    sensor.run()

    uart = YbUart(baudrate=UART_BAUDRATE)
    load_thresholds()
    tp = TOUCH(0)
    clock = time.clock()

    # ===== 状态变量 =====
    flag_detected = False         # 矩形是否锁定
    detect_counter = 0
    lost_counter = 0
    prev_rect_info = None
    touch_counter = 0
    last_send_time = time.ticks_ms()

    # ===== 四态状态机 =====
    work_mode = MODE_SCAN          # 初始：扫描模式
    laser_force = False
    circle_progress = 0

    # 各模式帧计数器
    scan_rect_found_frames = 0     # SCAN中连续发现矩形帧数
    align_laser_found_frames = 0   # ALIGN中连续发现激光帧数
    aim_laser_lost_frames = 0      # AIM中连续丢失激光帧数
    lost_rect_frames = 0           # 连续丢失矩形帧数（任意模式）

    circle_radius_px = 30
    circle_target_x = 0
    circle_target_y = 0

    rx_state = RX_WAIT_H1
    rx_buf = bytearray(5)
    rx_idx = 0
    rx_call_count = 0

    mode_names = {MODE_SCAN: "SCAN", MODE_ALIGN: "ALIGN",
                  MODE_AIM: "AIM", MODE_CIRCLE: "CIRCLE"}

    # 颜色常量
    C_GREEN  = (0, 255, 0)
    C_YELLOW = (255, 255, 0)
    C_RED    = (255, 0, 0)
    C_BLUE   = (0, 160, 255)
    C_ORANGE = (255, 128, 0)
    C_PURPLE = (128, 0, 255)
    C_CYAN   = (0, 255, 255)
    C_WHITE  = (255, 255, 255)

    SCALE = 2

    while True:
        clock.tick()
        os.exitpoint()

        # ===== UART接收 =====
        cmd, param = parse_cmd_frame()
        if cmd is not None:
            if cmd == CMD_SET_MODE_AIM:
                work_mode = MODE_AIM
                scan_rect_found_frames = 0; align_laser_found_frames = 0
                aim_laser_lost_frames = 0; lost_rect_frames = 0
                print("[RX] → AIM")
            elif cmd == CMD_SET_MODE_SCAN:
                work_mode = MODE_SCAN
                scan_rect_found_frames = 0; align_laser_found_frames = 0
                aim_laser_lost_frames = 0; lost_rect_frames = 0
                print("[RX] → SCAN")
            elif cmd == CMD_SET_MODE_CIRCLE:
                work_mode = MODE_CIRCLE; circle_progress = 0
                scan_rect_found_frames = 0; align_laser_found_frames = 0
                aim_laser_lost_frames = 0; lost_rect_frames = 0
                print("[RX] → CIRCLE")
            elif cmd == CMD_LASER_ON:
                laser_force = True; laser_on()
                print("[RX] LASER_ON")
            elif cmd == CMD_LASER_OFF:
                laser_force = False; laser_off()
                print("[RX] LASER_OFF")
            elif cmd == CMD_SET_CIRCLE_PROG:
                circle_progress = param

        # ===== 长按进阈值 =====
        points = tp.read()
        if len(points) > 0:
            touch_counter += 1
            if touch_counter > LONG_PRESS_THRESHOLD:
                run_threshold_ui(tp); touch_counter = 0
        else:
            touch_counter = max(0, touch_counter - 1)

        # ===== 拍照 =====
        img = sensor.snapshot(chn=CAM_CHN_ID_0)

        # ===== 矩形检测 =====
        img_gray = img.to_grayscale(copy=False)

        if HAS_CV_LITE:
            rect_result = detect_rects_cvlite(img_gray)
        else:
            if USE_OTSU:
                thresh = get_otsu_threshold(img_gray)
            else:
                thresh = black
            img_binary = img_gray.binary([thresh])
            if DILATE_TIMES > 0:
                img_binary.dilate(DILATE_TIMES)
            rect_result = detect_rects_fallback(img_binary)

        # ===== 矩形状态防抖 =====
        current_has_rect = rect_result is not None
        if current_has_rect:
            prev_rect_info = rect_result
            lost_rect_frames = 0
            if not flag_detected:
                detect_counter += 1
                if detect_counter >= MIN_DETECT_FRAMES:
                    flag_detected = True; detect_counter = 0
            else:
                lost_counter = 0
        else:
            detect_counter = 0
            lost_rect_frames += 1
            if flag_detected:
                lost_counter += 1
                if lost_counter >= MIN_LOST_FRAMES:
                    flag_detected = False; lost_counter = 0

        # 丢帧保持
        if not current_has_rect and flag_detected:
            rect_result = prev_rect_info

        # ===== 提取矩形信息 =====
        rect_cx, rect_cy = 0, 0
        rect_w, rect_h = 0, 0
        corners = None

        if rect_result is not None and flag_detected:
            rect_cx, rect_cy, rect_w, rect_h, corners, _ = rect_result

        # ===== 光斑检测（框内ROI搜索） =====
        roi_info = None
        if corners is not None and len(corners) == 4:
            xs = [p[0] for p in corners]; ys = [p[1] for p in corners]
            roi_info = (min(xs), min(ys), max(xs)-min(xs), max(ys)-min(ys))

        spot_blob, dot_x, dot_y = detect_laser_spot(img, roi_info)
        has_dot = spot_blob is not None

        # ===== 偏差计算 =====
        error = 0
        if has_dot and rect_cx > 0:
            error = int(math.sqrt((dot_x - rect_cx) ** 2 + (dot_y - rect_cy) ** 2))

        # ===== 圆参数 =====
        if flag_detected and rect_cx > 0 and rect_w > 0:
            circle_radius_px = calc_circle_radius_px(rect_w, rect_h)

        # ============================================================
        #  四态状态机：SCAN ←→ ALIGN ←→ AIM  (CIRCLE独立)
        # ============================================================

        if work_mode == MODE_SCAN:
            # SCAN → ALIGN: 连续发现矩形N帧
            if flag_detected and current_has_rect:
                scan_rect_found_frames += 1
                if scan_rect_found_frames >= AUTO_SCAN_FOUND_FRAMES:
                    work_mode = MODE_ALIGN
                    scan_rect_found_frames = 0
                    align_laser_found_frames = 0
                    aim_laser_lost_frames = 0
                    print("[AUTO] SCAN→ALIGN (矩形锁定,等待激光)")
            else:
                scan_rect_found_frames = 0

        elif work_mode == MODE_ALIGN:
            # ALIGN → SCAN: 矩形丢失太久
            if lost_rect_frames >= AUTO_SCAN_LOST_FRAMES:
                work_mode = MODE_SCAN
                scan_rect_found_frames = 0
                align_laser_found_frames = 0
                aim_laser_lost_frames = 0
                lost_rect_frames = 0
                print("[AUTO] ALIGN→SCAN (矩形丢失)")

            # ALIGN → AIM: 连续发现激光N帧
            elif has_dot:
                align_laser_found_frames += 1
                if align_laser_found_frames >= AUTO_ALIGN_FOUND_FRAMES:
                    work_mode = MODE_AIM
                    align_laser_found_frames = 0
                    aim_laser_lost_frames = 0
                    scan_rect_found_frames = 0
                    print("[AUTO] ALIGN→AIM (激光已对准!)")
            else:
                align_laser_found_frames = 0

        elif work_mode == MODE_AIM:
            # AIM → SCAN: 矩形丢失太久
            if lost_rect_frames >= AUTO_SCAN_LOST_FRAMES:
                work_mode = MODE_SCAN
                scan_rect_found_frames = 0
                align_laser_found_frames = 0
                aim_laser_lost_frames = 0
                lost_rect_frames = 0
                print("[AUTO] AIM→SCAN (矩形丢失)")

            # AIM → ALIGN: 激光丢失但矩形还在
            elif not has_dot:
                aim_laser_lost_frames += 1
                if aim_laser_lost_frames >= AUTO_AIM_LASER_LOST_FRAMES:
                    work_mode = MODE_ALIGN
                    aim_laser_lost_frames = 0
                    align_laser_found_frames = 0
                    scan_rect_found_frames = 0
                    print("[AUTO] AIM→ALIGN (激光丢失,重新引导)")
            else:
                aim_laser_lost_frames = 0

        elif work_mode == MODE_CIRCLE:
            # CIRCLE → SCAN: 矩形丢失太久
            if lost_rect_frames >= AUTO_SCAN_LOST_FRAMES:
                work_mode = MODE_SCAN
                scan_rect_found_frames = 0
                align_laser_found_frames = 0
                aim_laser_lost_frames = 0
                lost_rect_frames = 0
                print("[AUTO] CIRCLE→SCAN (矩形丢失)")

            # 计算圆周目标点
            if rect_cx > 0:
                circle_target_x, circle_target_y = calc_circle_point(
                    rect_cx, rect_cy, circle_radius_px, circle_progress)
            else:
                circle_target_x, circle_target_y = 0, 0

        # ===== 激光控制（按模式自动） =====
        if not laser_force:
            if work_mode in (MODE_ALIGN, MODE_AIM, MODE_CIRCLE) and flag_detected:
                laser_on()     # ALIGN/AIM/CIRCLE 都需要激光可见
            else:
                laser_off()    # SCAN 时关闭
            # 注: ALIGN模式开激光，让摄像头能看到并引导到靶上

        # ============================================================
        #  UART 发送（每30ms）
        # ============================================================
        now = time.ticks_ms()
        if time.ticks_diff(now, last_send_time) >= UART_SEND_INTERVAL_MS:
            last_send_time = now

            flags = 0
            if flag_detected and rect_cx > 0:
                flags |= 0x01     # bit0: 矩形锁定
            if has_dot:
                flags |= 0x02     # bit1: 激光可见

            # MSPM0/STM32通过flags判断当前状态:
            #   flags=0x00 → SCAN (无靶)
            #   flags=0x01 → ALIGN (有靶无激光) ← 队友需控制云台!
            #   flags=0x03 → AIM   (有靶有激光) ← 精确偏差反馈

            if work_mode == MODE_SCAN:
                tx_d0 = tx_d1 = tx_d2 = tx_d3 = tx_err = 0

            elif work_mode == MODE_ALIGN:
                # 发送靶心坐标，队友用来引导云台
                tx_d0 = rect_cx * SCALE if (flags & 0x01) else 0
                tx_d1 = rect_cy * SCALE if (flags & 0x01) else 0
                tx_d2 = 0            # 无激光
                tx_d3 = 0
                tx_err = 0

            elif work_mode == MODE_AIM:
                # 发送靶心+光斑+偏差，精确反馈
                tx_d0 = rect_cx * SCALE if (flags & 0x01) else 0
                tx_d1 = rect_cy * SCALE if (flags & 0x01) else 0
                tx_d2 = dot_x * SCALE  if (flags & 0x02) else 0
                tx_d3 = dot_y * SCALE  if (flags & 0x02) else 0
                tx_err = error * SCALE if (flags & 0x03) == 0x03 else 0

            elif work_mode == MODE_CIRCLE:
                tx_d0 = circle_target_x * SCALE if rect_cx > 0 else 0
                tx_d1 = circle_target_y * SCALE if rect_cx > 0 else 0
                tx_d2 = circle_progress
                tx_d3 = circle_radius_px * SCALE
                tx_err = 0

            else:
                tx_d0 = tx_d1 = tx_d2 = tx_d3 = tx_err = 0

            uart.send(pack_frame(tx_d0, tx_d1, tx_d2, tx_d3, tx_err, flags))

        # ============================================================
        #  绘制
        # ============================================================

        # 矩形框 + 靶心
        if corners is not None and flag_detected:
            for i in range(4):
                x1, y1 = corners[i]
                x2, y2 = corners[(i + 1) % 4]
                # ALIGN模式用青色框，AIM用绿色框
                c = C_CYAN if work_mode == MODE_ALIGN else C_GREEN
                img.draw_line(x1, y1, x2, y2, color=c, thickness=1)
            img.draw_circle(rect_cx, rect_cy, 3, color=C_YELLOW, thickness=1)
            if rect_cx > 0:
                img.draw_circle(rect_cx, rect_cy, circle_radius_px, color=C_BLUE, thickness=1)

        # 光斑标记
        if has_dot:
            img.draw_cross(dot_x, dot_y, color=C_PURPLE, size=5)
            # AIM模式下画偏差连线
            if work_mode == MODE_AIM and rect_cx > 0:
                img.draw_line(dot_x, dot_y, rect_cx, rect_cy, color=C_YELLOW, thickness=1)

        # 画圆目标点
        if work_mode == MODE_CIRCLE and circle_target_x > 0:
            img.draw_cross(circle_target_x, circle_target_y, color=C_ORANGE, size=5)

        # ALIGN模式文字提示
        if work_mode == MODE_ALIGN:
            img.draw_string_advanced(80, 100, 22, "ALIGNING...", color=C_CYAN)
            img.draw_string_advanced(60, 130, 16,
                "Guide laser to target", color=C_CYAN)

        # SCAN模式文字
        if work_mode == MODE_SCAN:
            img.draw_string_advanced(100, 110, 20, "Searching...", color=C_YELLOW)

        # 状态栏
        fps_str = "fps:{:.1f} m:{} l:{}".format(
            clock.fps(), mode_names[work_mode],
            "ON" if laser_pin.value() else "OFF")
        img.draw_string_advanced(2, 2, 12, fps_str, color=C_RED)

        # 第二行：详细状态
        if work_mode == MODE_ALIGN:
            detail = "laser wait:{}/{}".format(align_laser_found_frames, AUTO_ALIGN_FOUND_FRAMES)
            img.draw_string_advanced(2, 16, 11, detail, color=C_CYAN)
        elif work_mode == MODE_AIM and aim_laser_lost_frames > 0:
            detail = "laser lost:{}/{}".format(aim_laser_lost_frames, AUTO_AIM_LASER_LOST_FRAMES)
            img.draw_string_advanced(2, 16, 11, detail, color=C_YELLOW)
        elif work_mode == MODE_SCAN and scan_rect_found_frames > 0:
            detail = "rect found:{}/{}".format(scan_rect_found_frames, AUTO_SCAN_FOUND_FRAMES)
            img.draw_string_advanced(2, 16, 11, detail, color=C_GREEN)

        # 显示
        if not SKIP_COMPRESS_FOR_IDE:
            img.compress_for_ide()
        Display.show_image(img)
        gc.collect()

except KeyboardInterrupt as e:
    print("用户停止:", e)
except BaseException as e:
    print("异常: ", e)
finally:
    try:
        laser_off()
    except:
        pass
    if isinstance(sensor, Sensor):
        sensor.stop()
    try:
        if uart is not None:
            uart.deinit()
    except:
        pass
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()


'''
==================== 阶段6 vs 阶段5.5 核心变化 ====================

  状态机:  三态(SCAN/AIM/CIRCLE) → 四态(SCAN/ALIGN/AIM/CIRCLE)

  新增 MODE_ALIGN (模式码=1):
    进入条件: SCAN中连续发现矩形3帧
    退出条件: 激光连续发现3帧 → AIM
             矩形丢失15帧    → SCAN

  AIM新增回退:
    激光丢失10帧(rect还在) → 回退到ALIGN → 队友重新引导

==================== 队友(MSPM0/STM32)对接说明 ====================

  队友通过解析14字节帧的 flags 字段判断当前状态:

  ┌────────┬─────────────────────────────────────┐
  │ flags  │             含义与动作               │
  ├────────┼─────────────────────────────────────┤
  │  0x00  │ SCAN: 无靶, 自主搜索                 │
  │  0x01  │ ALIGN: 靶心已锁定(D0,D1),             │
  │        │       激光未识别, 用(D0,D1)控制云台   │
  │        │       把激光引导到靶上                │
  │  0x03  │ AIM:  靶+激光都有, 用(Error)做精确PID │
  │  0x07  │ CIRCLE相关(见CIRCLE模式数据)          │
  └────────┴─────────────────────────────────────┘

  队友伪代码:
    if (flags & 0x01) == 0:
        // SCAN: 无靶, 自主旋转搜索
        scan_rotate()
    elif (flags & 0x02) == 0:
        // ALIGN: 有靶无激光, 用D0/D1做云台引导
        gimbal_goto(D0, D1)
    else:
        // AIM: 有靶有激光, 用Error做精细PID
        pid_correct(Error)

==================== 版本演进 ====================

  阶段5.3: 640×480, find_rects, 固定阈值, ~10fps
  阶段5.4: 320×240, find_rects, OTSU,      ~15fps
  阶段5.5: 320×240, cv_lite,   OTSU,       ~35fps
  阶段6:   320×240, cv_lite,   OTSU, 四态  ~35fps (逻辑增强,性能持平)
'''
