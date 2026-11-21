"""
标定_串口.py
模型：yolov5nu.pt → best.kmodel (224×224)
功能：钢球检测 + 两点标定 + 位置显示 + 串口发送（含速度保护）
"""
from libs.PipeLine import PipeLine
from libs.YOLO import YOLO11
import gc, time, os, ujson
import ybUtils.YbKey as YbKey
from ybUtils.YbUart import YbUart

# ==================== 配置区 ====================
KMODEL_PATH = "/data/best_v5nu/best_5_2.kmodel"
LABELS = ["ball"]
MODEL_INPUT_SIZE = [224, 224]
RGB888P_SIZE = [640, 480]
DISPLAY_MODE = "lcd"
CONF_THRESH = 0.5
NMS_THRESH  = 0.45
MAX_BOXES_NUM = 3
DEBUG_MODE = 0

CALIB_FILE = "/data/calib.json"
UART_BAUD = 115200
SPEED_WINDOW = 5
# =================================================

class SpeedCalculator:
    def __init__(self, window=5, max_speed=50.0):
        self.window = window
        self.positions = []
        self.max_speed = max_speed          # 速度上限 cm/s

    def add(self, pos_cm):
        now = time.ticks_ms()
        self.positions.append((now, pos_cm))
        if len(self.positions) > self.window + 1:
            self.positions.pop(0)

        if len(self.positions) >= self.window + 1:
            t0, p0 = self.positions[0]
            t1, p1 = self.positions[-1]
            dt = time.ticks_diff(t1, t0) / 1000.0
            if dt > 0.001:
                speed = (p1 - p0) / dt
                # 限幅，防止视野丢失导致的速度跳变
                if speed >  self.max_speed: speed =  self.max_speed
                if speed < -self.max_speed: speed = -self.max_speed
                return speed
        return 0.0

def load_calib():
    try:
        with open(CALIB_FILE, "r") as f:
            data = ujson.load(f)
            scale = data.get("scale", 1.0)
            offset = data.get("offset", 0)
            return scale, offset
    except:
        return 1.0, 0

def save_calib(scale, offset):
    data = {"scale": scale, "offset": offset}
    with open(CALIB_FILE, "w") as f:
        ujson.dump(data, f)

def send_uart_packet(uart, deviation_cm_x100, speed_cms):
    dev = int(deviation_cm_x100)
    if dev > 32767: dev = 32767
    elif dev < -32768: dev = -32768
    spd = int(speed_cms)
    if spd > 127: spd = 127
    elif spd < -128: spd = -128
    spd_byte = spd & 0xFF
    high = (dev >> 8) & 0xFF
    low = dev & 0xFF
    packet = bytes([0xAA, high, low, spd_byte])
    checksum = 0xAA ^ high ^ low ^ spd_byte
    packet += bytes([checksum])
    uart.write(packet)

def main():
    pl = PipeLine(rgb888p_size=RGB888P_SIZE, display_mode=DISPLAY_MODE)
    pl.create(fps=60)
    display_size = pl.get_display_size()

    yolo = YOLO11(
        task_type="detect", mode="video",
        kmodel_path=KMODEL_PATH, labels=LABELS,
        rgb888p_size=RGB888P_SIZE, model_input_size=MODEL_INPUT_SIZE,
        display_size=display_size, conf_thresh=CONF_THRESH,
        nms_thresh=NMS_THRESH, max_boxes_num=MAX_BOXES_NUM,
        debug_mode=DEBUG_MODE
    )
    yolo.config_preprocess()

    key = YbKey.YbKey()
    uart = YbUart(baudrate=UART_BAUD)
    scale, offset = load_calib()
    spd_calc = SpeedCalculator(window=SPEED_WINDOW)

    calib_mode = False
    calib_step = 0
    calib_o_x = 0
    calib_5_x = 0
    key_last = False
    key_press_time = 0
    long_handled = False
    fps = 0.0; smooth_fps = 0.0; fps_cnt = 0; fps_start = time.ticks_ms()

    try:
        while True:
            img = pl.get_frame()
            res = yolo.run(img)

            best_box = None; best_score = 0.0
            if res and len(res[0]) > 0:
                for i, s in enumerate(res[2]):
                    if s > best_score:
                        best_score = s
                        best_box = res[0][i]

            yolo.draw_result(res, pl.osd_img)

            # 按键处理（标定逻辑不变）
            now = time.ticks_ms()
            cur = key.is_pressed()
            if cur and not key_last:
                key_press_time = now
                long_handled = False
            elif not cur and key_last:
                if not long_handled:
                    dur = time.ticks_diff(now, key_press_time)
                    if dur >= 1000:
                        calib_mode = not calib_mode
                        if calib_mode: calib_step = 0
                        long_handled = True
                    else:
                        if calib_mode and best_box:
                            x, y, w, h = best_box
                            cx = x + w // 2
                            if calib_step == 0:
                                calib_o_x = cx; calib_step = 1
                            elif calib_step == 1:
                                calib_5_x = cx
                                if abs(calib_5_x - calib_o_x) < 1:
                                    calib_step = 0
                                else:
                                    scale = 5.0 / (calib_5_x - calib_o_x)
                                    offset = calib_o_x
                                    save_calib(scale, offset)
                                    calib_mode = False; calib_step = 0
            key_last = cur

            # ---------- 新：位置与速度计算（带丢失保护）----------
            pos_cm = 0.0
            spd_cm = 0.0   # 统一变量名

            if best_box is not None:
                x, y, w, h = best_box
                cx = x + w // 2
                pos_cm = (cx - offset) * scale
                spd_cm = spd_calc.add(pos_cm)
            else:
                # 球丢失：重置速度计算器，强制速度为0
                spd_calc = SpeedCalculator(window=SPEED_WINDOW)
                spd_cm = 0.0

            if not calib_mode:
                dev_int = int(pos_cm * 100) if best_box else 0
                send_uart_packet(uart, dev_int, spd_cm)
            # ------------------------------------------------

            # 显示
            fps_cnt += 1
            el = time.ticks_diff(time.ticks_ms(), fps_start)
            if el >= 1000:
                fps = fps_cnt * 1000.0 / el
                smooth_fps = smooth_fps * 0.7 + fps * 0.3
                fps_cnt = 0; fps_start = time.ticks_ms()

            pl.osd_img.draw_string_advanced(10, 10, 24, "FPS: %.1f" % smooth_fps, color=(255,255,255))
            if calib_mode:
                hint = "O" if calib_step==0 else "+5cm"
                pl.osd_img.draw_string_advanced(10, 40, 24, "Calib: " + hint, color=(0,255,255))
            else:
                if best_box:
                    pl.osd_img.draw_string_advanced(10, 40, 24, "Pos: %.2f cm" % pos_cm, color=(0,255,0))
                    pl.osd_img.draw_string_advanced(10, 65, 20, "Speed: %.1f cm/s" % spd_cm, color=(255,255,0))
                    pl.osd_img.draw_string_advanced(10, 85, 20, "CX: %d" % cx, color=(200,200,200))
                else:
                    pl.osd_img.draw_string_advanced(10, 40, 24, "No ball", color=(255,0,0))

            pl.show_image()
            gc.collect()

    except KeyboardInterrupt:
        print("User stop")
    except Exception as e:
        import sys; sys.print_exception(e)
    finally:
        yolo.deinit(); pl.destroy(); uart.deinit()

if __name__ == "__main__":
    main()
