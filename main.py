from machine import Pin, SPI, ADC, UART, I2C, PWM, Encoder
import time
from machine import Timer
import struct
from speed_PID import SPEED_PID

'''
硬件载体：ESP32-V1
固件：ESP32_GENERIC-20260406-v1.28.0.bin
'''

'''=================工具函数============================================================'''
def limit(val, low, high):
    if val < low:
        return low
    if val > high:
        return high
    return val


'''=================电机 PWM 配置============================================================'''
IN1_L = PWM(Pin(13), freq=20000, duty=0)
IN2_L = PWM(Pin(15), freq=20000, duty=0)

IN1_R = PWM(Pin(25), freq=20000, duty=0)
IN2_R = PWM(Pin(14), freq=20000, duty=0)


def motor(left_speed, right_speed):
    """控制两电机 speed: -1023~1023"""
    left_speed = int(limit(left_speed, -1023, 1023))
    right_speed = int(limit(right_speed, -1023, 1023))

    if left_speed >= 0:
        IN1_L.duty(left_speed)
        IN2_L.duty(0)
    else:
        IN1_L.duty(0)
        IN2_L.duty(-left_speed)

    if right_speed >= 0:
        IN1_R.duty(right_speed)
        IN2_R.duty(0)
    else:
        IN1_R.duty(0)
        IN2_R.duty(-right_speed)


'''=================五路循迹ADC配置========================================================='''
ADC_pins = [36, 33, 32, 35, 34]
My_ADC = [ADC(Pin(x)) for x in ADC_pins]
for i in range(5):
    My_ADC[i].atten(ADC.ATTN_11DB)
    My_ADC[i].width(ADC.WIDTH_12BIT)

adcdata = [0, 0, 0, 0, 0]
ADC_value = [0.0, 0.0, 0.0, 0.0, 0.0]
State = [0, 0, 0, 0, 0]

# 黑线ADC更大 -> True
# 黑线ADC更小 -> False
LINE_BLACK_HIGH = True

# 按实际传感器阈值调整
TRACK_THRESHOLD = [700, 700, 700, 700, 700]


def get_track_binary():
    binary = []
    for i in range(5):
        val = 0
        for _ in range(3):
            val += My_ADC[i].read()

        avg_val = val // 3
        adcdata[i] = avg_val
        ADC_value[i] = avg_val * 3.3 / 4095

        if LINE_BLACK_HIGH:
            State[i] = 1 if avg_val > TRACK_THRESHOLD[i] else 0
        else:
            State[i] = 1 if avg_val < TRACK_THRESHOLD[i] else 0

        binary.append(State[i])
    return binary


SENSOR_WEIGHT = [-2, -1, 0, 1, 2]


def calc_error(bin_state):
    s0, s1, s2, s3, s4 = bin_state
    sum_w = (
        s0 * SENSOR_WEIGHT[0] +
        s1 * SENSOR_WEIGHT[1] +
        s2 * SENSOR_WEIGHT[2] +
        s3 * SENSOR_WEIGHT[3] +
        s4 * SENSOR_WEIGHT[4]
    )
    sum_sig = s0 + s1 + s2 + s3 + s4
    if sum_sig == 0:
        return None
    return sum_w / sum_sig


'''=================十字路口短时记忆========================================================='''
last_s0_on_time = -1000
last_s4_on_time = -1000
CROSS_DETECT_WINDOW = 140   # ms，可调 100~180


def is_cross_priority(bin_state, now_ms):
    """
    十字优先判定：
    1. 当前 s0 和 s4 同时为 1
    2. 或者在短时间窗口内，s0 和 s4 先后都出现过 1
    """
    global last_s0_on_time, last_s4_on_time

    s0, s1, s2, s3, s4 = bin_state

    if s0 == 1:
        last_s0_on_time = now_ms
    if s4 == 1:
        last_s4_on_time = now_ms

    if s0 == 1 and s4 == 1:
        return True

    if time.ticks_diff(now_ms, last_s0_on_time) <= CROSS_DETECT_WINDOW and \
       time.ticks_diff(now_ms, last_s4_on_time) <= CROSS_DETECT_WINDOW:
        return True

    return False


def classify_track(bin_state, last_dir, cross_flag=False):
    """
    返回:
    mode, dir_mark
    """
    s0, s1, s2, s3, s4 = bin_state
    pat = "{}{}{}{}{}".format(s0, s1, s2, s3, s4)

    # 十字优先
    if cross_flag:
        return "CROSS", 0

    if pat == "00000":
        return "LOST", last_dir

    if pat == "11111":
        return "CROSS", 0

    if pat in ("00100", "01110"):
        return "STRAIGHT", 0

    # 直角
    if pat == "11100":
        return "RIGHT_ANGLE_LEFT", -1

    if pat == "00111":
        return "RIGHT_ANGLE_RIGHT", 1

    # 曲线
    if pat in ("01100", "01000", "11000", "11110", "10000"):
        return "CURVE_LEFT", -1

    if pat in ("00110", "00010", "00011", "01111", "00001"):
        return "CURVE_RIGHT", 1

    # 兜底
    err = calc_error(bin_state)
    if err is None:
        return "LOST", last_dir
    elif err < -0.6:
        return "CURVE_LEFT", -1
    elif err > 0.6:
        return "CURVE_RIGHT", 1
    else:
        return "STRAIGHT", 0


'''=================循迹PID============================================================'''
class LinePID:
    def __init__(self, kp, ki, kd, int_limit, steer_limit):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.int_limit = int_limit
        self.steer_limit = steer_limit

        self.last_err = 0.0
        self.last_filter_d = 0.0
        self.integral = 0.0
        self.filtered_err = 0.0
        self.last_filtered_err = 0.0

        self.filter_alpha_slow = 0.4
        self.filter_alpha_fast = 0.1

    def compute(self, err):
        abs_err = abs(err)

        if abs_err < 0.8:
            alpha = self.filter_alpha_slow
        else:
            alpha = self.filter_alpha_fast

        self.filtered_err = alpha * self.filtered_err + (1 - alpha) * err

        p = self.Kp * err

        if abs_err < 0.8:
            self.integral += err
        elif abs_err > 1.5:
            self.integral = 0

        self.integral = max(-self.int_limit, min(self.int_limit, self.integral))
        i = self.Ki * self.integral

        raw_d = self.filtered_err - self.last_filtered_err
        filter_d = 0.4 * raw_d + 0.6 * self.last_filter_d
        d = self.Kd * filter_d
        self.last_filter_d = filter_d

        self.last_err = err
        self.last_filtered_err = self.filtered_err

        steer = p + i + d
        return max(-self.steer_limit, min(self.steer_limit, steer))


track_pid = LinePID(kp=4.5, ki=0.006, kd=8.0, int_limit=8, steer_limit=70.0)


'''=================电机 编码器 配置============================================================'''
encoder1 = Encoder(0, Pin(17, Pin.IN, Pin.PULL_UP), Pin(16, Pin.IN, Pin.PULL_UP), filter_ns=20, phases=1)
encoder2 = Encoder(4, Pin(18, Pin.IN, Pin.PULL_UP), Pin(19, Pin.IN, Pin.PULL_UP), filter_ns=20, phases=1)


'''=================定时器 配置==============================================================='''
encoder1_counter = [0, 0]
encoder2_counter = [0, 0]

PULSES_PER_ROUND = 10000

L_DEADZONE = 560
R_DEADZONE = 580

L_PWM_pid = SPEED_PID(
    mode=0, Kp=1.0, Ki=0.348, Kd=0.00,
    max_integral=1020, max_output=1023, min_output=-1023,
    DeadZone=L_DEADZONE
)

R_PWM_pid = SPEED_PID(
    mode=0, Kp=1.0, Ki=0.348, Kd=0.00,
    max_integral=1020, max_output=1023, min_output=-1023,
    DeadZone=R_DEADZONE
)


def Get_encoder_rpm():
    encoder1_counter[0] = encoder1.value()
    encoder2_counter[0] = encoder2.value()

    encoder1.value(0)
    encoder2.value(0)

    Encode1_RPM = encoder1_counter[0] * 0.3
    Encode2_RPM = encoder2_counter[0] * 0.3

    encoder1_counter[0] = 0
    encoder2_counter[0] = 0

    return Encode1_RPM, Encode2_RPM


actual_speed = [0.0, 0.0]
New_actual_speed = [0.0, 0.0]
target_speed = [0.0, 0.0]
pwm_value = [0.0, 0.0]


def Timer0_Pro(t):
    New_actual_speed[0], New_actual_speed[1] = Get_encoder_rpm()

    K1 = 0.7
    actual_speed[0] = K1 * actual_speed[0] + (1 - K1) * New_actual_speed[0]

    K2 = 0.7
    actual_speed[1] = K2 * actual_speed[1] + (1 - K2) * New_actual_speed[1]

    pwm_value[0] = L_PWM_pid.compute(target_speed[0], actual_speed[0])
    pwm_value[1] = R_PWM_pid.compute(target_speed[1], actual_speed[1])

    motor(pwm_value[0], pwm_value[1])


MyTimA = Timer(0)
MyTimA.init(period=20, mode=Timer.PERIODIC, callback=Timer0_Pro)


'''=================循迹全局变量==============================================================='''
last_valid_err = 0.0
last_steer = 0.0
MAX_STEER_DELTA = 18.0

last_track_dir = 0
lost_count = 0

# ===== 直角模式变量 =====
right_angle_mode = 0      # 0=关闭, 1=左直角, 2=右直角
right_angle_phase = 0     # 0=无, 1=强制转向阶段, 2=找线退出阶段
right_angle_force_ticks = 0
RIGHT_ANGLE_FORCE_TICKS = 14
# ==========================

start_time = time.ticks_ms()
last_print_time = start_time
print_interval = 200


def is_center_found(bin_state):
    """
    判断是否重新找到主线中心
    """
    pat = "{}{}{}{}{}".format(bin_state[0], bin_state[1], bin_state[2], bin_state[3], bin_state[4])
    if pat in ("00100", "01110", "00110", "01100"):
        return True
    return False


print('进入主循环-十字优先，非十字立即直角版')

while True:
    time.sleep_ms(10)

    current_time = time.ticks_ms()
    delta_interval = time.ticks_diff(current_time, last_print_time)

    bin_state = get_track_binary()
    cross_flag = is_cross_priority(bin_state, current_time)
    mode, dir_mark = classify_track(bin_state, last_track_dir, cross_flag)

    if dir_mark != 0:
        last_track_dir = dir_mark

    if mode == "LOST":
        lost_count += 1
    else:
        lost_count = 0

    # =========================================================
    # 十字优先
    # 如果不是十字，再立刻根据直角模式进入拐弯
    # =========================================================
    if right_angle_mode == 0:
        if (not cross_flag) and mode == "RIGHT_ANGLE_LEFT":
            right_angle_mode = 1
            right_angle_phase = 1
            right_angle_force_ticks = RIGHT_ANGLE_FORCE_TICKS

        elif (not cross_flag) and mode == "RIGHT_ANGLE_RIGHT":
            right_angle_mode = 2
            right_angle_phase = 1
            right_angle_force_ticks = RIGHT_ANGLE_FORCE_TICKS

    # ==================== 直角执行模式 ====================
    if right_angle_mode != 0:
        # ---------- 左直角 ----------
        if right_angle_mode == 1:
            if right_angle_phase == 1:
                target_speed[0] = -75.0
                target_speed[1] = 135.0
                right_angle_force_ticks -= 1

                # 如果转弯过程中明确发现是十字，则取消转弯
                if cross_flag:
                    right_angle_mode = 0
                    right_angle_phase = 0
                    right_angle_force_ticks = 0
                    last_valid_err = 0.0
                    last_steer = 0.0

                elif right_angle_force_ticks <= 0:
                    right_angle_phase = 2

            elif right_angle_phase == 2:
                target_speed[0] = -35.0
                target_speed[1] = 105.0

                if cross_flag:
                    right_angle_mode = 0
                    right_angle_phase = 0
                    last_valid_err = 0.0
                    last_steer = 0.0

                elif is_center_found(bin_state):
                    right_angle_mode = 0
                    right_angle_phase = 0
                    last_valid_err = 0.0
                    last_steer = 0.0

        # ---------- 右直角 ----------
        elif right_angle_mode == 2:
            if right_angle_phase == 1:
                target_speed[0] = 135.0
                target_speed[1] = -75.0
                right_angle_force_ticks -= 1

                # 如果转弯过程中明确发现是十字，则取消转弯
                if cross_flag:
                    right_angle_mode = 0
                    right_angle_phase = 0
                    right_angle_force_ticks = 0
                    last_valid_err = 0.0
                    last_steer = 0.0

                elif right_angle_force_ticks <= 0:
                    right_angle_phase = 2

            elif right_angle_phase == 2:
                target_speed[0] = 105.0
                target_speed[1] = -35.0

                if cross_flag:
                    right_angle_mode = 0
                    right_angle_phase = 0
                    last_valid_err = 0.0
                    last_steer = 0.0

                elif is_center_found(bin_state):
                    right_angle_mode = 0
                    right_angle_phase = 0
                    last_valid_err = 0.0
                    last_steer = 0.0

    else:
        # ==================== 普通直线/曲线 PID 模式 ====================
        err = calc_error(bin_state)

        if mode == "CROSS":
            # 十字路口直行优先
            use_err = 0.0
        else:
            if err is None:
                use_err = last_valid_err * 1.15
                use_err = max(-2.0, min(2.0, use_err))
            else:
                if abs(err) < 0.15:
                    use_err = 0.0
                else:
                    use_err = err
                last_valid_err = err

        abs_err = abs(use_err)

        if mode == "STRAIGHT":
            cur_base = 145.0
        elif mode == "CURVE_LEFT" or mode == "CURVE_RIGHT":
            if abs_err >= 1.2:
                cur_base = 95.0
            elif abs_err >= 0.6:
                cur_base = 115.0
            else:
                cur_base = 130.0
        elif mode == "CROSS":
            cur_base = 105.0
        elif mode == "LOST":
            cur_base = 75.0
        else:
            cur_base = 100.0

        if abs_err < 0.5:
            track_pid.Kp = 4.0
            track_pid.Kd = 6.0
        elif abs_err < 1.2:
            track_pid.Kp = 5.0
            track_pid.Kd = 7.0
        else:
            track_pid.Kp = 6.0
            track_pid.Kd = 5.0

        steer_speed = track_pid.compute(use_err)

        if abs_err >= 1.2:
            steer_gain = 1.5
        elif abs_err >= 0.6:
            steer_gain = 1.2
        else:
            steer_gain = 1.0

        steer_delta = steer_speed - last_steer
        if steer_delta > MAX_STEER_DELTA:
            steer_speed = last_steer + MAX_STEER_DELTA
        elif steer_delta < -MAX_STEER_DELTA:
            steer_speed = last_steer - MAX_STEER_DELTA

        if use_err <= -1.9:
            steer_speed = -60.0
        elif use_err >= 1.9:
            steer_speed = 60.0

        last_steer = steer_speed

        target_speed[0] = cur_base + steer_speed * steer_gain
        target_speed[1] = cur_base - steer_speed * steer_gain

        target_speed[0] = limit(target_speed[0], 18.0, 180.0)
        target_speed[1] = limit(target_speed[1], 18.0, 180.0)

    if delta_interval >= print_interval:
        last_print_time = current_time
        print(
            "ADC:", adcdata,
            "State:", bin_state,
            "CrossFlag:", cross_flag,
            "Mode:", mode,
            "Dir:", last_track_dir,
            "RA_mode:", right_angle_mode,
            "RA_phase:", right_angle_phase,
            "RA_ticks:", right_angle_force_ticks,
            "TGT:", [round(target_speed[0], 1), round(target_speed[1], 1)],
            "ACT:", [round(actual_speed[0], 1), round(actual_speed[1], 1)],
            "PWM:", [int(pwm_value[0]), int(pwm_value[1])]
        )


