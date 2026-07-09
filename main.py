from machine import Pin, SPI, ADC, UART, I2C, PWM, Encoder
import time
from machine import Timer
import struct
from speed_PID import SPEED_PID

'''
硬件载体：ESP32-V1
固件：ESP32_GENERIC-20260406-v1.28.0.bin

版本说明：
1. 保留原版直线、圆弧 PID、速度环、十字逻辑
2. 不再使用复杂 FSM，避免影响直线稳定性
3. 普通圆弧弯入口增加一次短顿/短低速过渡，避免高速冲入弯道
4. 直角弯仍使用原来的“预减速 + 强制差速转向”结构
5. 直角转向力度在原版基础上略增强，避免转不过去
6. 十字路口优先直行
'''

'''=================工具函数============================================================'''
def limit(val, low, high):
    if val < low:
        return low
    if val > high:
        return high
    return val

def approach(cur, target, step_up, step_down):
    if target > cur:
        return min(target, cur + step_up)
    return max(target, cur - step_down)


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

'''=================五路循迹 ADC 配置========================================================='''
ADC_pins = [36, 33, 32, 35, 34]
My_ADC = [ADC(Pin(x)) for x in ADC_pins]

for i in range(5):
    My_ADC[i].atten(ADC.ATTN_11DB)
    My_ADC[i].width(ADC.WIDTH_12BIT)

adcdata = [0, 0, 0, 0, 0]
ADC_value = [0.0, 0.0, 0.0, 0.0, 0.0]
State = [0, 0, 0, 0, 0]

LINE_BLACK_HIGH = True
TRACK_THRESHOLD = [700, 700, 700, 700, 700]
SENSOR_WEIGHT = [-2, -1, 0, 1, 2]


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


def calc_error_analog():
    """使用 ADC 模拟量计算误差，比二值误差更平滑。"""
    sum_w = 0
    sum_s = 0

    for i in range(5):
        if LINE_BLACK_HIGH:
            sig = adcdata[i] - TRACK_THRESHOLD[i]
        else:
            sig = TRACK_THRESHOLD[i] - adcdata[i]

        if sig < 0:
            sig = 0

        sum_w += SENSOR_WEIGHT[i] * sig
        sum_s += sig

    if sum_s < 40:
        return None

    return sum_w / sum_s


'''=================十字路口短时记忆========================================================='''
last_s0_on_time = -1000
last_s4_on_time = -1000
CROSS_DETECT_WINDOW = 130


def is_cross_priority(bin_state, now_ms):
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
    s0, s1, s2, s3, s4 = bin_state
    pat = "{}{}{}{}{}".format(s0, s1, s2, s3, s4)

    if cross_flag:
        return "CROSS", 0

    if pat == "00000":
        return "LOST", last_dir

    if pat == "11111":
        return "CROSS", 0

    if pat in ("00100", "01110"):
        return "STRAIGHT", 0

    if pat == "11100":
        return "RIGHT_ANGLE_LEFT", -1

    if pat == "00111":
        return "RIGHT_ANGLE_RIGHT", 1

    if pat in ("01100", "01000", "11000", "11110", "10000"):
        return "CURVE_LEFT", -1

    if pat in ("00110", "00010", "00011", "01111", "00001"):
        return "CURVE_RIGHT", 1

    err = calc_error(bin_state)

    if err is None:
        return "LOST", last_dir
    elif err < -0.6:
        return "CURVE_LEFT", -1
    elif err > 0.6:
        return "CURVE_RIGHT", 1
    else:
        return "STRAIGHT", 0


'''=================直线稳定加速逻辑======================================================'''
def is_stable_straight(bin_state, mode, abs_err):
    pat = "{}{}{}{}{}".format(bin_state[0], bin_state[1], bin_state[2], bin_state[3], bin_state[4])

    if mode != "STRAIGHT":
        return False

    if abs_err > 0.37:
        return False

    if pat not in ("00100", "01110"):
        return False

    return True


def is_hard_turn_warning(bin_state, mode, abs_err):
    s0, s1, s2, s3, s4 = bin_state

    if mode in ("RIGHT_ANGLE_LEFT", "RIGHT_ANGLE_RIGHT", "LOST"):
        return True

    if s0 == 1 or s4 == 1:
        return True

    if abs_err > 0.85:
        return True

    return False


def is_soft_turn_warning(bin_state, mode, abs_err):
    s0, s1, s2, s3, s4 = bin_state
    pat = "{}{}{}{}{}".format(s0, s1, s2, s3, s4)

    if mode in ("CURVE_LEFT", "CURVE_RIGHT"):
        return True

    if abs_err > 0.40:
        return True

    if pat in ("01100", "00110", "01000", "00010", "11000", "00011", "11110", "01111"):
        return True

    return False


'''=================循迹 PID============================================================'''
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

        self.filter_alpha_slow = 0.50
        self.filter_alpha_fast = 0.16

    def compute(self, err):
        abs_err = abs(err)

        if abs_err < 0.8:
            alpha = self.filter_alpha_slow
        else:
            alpha = self.filter_alpha_fast

        self.filtered_err = alpha * self.filtered_err + (1 - alpha) * err

        p = self.Kp * err

        if abs_err < 0.7:
            self.integral += err
        elif abs_err > 1.4:
            self.integral = 0

        self.integral = max(-self.int_limit, min(self.int_limit, self.integral))
        i = self.Ki * self.integral

        raw_d = self.filtered_err - self.last_filtered_err
        filter_d = 0.45 * raw_d + 0.55 * self.last_filter_d
        d = self.Kd * filter_d
        self.last_filter_d = filter_d

        self.last_err = err
        self.last_filtered_err = self.filtered_err

        steer = p + i + d
        return max(-self.steer_limit, min(self.steer_limit, steer))


track_pid = LinePID(
    kp=4.6,
    ki=0.004,
    kd=7.2,
    int_limit=7,
    steer_limit=72.0
)


'''=================编码器配置============================================================'''
encoder1 = Encoder(
    0,
    Pin(17, Pin.IN, Pin.PULL_UP),
    Pin(16, Pin.IN, Pin.PULL_UP),
    filter_ns=20,
    phases=1
)

encoder2 = Encoder(
    4,
    Pin(18, Pin.IN, Pin.PULL_UP),
    Pin(19, Pin.IN, Pin.PULL_UP),
    filter_ns=20,
    phases=1
)

encoder1_counter = [0, 0]
encoder2_counter = [0, 0]
PULSES_PER_ROUND = 10000


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


'''=================速度环 PID 配置============================================================'''
L_DEADZONE = 560
R_DEADZONE = 580

L_PWM_pid = SPEED_PID(
    mode=0,
    Kp=1.60,
    Ki=0.76,
    Kd=0.02,
    max_integral=2200,
    max_output=1023,
    min_output=-1023,
    DeadZone=L_DEADZONE
)

R_PWM_pid = SPEED_PID(
    mode=0,
    Kp=1.60,
    Ki=0.76,
    Kd=0.02,
    max_integral=2200,
    max_output=1023,
    min_output=-1023,
    DeadZone=R_DEADZONE
)


'''=================全局控制变量============================================================'''
actual_speed = [0.0, 0.0]
New_actual_speed = [0.0, 0.0]
target_speed = [0.0, 0.0]
pwm_value = [0.0, 0.0]

target_pwm_floor = 0
current_pwm_floor = 0

fast_brake_ticks = 0
FAST_BRAKE_PWM = 250


'''=================定时器中断：速度环============================================================'''
def Timer0_Pro(t):
    global current_pwm_floor, fast_brake_ticks

    New_actual_speed[0], New_actual_speed[1] = Get_encoder_rpm()

    K1 = 0.48
    K2 = 0.48

    actual_speed[0] = K1 * actual_speed[0] + (1 - K1) * New_actual_speed[0]
    actual_speed[1] = K2 * actual_speed[1] + (1 - K2) * New_actual_speed[1]

    current_pwm_floor = approach(current_pwm_floor, target_pwm_floor, 12, 45)

    if fast_brake_ticks > 0:
        fast_brake_ticks -= 1
        pwm_value[0] = -FAST_BRAKE_PWM
        pwm_value[1] = -FAST_BRAKE_PWM
        motor(pwm_value[0], pwm_value[1])
        return

    pwm_value[0] = L_PWM_pid.compute(target_speed[0], actual_speed[0])
    pwm_value[1] = R_PWM_pid.compute(target_speed[1], actual_speed[1])

    if current_pwm_floor > 0:
        if target_speed[0] > 340 and target_speed[1] > 340:
            if pwm_value[0] > 0 and pwm_value[0] < current_pwm_floor:
                pwm_value[0] = current_pwm_floor
            if pwm_value[1] > 0 and pwm_value[1] < current_pwm_floor:
                pwm_value[1] = current_pwm_floor

    motor(pwm_value[0], pwm_value[1])


'''=================循迹状态变量==============================================================='''
last_valid_err = 0.0
last_steer = 0.0
MAX_STEER_DELTA = 22.0

last_track_dir = 0
lost_count = 0

# 直角模式变量
right_angle_mode = 0
right_angle_phase = 0
right_angle_force_ticks = 0

# 原来 16，略增强到 18
RIGHT_ANGLE_FORCE_TICKS = 18

# 直线渐进加速变量
straight_hold_count = 0
straight_boost = 0.0
boost_block_ticks = 0

STRAIGHT_HOLD_TICKS = 2
STRAIGHT_BOOST_MAX = 210.0
STRAIGHT_BOOST_UP = 1.6
STRAIGHT_BOOST_DOWN_HARD = 22.0
STRAIGHT_BOOST_DOWN_SOFT = 4.0
BOOST_BLOCK_AFTER_TURN = 6

soft_warning_count = 0
stable_confirm_count = 0
SOFT_WARNING_CONFIRM_TICKS = 2
STRAIGHT_CONFIRM_TICKS = 2

# 弯前预减速准备状态，只给直角弯使用
turn_prepare_mode = 0
turn_prepare_ticks = 0
RIGHT_ANGLE_PREPARE_TICKS = 7
PREPARE_BASE_SPEED = 78.0
PREPARE_STEER_LIMIT = 38.0

# ================= 普通圆弧弯入口短顿 =================
# 只在刚进入圆弧弯时触发一次，避免一直刹车
bend_pause_ticks = 0
bend_pause_dir = 0
bend_cooldown_ticks = 0
last_mode_for_bend = "STRAIGHT"

# 主循环 10ms 一次，3 表示约 30ms 的低速过渡
BEND_PAUSE_TICKS = 3

# 防止同一个弯内反复触发短顿
BEND_COOLDOWN_TICKS = 4

# 普通弯入口短顿的低速 PID 基础速度
BEND_PAUSE_BASE_SPEED = 85.0
BEND_PAUSE_STEER_LIMIT = 45.0

# 进入普通弯时轻微反拖，1 个速度环周期约 20ms
BEND_ENTRY_BRAKE_TICKS = 1


'''=================辅助判断==============================================================='''
def is_center_found(bin_state):
    pat = "{}{}{}{}{}".format(bin_state[0], bin_state[1], bin_state[2], bin_state[3], bin_state[4])

    if pat in ("00100", "01110", "00110", "01100"):
        return True

    return False


'''=================主循环==============================================================='''
start_time = time.ticks_ms()
last_print_time = start_time
print_interval = 200

print('进入主循环-原版基础弯道短顿增强版')

MyTimA = Timer(0)
MyTimA.init(period=20, mode=Timer.PERIODIC, callback=Timer0_Pro)


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

    # ==================== 普通圆弧弯入口短顿检测 ====================
    # 只在刚从直线/十字进入圆弧弯时触发一次。
    # 不对十字触发，不对直角执行中触发。
    if bend_cooldown_ticks > 0:
        bend_cooldown_ticks -= 1

    if right_angle_mode == 0 and turn_prepare_mode == 0 and (not cross_flag):
        if mode in ("CURVE_LEFT", "CURVE_RIGHT"):
            trigger_bend_pause = False

        # 直线/十字进入圆弧
            if last_mode_for_bend in ("STRAIGHT", "CROSS"):
                trigger_bend_pause = True

        # 连续反向弯：左弯接右弯
            if last_mode_for_bend == "CURVE_LEFT" and mode == "CURVE_RIGHT":
                trigger_bend_pause = True

        # 连续反向弯：右弯接左弯
            if last_mode_for_bend == "CURVE_RIGHT" and mode == "CURVE_LEFT":
                trigger_bend_pause = True

            if trigger_bend_pause and bend_cooldown_ticks == 0:
                bend_pause_ticks = BEND_PAUSE_TICKS
                bend_pause_dir = dir_mark

                fast_brake_ticks = BEND_ENTRY_BRAKE_TICKS

                target_pwm_floor = 0
                straight_boost = 0.0
                straight_hold_count = 0
                soft_warning_count = 0
                stable_confirm_count = 0
                boost_block_ticks = BOOST_BLOCK_AFTER_TURN
    last_mode_for_bend = mode

    # ==================== 十字路口优先直行 ====================
    if cross_flag:
        turn_prepare_mode = 0
        turn_prepare_ticks = 0

        bend_pause_ticks = 0
        bend_cooldown_ticks = 0
        bend_pause_dir = 0

        soft_warning_count = 0
        stable_confirm_count = 0

        target_pwm_floor = 0
        fast_brake_ticks = 0

        if right_angle_mode != 0:
            right_angle_mode = 0
            right_angle_phase = 0
            right_angle_force_ticks = 0
            last_valid_err = 0.0
            last_steer = 0.0

    # ==================== 普通圆弧弯入口短顿执行 ====================
    if bend_pause_ticks > 0 and right_angle_mode == 0 and turn_prepare_mode == 0 and mode in ("CURVE_LEFT", "CURVE_RIGHT"):
        target_pwm_floor = 0

        err = calc_error_analog()
        if err is None:
            err = calc_error(bin_state)

        if err is None:
            if bend_pause_dir == -1:
                use_err = -1.2
            elif bend_pause_dir == 1:
                use_err = 1.2
            else:
                use_err = last_valid_err
        else:
            use_err = err
            last_valid_err = err

        pause_steer = track_pid.compute(use_err)
        pause_steer = limit(pause_steer, -BEND_PAUSE_STEER_LIMIT, BEND_PAUSE_STEER_LIMIT)

        target_speed[0] = BEND_PAUSE_BASE_SPEED + pause_steer
        target_speed[1] = BEND_PAUSE_BASE_SPEED - pause_steer

        target_speed[0] = limit(target_speed[0], 20.0, 155.0)
        target_speed[1] = limit(target_speed[1], 20.0, 155.0)

        bend_pause_ticks -= 1

        if bend_pause_ticks <= 0:
            bend_cooldown_ticks = BEND_COOLDOWN_TICKS

    # ==================== 只对直角弯进入弯前准备 ====================
    elif right_angle_mode == 0 and turn_prepare_mode == 0 and (not cross_flag):

        if mode == "RIGHT_ANGLE_LEFT":
            turn_prepare_mode = 3
            turn_prepare_ticks = RIGHT_ANGLE_PREPARE_TICKS

            bend_pause_ticks = 0
            bend_cooldown_ticks = BEND_COOLDOWN_TICKS

            fast_brake_ticks = 2
            target_pwm_floor = 0

            target_speed[0] = 0.0
            target_speed[1] = 0.0

            straight_boost = 0.0
            straight_hold_count = 0
            soft_warning_count = 0
            stable_confirm_count = 0
            boost_block_ticks = BOOST_BLOCK_AFTER_TURN

        elif mode == "RIGHT_ANGLE_RIGHT":
            turn_prepare_mode = 4
            turn_prepare_ticks = RIGHT_ANGLE_PREPARE_TICKS

            bend_pause_ticks = 0
            bend_cooldown_ticks = BEND_COOLDOWN_TICKS

            fast_brake_ticks = 2
            target_pwm_floor = 0

            target_speed[0] = 0.0
            target_speed[1] = 0.0

            straight_boost = 0.0
            straight_hold_count = 0
            soft_warning_count = 0
            stable_confirm_count = 0
            boost_block_ticks = BOOST_BLOCK_AFTER_TURN

    # ==================== 直角弯前预减速准备阶段 ====================
    if turn_prepare_mode != 0 and right_angle_mode == 0:
        target_pwm_floor = 0

        err = calc_error_analog()
        if err is None:
            err = calc_error(bin_state)

        if err is None:
            use_err = last_valid_err
        else:
            use_err = err
            last_valid_err = err

        prepare_steer = track_pid.compute(use_err)
        prepare_steer = limit(prepare_steer, -PREPARE_STEER_LIMIT, PREPARE_STEER_LIMIT)

        target_speed[0] = PREPARE_BASE_SPEED + prepare_steer
        target_speed[1] = PREPARE_BASE_SPEED - prepare_steer

        target_speed[0] = limit(target_speed[0], 15.0, 170.0)
        target_speed[1] = limit(target_speed[1], 15.0, 170.0)

        turn_prepare_ticks -= 1

        if cross_flag:
            turn_prepare_mode = 0
            turn_prepare_ticks = 0

        elif turn_prepare_ticks <= 0:
            if turn_prepare_mode == 3:
                right_angle_mode = 1
                right_angle_phase = 1
                right_angle_force_ticks = RIGHT_ANGLE_FORCE_TICKS

            elif turn_prepare_mode == 4:
                right_angle_mode = 2
                right_angle_phase = 1
                right_angle_force_ticks = RIGHT_ANGLE_FORCE_TICKS

            turn_prepare_mode = 0
            turn_prepare_ticks = 0

    # ==================== 直角执行模式 ====================
    elif right_angle_mode != 0:
        target_pwm_floor = 0

        if right_angle_mode == 1:
            if right_angle_phase == 1:
                # 左直角：左轮反转，右轮前进，较原版增强
                target_speed[0] = -145.0
                target_speed[1] = 235.0
                right_angle_force_ticks -= 1

                if cross_flag:
                    right_angle_mode = 0
                    right_angle_phase = 0
                    right_angle_force_ticks = 0
                    last_valid_err = 0.0
                    last_steer = 0.0
                    boost_block_ticks = BOOST_BLOCK_AFTER_TURN

                elif right_angle_force_ticks <= 0:
                    right_angle_phase = 2

            elif right_angle_phase == 2:
                target_speed[0] = -55.0
                target_speed[1] = 155.0

                if cross_flag:
                    right_angle_mode = 0
                    right_angle_phase = 0
                    last_valid_err = 0.0
                    last_steer = 0.0
                    boost_block_ticks = BOOST_BLOCK_AFTER_TURN

                elif is_center_found(bin_state):
                    right_angle_mode = 0
                    right_angle_phase = 0
                    last_valid_err = 0.0
                    last_steer = 0.0
                    boost_block_ticks = BOOST_BLOCK_AFTER_TURN

        elif right_angle_mode == 2:
            if right_angle_phase == 1:
                # 右直角：左轮前进，右轮反转，较原版增强
                target_speed[0] = 235.0
                target_speed[1] = -145.0
                right_angle_force_ticks -= 1

                if cross_flag:
                    right_angle_mode = 0
                    right_angle_phase = 0
                    right_angle_force_ticks = 0
                    last_valid_err = 0.0
                    last_steer = 0.0
                    boost_block_ticks = BOOST_BLOCK_AFTER_TURN

                elif right_angle_force_ticks <= 0:
                    right_angle_phase = 2

            elif right_angle_phase == 2:
                target_speed[0] = 155.0
                target_speed[1] = -55.0

                if cross_flag:
                    right_angle_mode = 0
                    right_angle_phase = 0
                    last_valid_err = 0.0
                    last_steer = 0.0
                    boost_block_ticks = BOOST_BLOCK_AFTER_TURN

                elif is_center_found(bin_state):
                    right_angle_mode = 0
                    right_angle_phase = 0
                    last_valid_err = 0.0
                    last_steer = 0.0
                    boost_block_ticks = BOOST_BLOCK_AFTER_TURN

    # ==================== 普通 PID 模式：直线 + 圆弧弯 + 十字 ====================
    else:
        err = calc_error_analog()
        if err is None:
            err = calc_error(bin_state)

        if mode == "CROSS":
            use_err = 0.0
        else:
            if err is None:
                use_err = last_valid_err * 1.12
                use_err = max(-2.0, min(2.0, use_err))
            else:
                if abs(err) < 0.12:
                    use_err = 0.0
                else:
                    use_err = err
                last_valid_err = err

        abs_err = abs(use_err)

        if mode == "STRAIGHT":
            cur_base = 300.0
        elif mode == "CURVE_LEFT" or mode == "CURVE_RIGHT":
            cur_base = 175.0
        elif mode == "CROSS":
            cur_base = 165.0
        elif mode == "LOST":
            cur_base = 70.0
        else:
            cur_base = 145.0

        if mode == "LOST":
            straight_boost = 0.0
            straight_hold_count = 0
            soft_warning_count = 0
            stable_confirm_count = 0
            boost_block_ticks = BOOST_BLOCK_AFTER_TURN
            target_pwm_floor = 0

        if mode in ("CURVE_LEFT", "CURVE_RIGHT"):
            boost_block_ticks = BOOST_BLOCK_AFTER_TURN
            target_pwm_floor = 0

        if mode == "CROSS":
            target_pwm_floor = 0

        if abs_err < 0.35:
            track_pid.Kp = 4.2
            track_pid.Kd = 5.8
        elif abs_err < 0.9:
            track_pid.Kp = 5.0
            track_pid.Kd = 6.5
        else:
            track_pid.Kp = 6.1
            track_pid.Kd = 5.2

        steer_speed = track_pid.compute(use_err)

        if abs_err >= 1.2:
            steer_gain = 1.45
        elif abs_err >= 0.6:
            steer_gain = 1.18
        else:
            steer_gain = 0.95

        steer_delta = steer_speed - last_steer
        if steer_delta > MAX_STEER_DELTA:
            steer_speed = last_steer + MAX_STEER_DELTA
        elif steer_delta < -MAX_STEER_DELTA:
            steer_speed = last_steer - MAX_STEER_DELTA

        if use_err <= -1.9:
            steer_speed = -66.0
        elif use_err >= 1.9:
            steer_speed = 66.0

        last_steer = steer_speed

        stable_straight = is_stable_straight(bin_state, mode, abs_err)
        hard_warning = is_hard_turn_warning(bin_state, mode, abs_err)
        soft_warning = is_soft_turn_warning(bin_state, mode, abs_err)

        if boost_block_ticks > 0:
            boost_block_ticks -= 1

        if stable_straight and boost_block_ticks == 0:
            stable_confirm_count += 1
        else:
            stable_confirm_count = 0

        if stable_confirm_count >= STRAIGHT_CONFIRM_TICKS:
            straight_hold_count += 1
        else:
            straight_hold_count = 0

        if hard_warning or boost_block_ticks > 0:
            straight_boost -= STRAIGHT_BOOST_DOWN_HARD
            soft_warning_count = 0

        else:
            if soft_warning:
                soft_warning_count += 1
                if soft_warning_count >= SOFT_WARNING_CONFIRM_TICKS:
                    straight_boost -= STRAIGHT_BOOST_DOWN_SOFT
            else:
                soft_warning_count = 0

                if stable_confirm_count >= STRAIGHT_CONFIRM_TICKS:
                    straight_boost += STRAIGHT_BOOST_UP
                else:
                    straight_boost -= 0.4

        straight_boost = limit(straight_boost, 0.0, STRAIGHT_BOOST_MAX)

        if mode == "STRAIGHT":
            cur_base += straight_boost

            if straight_boost > 155 and stable_confirm_count >= STRAIGHT_CONFIRM_TICKS:
                target_pwm_floor = 900
            elif straight_boost > 105 and stable_confirm_count >= STRAIGHT_CONFIRM_TICKS:
                target_pwm_floor = 850
            elif straight_boost > 55 and stable_confirm_count >= STRAIGHT_CONFIRM_TICKS:
                target_pwm_floor = 780
            else:
                target_pwm_floor = 0
        else:
            target_pwm_floor = 0

        target_speed[0] = cur_base + steer_speed * steer_gain
        target_speed[1] = cur_base - steer_speed * steer_gain

        if mode == "STRAIGHT":
            max_tgt = 610.0
        elif mode == "CURVE_LEFT" or mode == "CURVE_RIGHT":
            max_tgt = 300.0
        elif mode == "CROSS":
            max_tgt = 280.0
        elif mode == "LOST":
            max_tgt = 150.0
        else:
            max_tgt = 300.0

        target_speed[0] = limit(target_speed[0], 0.0, max_tgt)
        target_speed[1] = limit(target_speed[1], 0.0, max_tgt)

    # ==================== 调试打印 ====================
    if delta_interval >= print_interval:
        last_print_time = current_time

        print(
            "ADC:", adcdata,
            "State:", bin_state,
            "CrossFlag:", cross_flag,
            "Mode:", mode,
            "BendPause:", bend_pause_ticks,
            "BendCD:", bend_cooldown_ticks,
            "PrepMode:", turn_prepare_mode,
            "PrepTicks:", turn_prepare_ticks,
            "Boost:", round(straight_boost, 1),
            "StbCnt:", stable_confirm_count,
            "HoldCnt:", straight_hold_count,
            "SoftWarnCnt:", soft_warning_count,
            "BoostBlk:", boost_block_ticks,
            "PwmFloorT:", target_pwm_floor,
            "PwmFloorC:", int(current_pwm_floor),
            "BrakeTicks:", fast_brake_ticks,
            "Dir:", last_track_dir,
            "RA_mode:", right_angle_mode,
            "RA_phase:", right_angle_phase,
            "RA_ticks:", right_angle_force_ticks,
            "TGT:", [round(target_speed[0], 1), round(target_speed[1], 1)],
            "ACT:", [round(actual_speed[0], 1), round(actual_speed[1], 1)],
            "PWM:", [int(pwm_value[0]), int(pwm_value[1])]
        )
