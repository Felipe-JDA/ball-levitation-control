# ═══════════════════════════════════════════════════════════════
# CONTROLLER: NEURAL NETWORK — PURE INVERSE MODEL
# Plant: Ball levitation | Hardware: ESP32 + HC-SR04 + PWM fan
# Platform: MicroPython (ulab)
#
# The controller uses a Recursive Least Squares (RLS) estimator
# to learn a linear model of the plant online, then inverts it
# to compute the control action directly from the desired reference.
# ═══════════════════════════════════════════════════════════════

from machine import Pin, PWM, time_pulse_us
import time
import sys
import select
from ulab import numpy as np

# ── WIFI & TELEPLOT CONFIGURATION ──────────────────────────
WIFI_SSID     = "YOUR_SSID"
WIFI_PASS     = "YOUR_PASSWORD"
TELEPLOT_IP   = "YOUR_PC_IP"
TELEPLOT_PORT = 47269

# ── PIN MAPPING & SAMPLING ─────────────────────────────────
TRIG_PIN  = 14
ECHO_PIN  = 13
PWM_PIN   = 27
PWM_FREQ  = 1000
TS        = 100       # Sampling period (ms)

# ── CONTROL LIMITS ─────────────────────────────────────────
REF_MIN  = 7
REF_MAX  = 35
U_MIN    = -30.0
U_MAX    =  30.0
PWM_LOW  = 174        # PWM lower bound
PWM_HIGH = 188        # PWM upper bound

# ── RLS PARAMETERS ─────────────────────────────────────────
W4_MIN = 0.001        # Minimum physical threshold for w4 (fan effect)
WARMUP_CYCLES = 15    # Inactive cycles to stabilize filter and memory
current_cycle = 0

# ── ESTIMATION WEIGHTS (RLS) ──────────────────────────────
w_nn  = np.array([1.179789, 0.133111, -0.314370, -0.001370, -0.001061, -0.001586])
w_nn1 = np.array(w_nn)
P_nn  = np.eye(6) * 1000.0
x_nn  = np.zeros(6)

# ── SYSTEM VARIABLES ──────────────────────────────────────
ref  = 20.0
u    = 0.0
ye   = 20.0
ek   = 0.0

yk_1 = 20.0; yk_2 = 20.0; yk_3 = 20.0
uk_1 = 0.0;  uk_2 = 0.0;  uk_3 = 0.0

NUM_READINGS    = 5
readings        = [20.0] * NUM_READINGS
reading_index   = 0

# ── HARDWARE INITIALIZATION ───────────────────────────────
trig = Pin(TRIG_PIN, Pin.OUT)
echo = Pin(ECHO_PIN, Pin.IN)
pwm  = PWM(Pin(PWM_PIN), freq=PWM_FREQ)

_tp_sock = None
_tp_addr = None

# ── HELPER FUNCTIONS ──────────────────────────────────────
def constrain(val, lo, hi):
    return max(lo, min(hi, val))

def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# ── WIFI & TELEPLOT COMMUNICATION ─────────────────────────
def connect_wifi():
    import network
    wlan = network.WLAN(network.STA_IF)
    try:
        wlan.disconnect()
    except:
        pass
    wlan.active(False)
    time.sleep_ms(100)
    wlan.active(True)
    time.sleep_ms(100)

    if not wlan.isconnected():
        print(f"Connecting to WiFi: {WIFI_SSID}")
        try:
            wlan.connect(WIFI_SSID, WIFI_PASS)
        except OSError as e:
            return False
        attempts = 0
        while not wlan.isconnected():
            time.sleep_ms(500)
            attempts += 1
            if attempts > 20:
                return False
    print(f"WiFi OK: {wlan.ifconfig()[0]}")
    return True

def setup_teleplot():
    global _tp_sock, _tp_addr
    import socket
    _tp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    _tp_addr = (TELEPLOT_IP, TELEPLOT_PORT)
    print(f"Teleplot -> {TELEPLOT_IP}:{TELEPLOT_PORT}")

def send_teleplot(yk, ye):
    if _tp_sock is None: return
    try:
        msg = "ref:{:.2f}|g\nyk:{:.2f}|g\nye:{:.2f}|g\nek:{:.2f}|g\nu:{:.2f}|g\n".format(ref, yk, ye, ek, u)
        _tp_sock.sendto(msg.encode(), _tp_addr)
    except:
        pass

# ── REFERENCE CHANGE VIA SERIAL ───────────────────────────
def read_serial_reference():
    global ref
    if select.select([sys.stdin], [], [], 0)[0]:
        try:
            line = sys.stdin.readline().strip()
            if line:
                new_ref = float(line)
                if REF_MIN <= new_ref <= REF_MAX:
                    ref = new_ref
                    print(f">>> Reference updated: {ref:.1f} cm")
        except:
            pass

# ── ULTRASONIC SENSOR (MEDIAN FILTER) ─────────────────────
def initialize_filter():
    global readings, reading_index, yk_1, yk_2, yk_3

    trig.value(0); time.sleep_us(2)
    trig.value(1); time.sleep_us(10)
    trig.value(0)

    dur  = time_pulse_us(echo, 1, 30000)
    dist = 20.0 if dur <= 0 else dur * 0.0343 / 2.0

    readings = [dist] * NUM_READINGS
    reading_index = 0

    yk_1 = dist; yk_2 = dist; yk_3 = dist

def read_sensor():
    global reading_index
    trig.value(0); time.sleep_us(2)
    trig.value(1); time.sleep_us(10)
    trig.value(0)

    duration = time_pulse_us(echo, 1, 30000)
    if duration <= 0: return yk_1

    distance = duration * 0.0343 / 2.0
    if distance < 4.0 or distance > 40.0: return yk_1

    readings[reading_index] = distance
    reading_index = (reading_index + 1) % NUM_READINGS

    sorted_readings = sorted(readings)
    return sorted_readings[NUM_READINGS // 2]

# ── RLS ESTIMATOR (ONLINE MODEL LEARNING) ─────────────────
def run_forward_nn(yk):
    global w_nn, w_nn1, P_nn, x_nn

    x_nn = np.array([yk_1, yk_2, yk_3, uk_1, uk_2, uk_3])
    ye   = float(np.dot(x_nn, w_nn1))

    if current_cycle <= WARMUP_CYCLES:
        return ye

    ee = yk - ye
    Px = np.dot(P_nn, x_nn)
    denom = 1.0 + float(np.dot(x_nn, Px))
    
    K_scalar = ee / denom
    w_nn = w_nn1 + Px * K_scalar

    Px_col = Px.reshape((6, 1))
    Px_row = Px.reshape((1, 6))
    P_nn = P_nn - np.dot(Px_col, Px_row) * (1.0 / denom)

    # Silent mathematical stability protection
    has_nan = False
    for i in range(6):
        if float(w_nn[i]) != float(w_nn[i]):
            has_nan = True; break
            
    if has_nan:
        w_nn = np.array(w_nn1)
        P_nn = np.eye(6) * 1000.0
        ye = yk

    trace = sum([float(P_nn[i][i]) for i in range(6)])
    if trace > 1e12 or trace < 0:
        P_nn = np.eye(6) * 1000.0

    w_nn1 = np.array(w_nn)
    return ye

# ── INVERSE CONTROL LAW ───────────────────────────────────
def run_controller(yk, ye):
    global u, uk_1, uk_2, uk_3
    global yk_1, yk_2, yk_3, ek

    ek = ref - yk

    w1 = float(w_nn[0]); w2 = float(w_nn[1]); w3 = float(w_nn[2])
    w4 = float(w_nn[3]); w5 = float(w_nn[4]); w6 = float(w_nn[5])

    # Ensure fan effect is physically negative
    if w4 > -W4_MIN:
        w4 = -W4_MIN
        w_nn[3] = w4
        w_nn1[3] = w4

    # Hold during warmup phase
    if current_cycle <= WARMUP_CYCLES:
        u = 0.0
    else:
        # Active Inverse Model
        if abs(w4) > W4_MIN:
            u = (ref - w1*yk - w2*yk_1 - w3*yk_2 - w5*uk_1 - w6*uk_2) / w4
        else:
            u = uk_1

    u = constrain(u, U_MIN, U_MAX)

    pwm_8bit  = map_value(u, U_MIN, U_MAX, PWM_LOW, PWM_HIGH)
    pwm_8bit  = int(constrain(pwm_8bit, 0, 255))
    pwm_10bit = pwm_8bit * 1023 // 255
    pwm.duty(pwm_10bit)

    uk_3 = uk_2; uk_2 = uk_1; uk_1 = u
    yk_3 = yk_2; yk_2 = yk_1; yk_1 = yk

    print_serial(yk, ye)

# ── SERIAL OUTPUT & MONITORING ─────────────────────────────
def print_serial(yk, ye):
    print(
        "ref:{:.1f} yk:{:.2f} ye:{:.2f} ek:{:.2f} u:{:.2f} "
        "w:[{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}]".format(
            ref, yk, ye, ek, u,
            float(w_nn[0]), float(w_nn[1]), float(w_nn[2]),
            float(w_nn[3]), float(w_nn[4]), float(w_nn[5])
        )
    )
    send_teleplot(yk, ye)

# ── MAIN LOOP ─────────────────────────────────────────────
def main():
    global current_cycle

    print(" Controller: NN Inverse Model")
    print("")

    if connect_wifi(): setup_teleplot()

    initialize_filter()
    prev_time = time.ticks_ms()

    while True:
        now = time.ticks_ms()
        if time.ticks_diff(now, prev_time) < TS:
            continue
        prev_time = now

        current_cycle += 1

        read_serial_reference()
        yk = read_sensor()
        ye = run_forward_nn(yk)
        run_controller(yk, ye)

if __name__ == "__main__":
    main()
