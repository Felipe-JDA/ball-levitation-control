"""
CONTROLLER: GENERALIZED POLYNOMIAL WITH DIRECT LOOP GAIN
Plant: Ball levitation | Hardware: ESP32 + HC-SR04 + PWM fan
Platform: MicroPython (ulab)

3rd order model with RLS online estimation. Solves the Diophantine
equation and computes a direct loop gain (Kg) for zero steady-state error.
"""

from machine import Pin, PWM, time_pulse_us
import time
import sys
import select
from ulab import numpy as np

# WiFi and IP configuration
# for sending data via UDP

WIFI_SSID    = "YOUR_SSID"
WIFI_PASS    = "YOUR_PASSWORD"
TELEPLOT_IP  = "YOUR_PC_IP"     # IP "NOTE: Update this to your PC IP address"
TELEPLOT_PORT = 47269

# ESP32 variable configuration

TRIG_PIN  = 14
ECHO_PIN  = 13
PWM_PIN   = 27
PWM_FREQ  = 1000
TS        = 100      # Periodo de muestreo

# Logical limit configuration

REF_MIN   = 7
REF_MAX   = 35
U_MIN     = -35.0
U_MAX     = 35.0
PWM_LOW   = 196
PWM_HIGH  = 218

#  ADAPTIVE RLS ESTIMATOR
# Test= -1.09426701, 0.22627249, -0.14137760, 0.00029510, 0.00019154, -0.00617746
theta  = np.array([-1.355343,0.420962,-0.066558,-0.001875,0.003412,-0.003747])
theta1 = np.array(theta)

# Covariance matrix initialized to 10 million
P = np.eye(6) * 10000000.0

fi     = np.zeros(6)
UMBRAL = 0.05
LAMBDA = 0.98
estado = 0

#  Polynomial controller with direct loop gain + pole placement

POLO  = 0.4
polos = np.array([POLO] * 6)

p1 = 0.0; p2 = 0.0; p3 = 0.0
l0 = 0.0; l1 = 0.0; l2 = 0.0
Kg = 1.0

#  Control variables

ref  = 20.0
u    = 0.0

yk_1 = 0.0; yk_2 = 0.0; yk_3 = 0.0
uk_1 = 0.0; uk_2 = 0.0; uk_3 = 0.0
ek   = 0.0; ek_1 = 0.0; ek_2 = 0.0

#  Moving average filter

NUM_LECTURAS   = 3
readings       = [0.0] * NUM_LECTURAS
reading_index = 0
filter_sum    = 0.0

#  Hardware initialization

trig = Pin(TRIG_PIN, Pin.OUT)
echo = Pin(ECHO_PIN, Pin.IN)
pwm  = PWM(Pin(PWM_PIN), freq=PWM_FREQ)

# Socket initialized in main function
_tp_sock = None
_tp_addr = None

#  Helper functions for mapping and clamping

def constrain(val, lo, hi):
    """Clamps a value to the range [lo, hi]."""
    return max(lo, min(hi, val))


def map_value(x, in_min, in_max, out_min, out_max):
    """Linearly maps x from [in_min,in_max] a [out_min,out_max]."""
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#  WiFi connection packaged outside control logic in a reusable function

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
        print("Connecting to WiFi: {}".format(WIFI_SSID))
        try:
            wlan.connect(WIFI_SSID, WIFI_PASS)
        except OSError as e:
            print("WiFi ERROR: {}".format(e))
            return False
        attempts = 0
        while not wlan.isconnected():
            time.sleep_ms(500)
            print(".", end="")
            attempts += 1
            if attempts > 20:
                print("\nERROR: Could not connect to WiFi")
                return False
        print()
    print("WiFi OK: {}".format(wlan.ifconfig()[0]))
    return True



#  TELEPLOT - UDP output for data visualization (like Arduino Serial Plotter)

def setup_teleplot():
    global _tp_sock, _tp_addr
    import socket
    _tp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    _tp_addr = (TELEPLOT_IP, TELEPLOT_PORT)
    print("Teleplot → {}:{}".format(TELEPLOT_IP, TELEPLOT_PORT))


def send_teleplot(yk, ye):
    """Sends data to Teleplot via UDP."""
    if _tp_sock is None:
        return
    try:
        # Each variable on its own line, one UDP packet
        msg = "ref:{:.2f}|g\nyk:{:.2f}|g\nye:{:.2f}|g\nek:{:.2f}|g\nu:{:.2f}|g".format(
            ref, yk, ye, ek, u)
        _tp_sock.sendto(msg.encode(), _tp_addr)
    except:
        pass

#  Uses sys package for sending data through the Python terminal

def read_serial_reference():
    global ref
    if select.select([sys.stdin], [], [], 0)[0]:
        try:
            line = sys.stdin.readline().strip()
            if line:
                new_ref = float(line)
                if REF_MIN < new_ref < REF_MAX:
                    ref = new_ref
                    print(">>> New reference: {:.1f} cm".format(ref))
                else:
                    print(">>> Valid range: {}-{} cm".format(REF_MIN, REF_MAX))
        except:
            pass

#  Characteristic polynomial

def poly_coeffs(poles):
    """Characteristic polynomial from poles."""
    n = len(poles)
    coeffs = np.zeros(n + 1)
    coeffs[0] = 1.0
    for i in range(n):
        for j in range(n, 0, -1):
            coeffs[j] = coeffs[j] + poles[i] * coeffs[j - 1]
    return coeffs

#  Filter initialization function

def initialize_filter():
    global readings, reading_index, filter_sum

    trig.value(0); time.sleep_us(2)
    trig.value(1); time.sleep_us(10)
    trig.value(0)

    dur = time_pulse_us(echo, 1, 30000)
    dist = 20.0 if dur <= 0 else dur * 0.0343 / 2.0

    readings = [dist] * NUM_LECTURAS
    filter_sum = dist * NUM_LECTURAS
    reading_index = 0
    print("Filter initialized:", dist, "cm")

#  Controller calculation

def compute_controller():
    global p1, p2, p3, l0, l1, l2, Kg

    a1 = theta[0]; a2 = theta[1]; a3 = theta[2]
    b0 = theta[3]; b1 = theta[4]; b2 = theta[5]

    # Diophantine matrix 6×6
    A = np.array([
        [1,        0,        0,        0,   0,   0  ],
        [-1+a1,    1,        0,        b0,  0,   0  ],
        [-a1+a2,   -1+a1,    1,        b1,  b0,  0  ],
        [-a2+a3,   a2-a1,   -1+a1,     b2,  b1,  b0 ],
        [0,        -a2+a3,  -a1+a2,    0,   b2,  b1 ],
        [0,        0,       -a2+a3,    0,   0,   b2 ]
    ])

    # Desired polynomial (z - POLO)^6
    alpha = poly_coeffs(polos)

    # b vector of the Diophantine equation
    b_vec = np.array([
        alpha[1] + 1 - a1,
        alpha[2] + a1 - a2,
        alpha[3] + a2 - a3,
        alpha[4] + a3,
        alpha[5],
        alpha[6]
    ])

    # Solve X = A^-1 * b (ulab has linalg.inv for matrix inversion)
    try:
        A_inv = np.linalg.inv(A)
        X = np.dot(A_inv, b_vec)

        p1 = float(X[0]); p2 = float(X[1]); p3 = float(X[2])
        l0 = float(X[3]); l1 = float(X[4]); l2 = float(X[5])

        # Direct loop gain
        denom_H = 1.0 + a1 + a2 + a3
        denom_C = 1.0 + p1 + p2 + p3
        H1 = (b0 + b1 + b2) / denom_H if abs(denom_H) > 1e-9 else 0.0
        C1 = (l0 + l1 + l2) / denom_C if abs(denom_C) > 1e-9 else 0.0
        H1C1 = H1 * C1
        if abs(H1C1) < 0.001:
            H1C1 = -0.001 if H1C1 < 0 else 0.001
        Kg = (1.0 + H1C1) / H1C1
        Kg = constrain(Kg, -5.0, 5.0)

        print("p1={:.6f}  p2={:.6f}  p3={:.6f}".format(p1, p2, p3))
        print("l0={:.6f}  l1={:.6f}  l2={:.6f}".format(l0, l1, l2))
        print("Kg={:.6f}".format(Kg))

    except ValueError:
        print("ERROR: Singular Diophantine, coefficients not updated")

#  RLS Estimator + forgetting factor LAMBDA

def run_estimator(yk):
    global theta, theta1, P, fi, estado

    fi = np.array([-yk_1, -yk_2, -yk_3, uk_1, uk_2, uk_3])

    ye = float(np.dot(fi, theta1))
    ee = yk - ye

    if abs(ee) >= UMBRAL:
        # Gradient mode 
        estado = 1
        x1 = float(np.dot(fi, fi))
        x2 = ee / (x1 if x1 != 0 else 0.001)
        theta = theta1 + fi * x2
        P = P * (1.0 / LAMBDA)
    else:
        # RLS mode
        estado = 0
        Pfi = np.dot(P, fi)
        den = float(np.dot(fi, Pfi))
        gain = ee / (LAMBDA + den)
        theta = theta1 + Pfi * gain

        # Update P
        Pfi_col = Pfi.reshape((6, 1))
        Pfi_row = Pfi.reshape((1, 6))
        P = (P - np.dot(Pfi_col, Pfi_row) * (1.0 / (LAMBDA + den))) * (1.0 / LAMBDA)

    # NaN protection: if theta has NaN, restore data
    tiene_nan = False
    for i in range(6):
        v = float(theta[i])
        if v != v: 
            tiene_nan = True
            break
    if tiene_nan:
        theta = np.array(theta1)  
        P = np.eye(6) * 1000.0    
        ye = yk                    

    traza = 0.0
    for i in range(6):
        traza += float(P[i][i])
    if traza > 1e12 or traza < 0:
        P = np.eye(6) * 1000.0

    theta1 = np.array(theta)
    return ye


#  POLYNOMIAL CONTROLLER + Kg

def run_controller(yk, ye):
    global u, ek, ek_1, ek_2
    global uk_1, uk_2, uk_3
    global yk_1, yk_2, yk_3

    ek = (Kg * ref) - yk

    u = (-p1 * uk_1 - p2 * uk_2 - p3 * uk_3
         + l0 * ek   + l1 * ek_1  + l2 * ek_2)
    u = constrain(u, U_MIN, U_MAX)

    # MicroPython PWM: 10 bits mapped to 8 bits
    pwm_8bit = map_value(u, U_MIN, U_MAX, PWM_LOW, PWM_HIGH)
    pwm_8bit = int(constrain(pwm_8bit, 0, 255))
    pwm_10bit = pwm_8bit * 1023 // 255
    pwm.duty(pwm_10bit)

    # Update memory registers
    uk_3 = uk_2; uk_2 = uk_1; uk_1 = u
    ek_2 = ek_1; ek_1 = ek
    yk_3 = yk_2; yk_2 = yk_1; yk_1 = yk

    print_serial(yk, ye)

# Sensor reading

def read_sensor():
    global filter_sum, reading_index

    trig.value(0); time.sleep_us(2)
    trig.value(1); time.sleep_us(10)
    trig.value(0)

    duration = time_pulse_us(echo, 1, 30000)
    if duration <= 0:
        return yk_1

    distance = duration * 0.0343 / 2.0
    if distance < 4.0 or distance > 40.0:
        return yk_1

    filter_sum -= readings[reading_index]
    readings[reading_index] = distance
    filter_sum += distance
    reading_index = (reading_index + 1) % NUM_LECTURAS
    return filter_sum / NUM_LECTURAS

# Serial output + Teleplot

def print_serial(yk, ye):
    print(
        "ref:{:.1f} yk:{:.2f} ye:{:.2f} ek:{:.2f} u:{:.2f} "
        "Kg:{:.4f} estado:{} "
        "theta:[{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}]".format(
            ref, yk, ye, ek, u, Kg, estado,
            float(theta[0]), float(theta[1]), float(theta[2]),
            float(theta[3]), float(theta[4]), float(theta[5])
        )
    )
    
    send_teleplot(yk, ye)

# Main is the program entry point containing setup and loop

def main():
    global yk_1

    print("=" * 50)
    print("  Polinomial generalizado con kg — MicroPython")
    print("  TS = {} ms".format(TS))
    print("  Teleplot: {}:{}".format(TELEPLOT_IP, TELEPLOT_PORT))
    print("=" * 50)

    # Connect WiFi and configure Teleplot
    if connect_wifi():
        setup_teleplot()
    else:
        print("No Teleplot (no WiFi)")

    initialize_filter()
    compute_controller()
    print("-" * 50)
    print("Type a number in the terminal to change reference")
    print("-" * 50)

    prev_time = time.ticks_ms()

    while True:
        now = time.ticks_ms()
        if time.ticks_diff(now, prev_time) < TS:
            continue
        prev_time = now

        read_serial_reference()
        yk = read_sensor()
        ye = run_estimator(yk)
        run_controller(yk, ye)

# Entry point

if __name__ == "__main__":
    main()