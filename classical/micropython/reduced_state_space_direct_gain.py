"""
CONTROLLER: REDUCED STATE-SPACE WITH DIRECT LOOP GAIN
Plant: Ball levitation | Hardware: ESP32 + HC-SR04 + PWM fan
Platform: MicroPython (ulab)

Reduced-order state-space controller with online RLS estimation
and direct loop gain (Kg) for zero steady-state error.
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

TRIG_PIN = 14
ECHO_PIN = 13
PWM_PIN = 27
PWM_FREQ = 1000
TS = 100    # Periodo de muestreo

# Logical limit configuration

REF_MIN = 7;  REF_MAX = 35
U_MIN = -35.0;  U_MAX = 35.0
PWM_LOW = 196;  PWM_HIGH = 218

#  ADAPTIVE RLS ESTIMATOR
# -1.09426701, 0.22627249, -0.14137760, 0.00029510, 0.00019154, -0.00617746
theta  = np.array([-1.09426701, 0.22627249, -0.14137760, 0.00029510, 0.00019154, -0.00617746])
theta1 = np.array(theta)
P = np.eye(6) * 10000000.0
fi = np.zeros(6)
UMBRAL = 0.05;  LAMBDA = 0.98;  estado = 0

# Controller espacio de estados + polos + polos observadores

polo_c = 0.45;  polo_o = 0.15

K     = np.zeros(3)
N_bar = 1.0
z_obs = np.zeros(2)
F_obs = np.zeros((2, 2))
Gy    = np.zeros(2)
Gu    = np.zeros(2)

# Control variables

ref = 20.0;  u = 0.0;  ek = 0.0
yk_1 = 0.0;  yk_2 = 0.0;  yk_3 = 0.0
uk_1 = 0.0;  uk_2 = 0.0;  uk_3 = 0.0

#  Moving average filter

NUM_LECTURAS = 3
readings = [0.0] * NUM_LECTURAS;  reading_index = 0;  filter_sum = 0.0

#  Hardware initialization

trig = Pin(TRIG_PIN, Pin.OUT);  echo = Pin(ECHO_PIN, Pin.IN)
pwm  = PWM(Pin(PWM_PIN), freq=PWM_FREQ)

_tp_sock = None
_tp_addr = None

def constrain(v, lo, hi): return max(lo, min(hi, v))
def map_value(x, a, b, c, d): return (x - a) * (d - c) / (b - a) + c

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
    if _tp_sock is None: return
    try:
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

# Moving average filter initialization

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
    print("Filter:", dist, "cm")

# Calculo del controlardor con ackerman

def compute_controller():
    global K, N_bar, F_obs, Gy, Gu

    a1, a2, a3 = float(theta[0]), float(theta[1]), float(theta[2])
    b0, b1, b2 = float(theta[3]), float(theta[4]), float(theta[5])

    A = np.array([[-a1, 1, 0], [-a2, 0, 1], [-a3, 0, 0]])
    B = np.array([b0, b1, b2])
    C = np.array([1.0, 0.0, 0.0])
    I3 = np.eye(3)

    al1 = -3.0 * polo_c
    al2 =  3.0 * polo_c * polo_c
    al3 = -polo_c ** 3

    A2 = np.dot(A, A);  A3 = np.dot(A2, A)
    phi_c = A3 + A2 * al1 + A * al2 + I3 * al3

    AB  = np.dot(A, B)
    A2B = np.dot(A, AB)
    Co = np.zeros((3, 3))
    for i in range(3):
        Co[i][0] = B[i];  Co[i][1] = AB[i];  Co[i][2] = A2B[i]

    try:
        Co_inv = np.linalg.inv(Co)
        e3 = np.array([0.0, 0.0, 1.0])
        K = np.dot(np.dot(e3, Co_inv), phi_c)
    except:
        print("ERROR: Controlabilidad singular")

    try:
        BK = np.array([[B[i] * K[j] for j in range(3)] for i in range(3)])
        M = I3 - A + BK
        M_inv = np.linalg.inv(M)
        N_bar = 1.0 / float(np.dot(C, np.dot(M_inv, B)))
    except:
        print("ERROR: N_bar singular")

    Lobs = np.array([-2.0 * polo_o, polo_o * polo_o])
    A12 = np.array([[1.0, 0.0]])
    A22 = np.array([[0.0, 1.0], [0.0, 0.0]])

    F_obs = A22 - np.dot(Lobs.reshape((2, 1)), A12)
    Gy[:] = (np.dot(F_obs, Lobs) + np.array([-a2, -a3]) - Lobs * (-a1))
    Gu[:] = (np.array([b1, b2]) - Lobs * b0)

    print("K:", K)
    print("N_bar:", N_bar)

# Estimador RLS

def run_estimator(yk):
    global theta, theta1, P, fi, estado
    fi = np.array([-yk_1, -yk_2, -yk_3, uk_1, uk_2, uk_3])
    ye = float(np.dot(fi, theta1))
    ee = yk - ye
    if abs(ee) >= UMBRAL:
        estado = 1
        x1 = float(np.dot(fi, fi))
        theta = theta1 + fi * (ee / (x1 if x1 != 0 else 0.001))
        P = P * (1.0 / LAMBDA)
    else:
        estado = 0
        Pfi = np.dot(P, fi)
        den = float(np.dot(fi, Pfi))
        theta = theta1 + Pfi * (ee / (LAMBDA + den))
        Pfi_c = Pfi.reshape((6, 1));  Pfi_r = Pfi.reshape((1, 6))
        P = (P - np.dot(Pfi_c, Pfi_r) * (1.0 / (LAMBDA + den))) * (1.0 / LAMBDA)

    # Protección NaN
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

    # Protección traza P
    traza = 0.0
    for i in range(6):
        traza += float(P[i][i])
    if traza > 1e12 or traza < 0:
        P = np.eye(6) * 1000.0

    theta1 = np.array(theta)
    return ye

# Controller u

def run_controller(yk, ye):
    global u, ek, uk_1, uk_2, uk_3, yk_1, yk_2, yk_3, z_obs

    x_red = z_obs + np.array([-2.0 * polo_o, polo_o * polo_o]) * yk
    x = np.array([yk, float(x_red[0]), float(x_red[1])])

    u_ctrl = float(np.dot(K, x))
    u = (N_bar * ref) - u_ctrl
    u = constrain(u, U_MIN, U_MAX)

    pwm_8bit = int(constrain(map_value(u, U_MIN, U_MAX, PWM_LOW, PWM_HIGH), 0, 255))
    pwm.duty(pwm_8bit * 1023 // 255)

    z_obs = np.dot(F_obs, z_obs) + Gy * yk + Gu * u
    uk_3 = uk_2;  uk_2 = uk_1;  uk_1 = u
    yk_3 = yk_2;  yk_2 = yk_1;  yk_1 = yk
    ek = ref - yk
    print_serial(yk, ye)

# Sensor reading

def read_sensor():
    global filter_sum, reading_index
    trig.value(0); time.sleep_us(2)
    trig.value(1); time.sleep_us(10)
    trig.value(0)
    dur = time_pulse_us(echo, 1, 30000)
    if dur <= 0: return yk_1
    d = dur * 0.0343 / 2.0
    if d < 4.0 or d > 40.0: return yk_1
    filter_sum -= readings[reading_index]
    readings[reading_index] = d
    filter_sum += d
    reading_index = (reading_index + 1) % NUM_LECTURAS
    return filter_sum / NUM_LECTURAS

# Serial output + Teleplot

def print_serial(yk, ye):
    print("ref:{:.1f} yk:{:.2f} ye:{:.2f} ek:{:.2f} u:{:.2f} Nbar:{:.4f} estado:{} "
          "theta:[{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}]".format(
        ref, yk, ye, ek, u, N_bar, estado,
        float(theta[0]), float(theta[1]), float(theta[2]),
        float(theta[3]), float(theta[4]), float(theta[5])))
    send_teleplot(yk, ye)

# Main is the program entry point containing setup and loop

def main():
    print("=" * 50)
    print("  EE Reducido + Lazo directo — MicroPython")
    print("  TS = {} ms".format(TS))
    print("  Teleplot: {}:{}".format(TELEPLOT_IP, TELEPLOT_PORT))
    print("=" * 50)

    if connect_wifi():
        setup_teleplot()
    else:
        print("No Teleplot (no WiFi)")

    initialize_filter()
    compute_controller()
    print("-" * 50)
    print("Type a number in the terminal to change reference")
    print("-" * 50)

    t_ant = time.ticks_ms()
    while True:
        t = time.ticks_ms()
        if time.ticks_diff(t, t_ant) < TS: continue
        t_ant = t
        read_serial_reference()
        yk = read_sensor()
        ye = run_estimator(yk)
        run_controller(yk, ye)

if __name__ == "__main__":
    main()