// ═══════════════════════════════════════════════════════════════
// CONTROLLER: GENERALIZED POLYNOMIAL WITH INTEGRAL ACTION
// Plant: Ball levitation | Hardware: ESP32 + HC-SR04 + PWM fan
// Platform: Arduino (ESP32)
//
// 3rd order model with RLS estimation. Uses Sylvester matrix
// to solve the Diophantine equation with integral action via
// difference operator (z-1).
// ═══════════════════════════════════════════════════════════════
using namespace BLA;

//  Preprocessor macros for quick configuration

#define TRIG_PIN    14
#define ECHO_PIN    13
#define PWM_PIN     27
#define PWM_FREQ    1000
#define PWM_RES     8
#define TS          100       // Sampling period 

//  Plant physical limits

#define REF_MIN     7         
#define REF_MAX     35        
#define U_MIN      -35        
#define U_MAX       35        
#define PWM_LOW     196      
#define PWM_HIGH    218      

//  RLS Estimator
//  RLS identifies the plant model online
//  LAMBDA: forgetting factor


// ORIGINAL = -0.756039, -0.043653, -0.199548, -0.021742, 0.003863, -0.020670
BLA::Matrix<6>   theta  = {-0.756039, -0.043653, -0.199548, -0.021742, 0.003863, -0.020670};
BLA::Matrix<6>   theta1 = theta;
BLA::Matrix<6,6> P      = BLA::Eye<6,6>() * 10000000.0f;
BLA::Matrix<6,6> Mp;
BLA::Matrix<6>   fi     = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
const float      UMBRAL = 0.05f;
const float      LAMBDA = 0.98f;
int              estado = 0;    
// mode: 0 = RLS, 1 = gradient

//  Generalized polynomial controller + pole placement

const float POLO = 0.50f;
BLA::Matrix<8> polos = {POLO, POLO, POLO, POLO, POLO, POLO, POLO, POLO};

float p1 = 0, p2 = 0, p3 = 0, p4 = 0;  // Coefs del polinomio P
float l0 = 0, l1 = 0, l2 = 0, l3 = 0;  // Coefs del polinomio L

//  Control variables

float ref  = 20.0f;    // Desired reference 
float u    = 0.0f;     // Current control signal

float yk_1 = 0.0f, yk_2 = 0.0f, yk_3 = 0.0f;     // Past outputs
float uk_1 = 0.0f, uk_2 = 0.0f, uk_3 = 0.0f;     // Past controls
float uk_4 = 0.0f, uk_5 = 0.0f;                  // More past controls (integral)
float ek   = 0.0f, ek_1 = 0.0f, ek_2 = 0.0f;     // Errors
float ek_3 = 0.0f, ek_4 = 0.0f;

//  Moving average filter

const int NUM_LECTURAS = 5;
float lecturas[NUM_LECTURAS] = {0};
int   readingIndex = 0;
float suma = 0.0f;

unsigned long prevTime = 0;

//  Function prototypes

void           initializeFilter();
float          readSensor();
void           readSerialReference();
BLA::Matrix<9> Poly(BLA::Matrix<8> p);
void           computeController();
float          runEstimator(float yk);
void           runController(float yk, float ye);
void           printSerial(float yk, float ye);

//  SETUP 

void setup() {
  Serial.begin(115200);
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  initializeFilter();
  computeController();
  prevTime = millis();
}

//  LOOP — every TS ms: read → estimate → control

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - prevTime < TS) return;
  prevTime = currentTime;

  readSerialReference();
  float yk = readSensor();
  float ye = runEstimator(yk);
  runController(yk, ye);
}

//  Fills the buffer with the first real sensor reading

void initializeFilter() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long  dur  = pulseIn(ECHO_PIN, HIGH, 30000);
  float dist = (dur == 0) ? 20.0f : dur * 0.0343f / 2.0f;

  for (int i = 0; i < NUM_LECTURAS; i++) lecturas[i] = dist;
  suma          = dist * NUM_LECTURAS;
  readingIndex = 0;

  Serial.print("Filter initialized with: ");
  Serial.println(dist);
}


//  Characteristic polynomial

BLA::Matrix<9> Poly(BLA::Matrix<8> p) {
  BLA::Matrix<9> coeffs = {1, 0, 0, 0, 0, 0, 0, 0, 0};
  for (int i = 0; i < 8; i++) {
    for (int j = 8; j > 0; j--) {
      coeffs(j) = coeffs(j) - p(i) * coeffs(j - 1);
    }
  }
  return coeffs;
}


//  Controller calculation

void computeController() {
  float a1 = theta(0), a2 = theta(1), a3 = theta(2);
  float b0 = theta(3), b1 = theta(4), b2 = theta(5);

  // Sylvester matrix with (z-1) included
  BLA::Matrix<8,8> A = {
    1,      0,       0,       0,       0,  0,  0,  0,
    a1-1,   1,       0,       0,       b0, 0,  0,  0,
    -a1+a2, a1-1,    1,       0,       b1, b0, 0,  0,
    -a2+a3, -a1+a2,  a1-1,    1,       b2, b1, b0, 0,
    -a3,    -a2+a3,  -a1+a2,  a1-1,    0,  b2, b1, b0,
    0,      -a3,     -a2+a3,  -a1+a2,  0,  0,  b2, b1,
    0,      0,       -a3,     -a2+a3,  0,  0,  0,  b2,
    0,      0,       0,       -a3,     0,  0,  0,  0
  };

  BLA::Matrix<9> alpha = Poly(polos);

  BLA::Matrix<8> b_vec = {
    alpha(1) + 1 - a1,
    alpha(2) + a1,
    alpha(3) + a2 - a3,
    alpha(4) + a3,
    alpha(5),
    alpha(6),
    alpha(7),
    alpha(8)
  };

  BLA::Matrix<8,8> A_inv = A;
  bool ok = BLA::Invert(A_inv);
  if (!ok) {
    Serial.println("ERROR: Singular matrix A");
    return;
  }
  BLA::Matrix<8> X = A_inv * b_vec;

  p1 = X(0); p2 = X(1); p3 = X(2); p4 = X(3);
  l0 = X(4); l1 = X(5); l2 = X(6); l3 = X(7);

  Serial.print("Controller -> p1:"); Serial.print(p1, 6);
  Serial.print(" p2:"); Serial.print(p2, 6);
  Serial.print(" p3:"); Serial.print(p3, 6);
  Serial.print(" p4:"); Serial.print(p4, 6);
  Serial.print(" | l0:"); Serial.print(l0, 6);
  Serial.print(" l1:"); Serial.print(l1, 6);
  Serial.print(" l2:"); Serial.print(l2, 6);
  Serial.print(" l3:"); Serial.println(l3, 6);
}

//  RLS ESTIMATOR WITH FORGETTING FACTOR
//  Allows forgetting a percentage of memory depending on control difficulty

float runEstimator(float yk) {
  fi = {-yk_1, -yk_2, -yk_3, uk_1, uk_2, uk_3};

  float ye = (~fi * theta1)(0, 0);
  float ee = yk - ye;

  if (abs(ee) >= UMBRAL) {
    estado   = 1;
    float x1 = (~fi * fi)(0, 0);
    float x2 = ee / ((x1 == 0) ? 0.001f : x1);
    theta    = theta1 + fi * x2;
    Mp       = P / LAMBDA;
  } else {
    estado    = 0;
    float den = (~fi * P * fi)(0, 0);
    theta     = theta1 + (P * fi) * (ee / (LAMBDA + den));
    Mp        = (P - ((P * fi * ~fi * P) / (LAMBDA + den))) / LAMBDA;
  }

  theta1 = theta;
  P      = Mp;
  return ye;
}

//  Generalized polynomial control

void runController(float yk, float ye) {
  ek = ref - yk;

  // U component: includes integral action via differences
  float uk_parte_u = -(p1-1)*uk_1 - (p2-p1)*uk_2 - (p3-p2)*uk_3
                     - (p4-p3)*uk_4 - (-p4)*uk_5;
  // E component: L(z) polynomial over the error
  float uk_parte_e = l0*ek_1 + l1*ek_2 + l2*ek_3 + l3*ek_4;

  u = uk_parte_u + uk_parte_e;
  u = constrain(u, U_MIN, U_MAX);

  int pwm_out = map(u, U_MIN, U_MAX, PWM_LOW, PWM_HIGH);
  pwm_out = constrain(pwm_out, 0, 255);
  ledcWrite(PWM_PIN, pwm_out);

  // Update past states
  uk_5 = uk_4; uk_4 = uk_3; uk_3 = uk_2; uk_2 = uk_1; uk_1 = u;
  ek_4 = ek_3; ek_3 = ek_2; ek_2 = ek_1; ek_1 = ek;
  yk_3 = yk_2; yk_2 = yk_1; yk_1 = yk;

  printSerial(yk, ye);
}

//  HC-SR04 SENSOR READING
//  Limited to 4 and 40 cm for safety

float readSensor() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return yk_1;

  float distance = duration * 0.0343f / 2.0f;
  if (distance < 4.0f || distance > 40.0f) return yk_1;

  suma -= lecturas[readingIndex];
  lecturas[readingIndex] = distance;
  suma += distance;
  readingIndex = (readingIndex + 1) % NUM_LECTURAS;
  return suma / NUM_LECTURAS;
}

//  Reference reading in its own function to prevent invalid values

void readSerialReference() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    float newRef = input.toFloat();
    if (newRef > REF_MIN && newRef < REF_MAX) {
      ref = newRef;
      Serial.print("New reference: "); Serial.println(ref);
    } else {
      Serial.print("Invalid reference. Range: ");
      Serial.print(REF_MIN); Serial.print(" - "); Serial.println(REF_MAX);
    }
  }
}

//  Serial Plotter output

void printSerial(float yk, float ye) {

  Serial.print("ref:");       Serial.print(ref);
  Serial.print(", ye:");      Serial.print(ye);
  Serial.print(", yk:");      Serial.print(yk);
  Serial.print(", ek=");      Serial.print(ek);
  Serial.print(", u=");       Serial.print(u);
  Serial.print(", RU:");      Serial.print(ref - 2);
  Serial.print(", RL:");      Serial.print(ref + 2);
  Serial.print(", UL:");      Serial.print(39);
  Serial.print(", LL:");      Serial.print(6);
  Serial.print(", theta=");   Serial.print(theta(0), 6);
  Serial.print(", ");         Serial.print(theta(1), 6);
  Serial.print(", ");         Serial.print(theta(2), 6);
  Serial.print(", ");         Serial.print(theta(3), 6);
  Serial.print(", ");         Serial.print(theta(4), 6);
  Serial.print(", ");         Serial.print(theta(5), 6);
  Serial.print(", estado=");  Serial.println(estado);
}