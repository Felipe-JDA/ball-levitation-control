// ═══════════════════════════════════════════════════════════════
// CONTROLLER: SEMI-ADAPTIVE PID TYPE 2
// Plant: Ball levitation | Hardware: ESP32 + HC-SR04 + PWM fan
// Platform: Arduino (ESP32)
//
// 2nd order model with RLS online estimation. Solves the
// Diophantine equation for pole placement with 4 coefficients.
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
#define PWM_HIGH    216       

//  RLS Estimator
//  RLS identifies the plant model online
//  LAMBDA: forgetting factor


// ORIGINAL = -0.811888, -0.188585, 0.009829, -0.019037
// TEST = -0.971311, -0.037419, -0.015961, -0.009705
BLA::Matrix<4>   theta  = {-1.554093, 0.539169, -0.014852, -0.004281};
BLA::Matrix<4>   theta1 = theta;
BLA::Matrix<4,4> P      = BLA::Eye<4,4>() * 100000.0f;
BLA::Matrix<4,4> Mp;
BLA::Matrix<4>   fi     = {0.0, 0.0, 0.0, 0.0};
const float      UMBRAL = 0.05f;
const float      LAMBDA = 0.98f;
int              estado = 0;    
// mode: 0 = RLS, 1 = gradient

//  PID Type 2 controller + pole placement

const float POLO = 0.40f;
float       c0, c1, c2, c3;

//  Control variables

float ref  = 20.0f;                               // Desired reference 
float u    = 0.0f;                                // Current control signal
float yk_1 = 0.0f, yk_2 = 0.0f;                   // Past outputs
float uk_1 = 0.0f, uk_2 = 0.0f;                   // Past controls
float ek   = 0.0f, ek_1 = 0.0f, ek_2 = 0.0f;      // Errors

//  Moving average filter

const int NUM_LECTURAS = 3;
float lecturas[NUM_LECTURAS] = {0};
int   readingIndex = 0;
float suma = 0.0f;

unsigned long prevTime = 0;

//  Function prototypes

void  initializeFilter();
float readSensor();
void  readSerialReference();
void  computeCoefficients(float a1, float a2, float b0, float b1);
float runEstimator(float yk);
void  runController(float yk, float ye);
void  printSerial(float yk, float ye);

//  SETUP

void setup() {
  Serial.begin(115200);
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  initializeFilter();
  computeCoefficients(theta(0), theta(1), theta(2), theta(3));
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

//  Coefficient calculation

void computeCoefficients(float a1, float a2, float b0, float b1) {
  float p  = POLO;
  float p2 = p  * p;
  float p3 = p2 * p;
  float p4 = p3 * p;

  // (Diophantine equation)
  BLA::Matrix<4,4> A = {
    b0,  0,   0,   1,
    b1,  b0,  0,   -1 + a1,
    0,   b1,  b0,  -a1 + a2,
    0,   0,   b1,  -a2
  };

  // Desired coefficient vector
  BLA::Matrix<4> b_vec = {
    -4*p   + 1 - a1,
     6*p2  + a1 - a2,
    -4*p3  + a2,
     p4
  };

  BLA::Matrix<4,4> Ainv = A;
  bool ok = Invert(Ainv);

  if (ok) {
    BLA::Matrix<4> X = Ainv * b_vec;
    c0 = X(0);  c1 = X(1);  c2 = X(2);  c3 = X(3);

    Serial.print("Coefficients -> c0: "); Serial.print(c0, 6);
    Serial.print(" | c1: ");              Serial.print(c1, 6);
    Serial.print(" | c2: ");              Serial.print(c2, 6);
    Serial.print(" | c3: ");              Serial.println(c3, 6);
  } else {
    Serial.println("ERROR: Singular matrix. Check initial theta.");
  }
}

//  RLS ESTIMATOR WITH FORGETTING FACTOR
//  Allows forgetting a percentage of memory depending on control difficulty

float runEstimator(float yk) {
  fi = {-yk_1, -yk_2, uk_1, uk_2};

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

// PID Type 1 control + PWM with constrain

void runController(float yk, float ye) {
  ek = ref - yk;

  u = -(c3 - 1)*uk_1 + c3*uk_2 + c0*ek + c1*ek_1 + c2*ek_2;
  u = constrain(u, U_MIN, U_MAX);

  int pwm_out = map(u, U_MIN, U_MAX, PWM_LOW, PWM_HIGH);
  pwm_out = constrain(pwm_out, 0, 255);
  ledcWrite(PWM_PIN, pwm_out);

  uk_2 = uk_1;  uk_1 = u;
  yk_2 = yk_1;  yk_1 = yk;
  ek_2 = ek_1;  ek_1 = ek;

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
  Serial.print(", estado=");  Serial.println(estado);
}