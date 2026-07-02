// ═══════════════════════════════════════════════════════════════
// CONTROLLER: SEMI-ADAPTIVE PID TYPE 1
// Plant: Ball levitation | Hardware: ESP32 + HC-SR04 + PWM fan
// Platform: Arduino (ESP32)
//
// 1st order model with RLS online estimation and pole placement.
// The estimator switches between gradient and RLS modes based on
// the estimation error threshold.
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


// Test = -0.895899, -0.068219
// Original = -0.962461, -0.073605
BLA::Matrix<2>   theta  = {-0.895899, -0.068219};
BLA::Matrix<2>   theta1 = theta;
BLA::Matrix<2,2> P      = BLA::Eye<2,2>() * 10000000.0f;
BLA::Matrix<2,2> Mp;
BLA::Matrix<2>   fi     = {0.0, 0.0};
const float      UMBRAL = 0.05f;
const float      LAMBDA = 0.98f;
int              estado = 0;    
// mode: 0 = RLS, 1 = gradient

//  PID Type 1 controller + pole placement

const float POLO = 0.35f;
float       c0, c1, c2;

//  Control variables

float ref  = 20.0f;    // Reference
float u    = 0.0f;     // Current control signal
float yk_1 = 0.0f;     // y(k-1)
float uk_1 = 0.0f;     // u(k-1)
float ek   = 0.0f;     // e(k) = ref - y(k)
float ek_1 = 0.0f;     // e(k-1)
float ek_2 = 0.0f;     // e(k-2)

//  Moving average filter

const int NUM_LECTURAS = 3;
float lecturas[NUM_LECTURAS] = {0};
int   indiceLectura = 0;
float suma = 0.0f;

unsigned long tiempoAnterior = 0;

//  Function prototypes

void  inicializarFiltro();
float leerSensor();
void  leerReferenceSerial();
void  calcularCoeficientes(float a1, float b0);
float ejecutarEstimador(float yk);
void  ejecutarControlador(float yk, float ye);
void  imprimirSerial(float yk, float ye);

//  SETUP 

void setup() {
  Serial.begin(115200);
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  inicializarFiltro();
  calcularCoeficientes(theta(0), theta(1));
  tiempoAnterior = millis();
}

//  LOOP — every TS ms: read → estimate → control

void loop() {
  unsigned long tiempoActual = millis();
  if (tiempoActual - tiempoAnterior < TS) return;
  tiempoAnterior = tiempoActual;

  leerReferenceSerial();
  float yk = leerSensor();
  float ye = ejecutarEstimador(yk);
  ejecutarControlador(yk, ye);
}

//  Fills the buffer with the first real sensor reading

void inicializarFiltro() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long  dur  = pulseIn(ECHO_PIN, HIGH, 30000);
  float dist = (dur == 0) ? 20.0f : dur * 0.0343f / 2.0f;

  for (int i = 0; i < NUM_LECTURAS; i++) lecturas[i] = dist;
  suma          = dist * NUM_LECTURAS;
  indiceLectura = 0;

  Serial.print("Filter initialized with: ");
  Serial.println(dist);
}

//  Coefficient calculation

void calcularCoeficientes(float a1, float b0) {
  float p1 = POLO, p2 = POLO, p3 = POLO;
  float pol2 = -(p1 + p2 + p3);
  float pol3 =  (p1*p2 + p1*p3 + p2*p3);
  float pol4 = -(p1*p2*p3);

  c0 = (pol2 - a1 + 1) / b0;
  c1 = (pol3 + a1)      / b0;
  c2 =  pol4             / b0;

  Serial.print("Coefficients -> c0: "); Serial.print(c0, 6);
  Serial.print(" | c1: ");              Serial.print(c1, 6);
  Serial.print(" | c2: ");              Serial.println(c2, 6);
}

//  RLS ESTIMATOR WITH FORGETTING FACTOR
//  Allows forgetting a percentage of memory depending on control difficulty

float ejecutarEstimador(float yk) {
  fi = {-yk_1, uk_1};

  float ye = (~fi * theta1)(0, 0);     // Prediction
  float ee = yk - ye;                   // Estimation error

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

void ejecutarControlador(float yk, float ye) {
  ek = ref - yk;

  u = uk_1 + c0*ek + c1*ek_1 + c2*ek_2;
  u = constrain(u, U_MIN, U_MAX);

  int pwm_out = map(u, U_MIN, U_MAX, PWM_LOW, PWM_HIGH);
  pwm_out = constrain(pwm_out, 0, 255);
  ledcWrite(PWM_PIN, pwm_out);

  uk_1 = u;   yk_1 = yk;
  ek_2 = ek_1; ek_1 = ek;

  imprimirSerial(yk, ye);
}

//  HC-SR04 SENSOR READING
//  Limited to 4 and 40 cm for safety

float leerSensor() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duracion = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duracion == 0) return yk_1;

  float distancia = duracion * 0.0343f / 2.0f;
  if (distancia < 4.0f || distancia > 40.0f) return yk_1;

  suma -= lecturas[indiceLectura];
  lecturas[indiceLectura] = distancia;
  suma += distancia;
  indiceLectura = (indiceLectura + 1) % NUM_LECTURAS;
  return suma / NUM_LECTURAS;
}

//  Reference reading in its own function to prevent invalid values

void leerReferenceSerial() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    float nuevaRef = input.toFloat();
    if (nuevaRef > REF_MIN && nuevaRef < REF_MAX) {
      ref = nuevaRef;
      Serial.print("New reference: "); Serial.println(ref);
    } else {
      Serial.print("Invalid reference. Range: ");
      Serial.print(REF_MIN); Serial.print(" - "); Serial.println(REF_MAX);
    }
  }
}

//  Serial Plotter output

void imprimirSerial(float yk, float ye) {

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
  Serial.print(", estado=");  Serial.println(estado);
}