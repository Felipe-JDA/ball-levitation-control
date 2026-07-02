// ═══════════════════════════════════════════════════════════════
// CONTROLLER: FUZZY LOGIC — 49 RULES (7×7)
// Plant: Ball levitation | Hardware: ESP32 + HC-SR04 + PWM fan
// Platform: Arduino (ESP32)
//
// Uses Gaussian membership functions for error and error derivative,
// with centroid defuzzification over a 7×7 rule base.
// ═══════════════════════════════════════════════════════════════

// ── Pin Configuration ───────────────────────────────────────
#define TRIG_PIN    14
#define ECHO_PIN    13
#define PWM_PIN     27
#define PWM_FREQ    1000
#define PWM_RES     8
#define TS          100   // Sampling period (ms)

// ── Reference and Control Limits ────────────────────────────
#define REF_MIN     7     // Minimum allowed distance (cm)
#define REF_MAX     35    // Maximum allowed distance (cm)
#define U_MIN      -42.0  // Minimum controller output
#define U_MAX       42.0  // Maximum controller output

// PWM range centered at 176: plant equilibrium point.
// Gravity bias is absorbed here, not in the fuzzy table.
#define PWM_LOW     151
#define PWM_HIGH    199

// ── Control Loop Variables ──────────────────────────────────
float ref    = 21.0;  // Reference setpoint (cm)
float u      = 0.0;   // Controller output
float ek     = 0.0;   // Current error
float ek_1   = 0.0;   // Previous error
float de_val = 0.0;   // Error derivative (for serial output)

// ── Moving Average Filter (5 samples) ───────────────────────
const int NUM_READINGS = 5;
float readings[NUM_READINGS];
int   readingIndex = 0;
float filterSum = 0.0;
bool  bufferFull = false;
float yk_1 = 20.0;  // Last valid reading

// ── Error Membership Functions ──────────────────────────────
// 7 Gaussians from -21 to +21 cm, step = 7
// σ = 7 / 2.828 = 2.48
const float CE[7]    = {-21.0, -14.0, -7.0, 0.0, 7.0, 14.0, 21.0};
const float SIGMA_E  = 2.48;

// ── Error Derivative Membership Functions ───────────────────
// 7 Gaussians from -42 to +42 cm/s, step = 14
// σ = 14 / 2.828 = 4.95
const float CDE[7]   = {-42.0, -28.0, -14.0, 0.0, 14.0, 28.0, 42.0};
const float SIGMA_DE = 4.95;

// ── Defuzzification Table (49 rules) ────────────────────────
// Formula with Bias: clamp(-3*CE[i] - 0.5*CDE[j] + 3.0, -42, 42)
// +3.0 is added across the matrix to adjust for the real equilibrium point.
// Rows = error (NG→PG) | Columns = derivative (NG→PG)
float defuzz[49] = {
  //          NG      NM      NP       Z      PP      PM      PG
  /* NG */  42.0,  42.0,  42.0,  42.0,  42.0,  42.0,  42.0,
  /* NM */  42.0,  42.0,  42.0,  42.0,  38.0,  31.0,  24.0,
  /* NP */  42.0,  38.0,  31.0,  24.0,  17.0,  10.0,   3.0,
  /*  Z */  24.0,  17.0,  10.0,   3.0,  -4.0, -11.0, -18.0,
  /* PP */   3.0,  -4.0, -11.0, -18.0, -25.0, -32.0, -39.0,
  /* PM */ -18.0, -25.0, -32.0, -39.0, -42.0, -42.0, -42.0,
  /* PG */ -39.0, -42.0, -42.0, -42.0, -42.0, -42.0, -42.0
};

unsigned long prevTime = 0;

void  initializeFilter();
float readSensor();
void  readSerialReference();
float fuzzyController(float yk);
float gaussian(float x, float center, float sigma);

void setup() {
  Serial.begin(115200);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);
  initializeFilter();
  Serial.println("Controller: Fuzzy Logic 49 Rules (7x7)");
  prevTime = millis();
}

// Main loop: measure → compute → actuate, every TS ms
void loop() {
  unsigned long now = millis();
  if (now - prevTime < TS) return;
  prevTime = now;

  readSerialReference();
  float yk = readSensor();

  // ── CONTROLLER OUTPUT ─────────────────────────────────
  // u is the fuzzy controller output ∈ [-42, 42]
  u = fuzzyController(yk);
  // ──────────────────────────────────────────────────────

  // Map u ∈ [-42, 42] → PWM ∈ [151, 199]
  float pwm_f = PWM_LOW + (u - U_MIN) / (U_MAX - U_MIN) * (PWM_HIGH - PWM_LOW);
  int pwm_out = constrain((int)pwm_f, 0, 255);
  ledcWrite(PWM_PIN, pwm_out);

  Serial.print("ref:");  Serial.print(ref, 1);
  Serial.print(" yk:");  Serial.print(yk, 2);
  Serial.print(" ek:");  Serial.print(ek, 2);
  Serial.print(" de:");  Serial.print(de_val, 2);
  Serial.print(" u:");   Serial.print(u, 2);
  Serial.print(" TOP:"); Serial.print(REF_MAX);
  Serial.print(" LOW:"); Serial.println(REF_MIN);
}

// Gaussian: produces smooth transitions between fuzzy sets
float gaussian(float x, float center, float sigma) {
  float z = (x - center) / sigma;
  return exp(-0.5 * z * z);
}

// Fuzzy controller: fuzzification → inference → defuzzification
float fuzzyController(float yk) {

  // Error and error derivative
  ek = ref - yk;
  float h  = TS / 1000.0;
  float de = (ek - ek_1) / h;
  de_val   = de;
  ek_1     = ek;

  // Fuzzification: membership degree of ek and de in each set
  float mu_e[7], mu_de[7];
  for (int i = 0; i < 7; i++) {
    mu_e[i]  = gaussian(ek, CE[i],  SIGMA_E);
    mu_de[i] = gaussian(de, CDE[i], SIGMA_DE);
  }

  // Inference + centroid defuzzification (49 rules)
  // uk = Σ(μ_e × μ_de × defuzz) / Σ(μ_e × μ_de)
  float numerator = 0.0, denominator = 0.0;
  for (int i = 0; i < 7; i++) {
    for (int j = 0; j < 7; j++) {
      float weight  = mu_e[i] * mu_de[j];
      numerator    += weight * defuzz[i * 7 + j];
      denominator  += weight;
    }
  }

  if (denominator < 1e-10) return 0.0;

  // ── U(k): final control effort ──────────────────────
  return numerator / denominator;
}

// Initializes the filter and preloads ek_1 to avoid derivative spike at startup
void initializeFilter() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long dur = pulseIn(ECHO_PIN, HIGH, 25000);
  float dist = (dur <= 0) ? 20.0 : dur * 0.0343 / 2.0;

  for (int i = 0; i < NUM_READINGS; i++) readings[i] = dist;
  filterSum = dist * NUM_READINGS;
  readingIndex = 0;
  bufferFull = true;
  yk_1  = dist;
  ek_1  = ref - yk_1;  // Avoids derivative spike on first cycle

  Serial.print("Filter initialized: ");
  Serial.print(dist, 2);
  Serial.println(" cm");
}

// Reads sensor and returns filtered distance (cm)
float readSensor() {
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 25000);
  if (duration <= 0) return yk_1;  // Timeout: return last valid reading

  float distance = duration * 0.0343 / 2.0;

  // Discard readings outside the sensor's physical range
  if (distance < 4.0 || distance > 40.0) return yk_1;

  // Update circular buffer
  filterSum -= readings[readingIndex];
  readings[readingIndex] = distance;
  filterSum += distance;
  readingIndex = (readingIndex + 1) % NUM_READINGS;

  yk_1 = filterSum / NUM_READINGS;
  return yk_1;
}

// Read serial commands:
//   <number>    → new reference in cm  (e.g. "21")
//   def:<i>,<v> → edit defuzz[i] live   (e.g. "def:24,5.0")
void readSerialReference() {
  static String buffer = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      buffer.trim();

      if (buffer.startsWith("def:")) {
        int comma = buffer.indexOf(',');
        if (comma > 0) {
          int   idx = buffer.substring(4, comma).toInt();
          float val = buffer.substring(comma + 1).toFloat();
          if (idx >= 0 && idx < 49) {
            defuzz[idx] = val;
            Serial.print(">>> defuzz["); Serial.print(idx);
            Serial.print("] = ");        Serial.println(val);
          } else {
            Serial.println(">>> Index out of range [0-48]");
          }
        }
      } else {
        float newRef = buffer.toFloat();
        if (newRef >= REF_MIN && newRef <= REF_MAX) {
          ref = newRef;
          Serial.print(">>> New reference: ");
          Serial.print(ref, 1);
          Serial.println(" cm");
        } else if (buffer.length() > 0) {
          Serial.print(">>> Valid range: ");
          Serial.print(REF_MIN); Serial.print("-");
          Serial.print(REF_MAX); Serial.println(" cm");
        }
      }
      buffer = "";
    } else {
      buffer += c;
    }
  }
}
