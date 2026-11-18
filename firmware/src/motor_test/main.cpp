// Motor + encoder test on the robot. WHEELS OFF THE GROUND (robot on a box).
//
// Wiring (TB6612FNG):
//   Left  motor (A): PWMA 25, AIN1 26, AIN2 27, AO1 red, AO2 black; encoder yellow 34, white 35
//   Right motor (B): PWMB 14, BIN1 17, BIN2 13, BO1 red, BO2 black; encoder yellow 32, white 39
//   STBY 33. The driver stays in standby (motors off) until a command is given.
//
// Serial 115200, single keys:
//   l / r / b   select left, right or both motors
//   + / -       duty +20 / -20 (range -255..255; negative = reverse)
//   1..9        duty 25%..100% steps (1 = 28, 9 = 255), forward
//   0 or space  stop everything (standby)
//   s           sweep the selected motor(s): finds the lowest duty that turns the wheel, both ways
//   g           turn the selected wheel(s) by ONE_TURN_TICKS at low duty, brake, print the count:
//               the tape mark should end where it started if the counts per turn are right
//   f           fine sweep at the low end (duty 2..30, 0.5 s per step), both directions
//   z           zero the encoder counters
//
// Safety: stops if no key for 15 s, and if a motor gets duty >= 30 but its encoder does not move
// for 0.5 s (stalled wheel, or encoder not connected). The TB6612 is rated 1 A per channel.
#include <Arduino.h>

static const uint8_t STBY = 33;
static const uint32_t PWM_HZ = 20000;   // above hearing
static const uint8_t PWM_BITS = 8;
static const int STALL_DUTY = 30;    // the wheels turn from duty 20, so no movement at 30+ means blocked
static const uint32_t STALL_MS = 500, KEY_TIMEOUT_MS = 15000, PRINT_MS = 250;
static const int32_t ONE_TURN_TICKS = 5000;   // from the hand test (about 4,800-5,200 per turn)
static const int GO_DUTY = 28;

// ---------- encoders (4x quadrature decoding, as in encoder_test) ----------
struct Enc {
  uint8_t pinA, pinB;
  volatile int32_t net = 0;
  volatile uint32_t invalid = 0;
  volatile uint8_t prev = 0;
};
Enc encL{34, 35}, encR{32, 39};
static const int8_t STEP[16] = {0, +1, -1, 2, -1, 0, 2, +1, +1, 2, 0, -1, 2, -1, +1, 0};

void IRAM_ATTR onEdge(Enc* e) {
  uint8_t now = (digitalRead(e->pinA) << 1) | digitalRead(e->pinB);
  int8_t s = STEP[(e->prev << 2) | now];
  if (s == 2) e->invalid = e->invalid + 1;
  else e->net = e->net + s;
  e->prev = now;
}
void IRAM_ATTR isrL() { onEdge(&encL); }
void IRAM_ATTR isrR() { onEdge(&encR); }

void attachEnc(Enc& e, void (*isr)()) {
  pinMode(e.pinA, INPUT);   // 34/35/39 have no internal pull-ups
  pinMode(e.pinB, INPUT);
  e.prev = (digitalRead(e.pinA) << 1) | digitalRead(e.pinB);
  attachInterrupt(digitalPinToInterrupt(e.pinA), isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(e.pinB), isr, CHANGE);
}

int32_t ticks(Enc& e) { noInterrupts(); int32_t v = e.net; interrupts(); return v; }
uint32_t invalid(Enc& e) { noInterrupts(); uint32_t v = e.invalid; interrupts(); return v; }
void zero(Enc& e) { noInterrupts(); e.net = 0; e.invalid = 0; interrupts(); }

// ---------- motors ----------
struct Motor {
  const char* name;
  uint8_t pwm, in1, in2;
  Enc* enc;
  int duty = 0;
  int32_t lastTicks = 0;
  uint32_t lastMove = 0;
};
Motor left{"left", 25, 26, 27, &encL}, right{"right", 14, 17, 13, &encR};

void apply(Motor& m, int duty) {
  m.duty = constrain(duty, -255, 255);
  if (m.duty > 0)      { digitalWrite(m.in1, HIGH); digitalWrite(m.in2, LOW); }
  else if (m.duty < 0) { digitalWrite(m.in1, LOW);  digitalWrite(m.in2, HIGH); }
  else                 { digitalWrite(m.in1, LOW);  digitalWrite(m.in2, LOW); }   // coast
  ledcWrite(m.pwm, abs(m.duty));
  m.lastTicks = ticks(*m.enc);
  m.lastMove = millis();
  if (m.duty != 0) digitalWrite(STBY, HIGH);
}

void stopAll(const char* why) {
  apply(left, 0);
  apply(right, 0);
  digitalWrite(STBY, LOW);
  Serial.printf("STOP: %s\n", why);
}

// TB6612 short brake: IN1 = IN2 = HIGH stops the wheel quickly (less coasting than standby)
void brake(Motor& m) {
  digitalWrite(m.in1, HIGH); digitalWrite(m.in2, HIGH);
  ledcWrite(m.pwm, 255);
  m.duty = 0;
}

void setupMotor(Motor& m) {
  pinMode(m.in1, OUTPUT); digitalWrite(m.in1, LOW);
  pinMode(m.in2, OUTPUT); digitalWrite(m.in2, LOW);
  ledcAttach(m.pwm, PWM_HZ, PWM_BITS);
  ledcWrite(m.pwm, 0);
}

// true if this motor is stalled (driven hard, encoder not moving)
bool stalled(Motor& m) {
  int32_t t = ticks(*m.enc);
  if (t != m.lastTicks || abs(m.duty) < STALL_DUTY) { m.lastTicks = t; m.lastMove = millis(); return false; }
  return millis() - m.lastMove > STALL_MS;
}

// ---------- sweep ----------
bool keyPressed() { if (Serial.available()) { while (Serial.available()) Serial.read(); return true; } return false; }

// Ramp one direction; returns the first duty where the wheel turns (>= 20 ticks/s), or 0 if none
int sweepDir(Motor& m, int sign) {
  Serial.printf("  %s %s: duty  ticks/s\n", m.name, sign > 0 ? "forward" : "reverse");
  int start = 0;
  for (int d = 20; d <= 255; d += 10) {
    apply(m, sign * d);
    delay(300);                                  // let it speed up
    int32_t t0 = ticks(*m.enc);
    delay(200);
    float rate = (ticks(*m.enc) - t0) / 0.2f;
    Serial.printf("        %4d  %8.0f\n", sign * d, rate);
    if (!start && fabsf(rate) >= 20) start = d;
    if (keyPressed()) { stopAll("sweep aborted"); return start; }
    if (d >= STALL_DUTY && fabsf(rate) < 1) { stopAll("no encoder movement at high duty"); return start; }
    if (start && d >= start + 60) break;         // enough to see the trend
  }
  apply(m, 0);
  delay(400);
  return start;
}

void sweep(Motor& m) {
  Serial.printf("sweep %s (any key aborts)\n", m.name);
  zero(*m.enc);
  int f = sweepDir(m, +1);
  int32_t fwdTicks = ticks(*m.enc);
  int r = sweepDir(m, -1);
  Serial.printf("RESULT %s: starts at duty %d forward, %d reverse; forward duty gave %s ticks; "
                "invalid steps %lu\n", m.name, f, r, fwdTicks >= 0 ? "POSITIVE" : "NEGATIVE",
                (unsigned long)invalid(*m.enc));
}

// Turn one wheel by ONE_TURN_TICKS encoder counts, then brake and report
void goOneTurn(Motor& m) {
  zero(*m.enc);
  Serial.printf("%s: one turn = %ld counts at duty %d; watch the tape mark\n", m.name,
                (long)ONE_TURN_TICKS, GO_DUTY);
  delay(1500);
  apply(m, GO_DUTY);
  uint32_t t0 = millis();
  while (abs(ticks(*m.enc)) < ONE_TURN_TICKS) {
    if (millis() - t0 > 5000) { stopAll("5 s without reaching one turn"); return; }
    if (keyPressed()) { stopAll("aborted"); return; }
  }
  int32_t atStop = ticks(*m.enc);
  brake(m);
  delay(500);
  int32_t final = ticks(*m.enc);
  apply(m, 0);
  digitalWrite(STBY, LOW);
  Serial.printf("RESULT %s: braked at %ld, stopped at %ld counts (%.2f turns if 5000/turn), "
                "took %lu ms\n", m.name, (long)atStop, (long)final, abs(final) / 5000.0f,
                (unsigned long)(millis() - t0 - 500));
}

// Fine low-end sweep: lowest duty that keeps the wheel turning (>= 500 counts/s, about 0.1 turn/s)
int fineDir(Motor& m, int sign) {
  Serial.printf("  %s %s: duty  counts/s  turns/s\n", m.name, sign > 0 ? "forward" : "reverse");
  int start = 0;
  for (int d = 2; d <= 30; d += 2) {
    apply(m, sign * d);
    delay(400);
    int32_t t0 = ticks(*m.enc);
    delay(500);
    float rate = (ticks(*m.enc) - t0) / 0.5f;
    Serial.printf("        %4d  %8.0f  %6.2f\n", sign * d, rate, fabsf(rate) / 5000.0f);
    if (!start && fabsf(rate) >= 500) start = d;
    if (keyPressed()) { stopAll("fine sweep aborted"); return start; }
  }
  apply(m, 0);
  delay(600);
  return start;
}

// ---------- main ----------
char sel = 'b';
uint32_t lastKey = 0;

void setDuty(int d) {
  if (sel != 'r') apply(left, d);
  if (sel != 'l') apply(right, d);
}

int selectedDuty() { return sel == 'r' ? right.duty : left.duty; }

void setup() {
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);          // motors off before anything else is configured
  setupMotor(left);
  setupMotor(right);
  Serial.begin(115200);
  delay(300);
  attachEnc(encL, isrL);
  attachEnc(encR, isrR);
  Serial.println("\nMotor test. WHEELS OFF THE GROUND. Keys: l r b select, + - duty, 1-9 speed, "
                 "0/space stop, s sweep, f fine sweep, g one turn, z zero");
  Serial.println("selected: both");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    lastKey = millis();
    switch (c) {
      case 'l': case 'r': case 'b':
        sel = c;
        Serial.printf("selected: %s\n", c == 'l' ? "left" : c == 'r' ? "right" : "both");
        break;
      case '+': setDuty(selectedDuty() + 20); break;
      case '-': setDuty(selectedDuty() - 20); break;
      case '0': case ' ': stopAll("key"); break;
      case 'z': zero(encL); zero(encR); Serial.println("zeroed"); break;
      case 'g':
        stopAll("before one-turn test");
        if (sel != 'r') goOneTurn(left);
        if (sel != 'l') { if (sel == 'b') delay(1000); goOneTurn(right); }
        lastKey = millis();
        break;
      case 'f':
        stopAll("before fine sweep");
        if (sel != 'r') { int a = fineDir(left, +1), b = fineDir(left, -1);
          Serial.printf("RESULT left: turns from duty %d forward, %d reverse\n", a, b); }
        if (sel != 'l') { int a = fineDir(right, +1), b = fineDir(right, -1);
          Serial.printf("RESULT right: turns from duty %d forward, %d reverse\n", a, b); }
        stopAll("fine sweep done");
        lastKey = millis();
        break;
      case 's':
        stopAll("before sweep");
        if (sel != 'r') sweep(left);
        if (sel != 'l') sweep(right);
        stopAll("sweep done");
        lastKey = millis();
        break;
      default:
        if (c >= '1' && c <= '9') setDuty(28 + (c - '1') * (255 - 28) / 8);
    }
  }

  bool running = left.duty != 0 || right.duty != 0;
  if (running && millis() - lastKey > KEY_TIMEOUT_MS) stopAll("no key for 15 s");
  if (stalled(left)) stopAll("left wheel stalled or its encoder is not moving");
  if (stalled(right)) stopAll("right wheel stalled or its encoder is not moving");

  static uint32_t lastPrint = 0;
  static int32_t pl = 0, pr = 0;
  if (running && millis() - lastPrint >= PRINT_MS) {
    float dt = (millis() - lastPrint) / 1000.0f;
    lastPrint = millis();
    int32_t tl = ticks(encL), tr = ticks(encR);
    Serial.printf("duty L %4d R %4d | ticks L %8ld R %8ld | ticks/s L %6.0f R %6.0f | invalid L %lu R %lu\n",
                  left.duty, right.duty, (long)tl, (long)tr, (tl - pl) / dt, (tr - pr) / dt,
                  (unsigned long)invalid(encL), (unsigned long)invalid(encR));
    pl = tl; pr = tr;
  }
}
