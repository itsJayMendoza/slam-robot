// slam-robot base firmware: two wheel-speed loops on the ESP32, commanded over USB serial.
//
// Wiring: left motor  PWMA 25, AIN1 26, AIN2 27; encoder yellow 34, white 35
//         right motor PWMB 14, BIN1 17, BIN2 13; encoder yellow 32, white 39;  STBY 33
// Measured on the robot (motor_test): about 5,000 counts per wheel turn, left motor direction and
// right encoder sign reversed (corrected below, so forward is positive on both wheels).
//
// Protocol: ASCII lines at 115200 baud, '\n' terminated. Speeds in encoder counts per second.
//   v <l> <r>          set wheel speeds (clamped to +/-MAX_CPS); must repeat within 500 ms
//   pwm <l> <r>        open loop duty, -DUTY_MAX..DUTY_MAX of 2047 (tests only); same timeout
//   drive <l> <r> <ms> like v but runs for <ms> (max 10000) without repeating, for manual tests
//   e                  read encoders
//   x                  stop and clear a fault
//   k [kp ki kff kofs] print or set the speed-loop gains
//   ?                  status (targets, measured speeds, duties, fault)
//   ver                firmware version
// Replies: v, pwm, drive and e answer "e <left counts> <right counts> <left cps> <right cps> <fault>";
// x and k answer "ok ..."; errors "err ..."; lines starting with '#' are comments.
// Fault bits (latched, motors off until x): 1/2 left/right stalled, 4/8 left/right turning the
// wrong way, 16/32 left/right overspeed (encoder sign or wiring wrong).
#include <Arduino.h>
#include "driver/pulse_cnt.h"

static const char* VERSION = "slam-robot-base 0.1";

// ---------- limits and timing ----------
static const uint32_t LOOP_US = 20000;            // 50 Hz speed loop
static const uint32_t CMD_TIMEOUT_MS = 500;       // stop if the host goes quiet
static const uint32_t DRIVE_MAX_MS = 10000;
static const float MAX_CPS = 7000;                // about 0.32 m/s (5,000 counts per 0.226 m turn)
static const float ACCEL_CPS2 = 15000;            // target ramp, about 0.7 m/s^2
static const int PWM_BITS = 11, PWM_MAX = 2047;   // 20 kHz with 2,048 steps
static const uint32_t PWM_HZ = 20000;
static const int DUTY_MAX = 500;                  // about 24 %; the robot needs far less
static const int STALL_DUTY = 250;                // stall: this much duty but under STALL_CPS ...
static const float STALL_CPS = 300;
static const uint32_t STALL_MS = 1000;            // ... for this long
static const float WRONG_DIR_CPS = 1000;          // turning against the target this fast ...
static const uint32_t WRONG_DIR_MS = 300;         // ... for this long
static const float OVERSPEED_CPS = MAX_CPS * 1.6f;

// Speed loop: duty = kff * target + kofs * sign(target) + kp * error + ki * integral(error).
// Defaults from the wheels-in-the-air sweep (duty8 = 4.5 + cps / 1100, i.e. duty11 = 36 + cps / 137.5);
// to be tuned on the floor with "k".
static float KP = 0.01f, KI = 0.02f, KFF = 1.0f / 137.5f, KOFS = 36.0f;
static const float INTEG_DUTY_MAX = 200;          // anti-windup: the integral term stays within this

static const uint8_t STBY = 33;

// ---------- encoders on the pulse-counter hardware (4x quadrature, glitch filtered) ----------
struct Encoder {
  pcnt_unit_handle_t unit = nullptr;
  int sign;
};

bool setupEncoder(Encoder& e, int pinA, int pinB) {
  pcnt_unit_config_t uc = {};
  uc.low_limit = -30000;
  uc.high_limit = 30000;
  uc.flags.accum_count = 1;                        // keep counting past the 16-bit hardware range
  if (pcnt_new_unit(&uc, &e.unit) != ESP_OK) return false;
  pcnt_glitch_filter_config_t fc = {};
  fc.max_glitch_ns = 1000;
  pcnt_unit_set_glitch_filter(e.unit, &fc);
  pcnt_chan_config_t ca = {}, cb = {};
  ca.edge_gpio_num = pinA; ca.level_gpio_num = pinB;
  cb.edge_gpio_num = pinB; cb.level_gpio_num = pinA;
  pcnt_channel_handle_t chA, chB;
  if (pcnt_new_channel(e.unit, &ca, &chA) != ESP_OK) return false;
  if (pcnt_new_channel(e.unit, &cb, &chB) != ESP_OK) return false;
  // Same direction convention as encoder_test (checked on the robot: these actions give the same
  // sign as the test sketch; the opposite pair counted every wheel backwards)
  pcnt_channel_set_edge_action(chA, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
  pcnt_channel_set_level_action(chA, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
  pcnt_channel_set_edge_action(chB, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
  pcnt_channel_set_level_action(chB, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
  pcnt_unit_add_watch_point(e.unit, uc.low_limit);
  pcnt_unit_add_watch_point(e.unit, uc.high_limit);
  pcnt_unit_enable(e.unit);
  pcnt_unit_clear_count(e.unit);
  pcnt_unit_start(e.unit);
  return true;
}

int32_t readEncoder(Encoder& e) {
  int v = 0;
  pcnt_unit_get_count(e.unit, &v);
  return e.sign * v;
}

// ---------- wheels ----------
struct Wheel {
  const char* name;
  uint8_t pwm, in1, in2;
  int motorSign;
  Encoder enc;
  float cmd = 0, target = 0;          // commanded and ramped target, counts/s
  float meas = 0;                     // measured speed, counts/s (filtered)
  float integ = 0;
  int duty = 0;
  int32_t lastCount = 0;
  uint32_t stallSince = 0, wrongSince = 0, overSince = 0;
};

Wheel left{"left", 25, 26, 27, -1, {nullptr, +1}};
Wheel right{"right", 14, 17, 13, +1, {nullptr, -1}};

enum Mode { IDLE, SPEED, OPEN_LOOP };
Mode mode = IDLE;
uint32_t deadline = 0;                // millis() after which commands time out
int fault = 0;

void writeMotor(Wheel& w, int duty) {
  w.duty = constrain(duty, -DUTY_MAX, DUTY_MAX);
  int d = w.motorSign * w.duty;
  if (d > 0)      { digitalWrite(w.in1, HIGH); digitalWrite(w.in2, LOW); }
  else if (d < 0) { digitalWrite(w.in1, LOW);  digitalWrite(w.in2, HIGH); }
  else            { digitalWrite(w.in1, HIGH); digitalWrite(w.in2, HIGH); }   // short brake
  ledcWrite(w.pwm, d == 0 ? PWM_MAX : abs(d));
}

void resetWheel(Wheel& w) {
  w.cmd = w.target = w.integ = 0;
  w.stallSince = w.wrongSince = w.overSince = 0;
  writeMotor(w, 0);
}

void stopAll() {
  mode = IDLE;
  resetWheel(left);
  resetWheel(right);
  digitalWrite(STBY, LOW);
}

void setupWheel(Wheel& w) {
  pinMode(w.in1, OUTPUT); digitalWrite(w.in1, LOW);
  pinMode(w.in2, OUTPUT); digitalWrite(w.in2, LOW);
  ledcAttach(w.pwm, PWM_HZ, PWM_BITS);
  ledcWrite(w.pwm, 0);
}

float sgn(float x) { return (x > 0) - (x < 0); }

// Returns fault bits raised by this wheel (bit 0 = stall, 1 = wrong way, 2 = overspeed)
int checkFaults(Wheel& w, uint32_t now) {
  int f = 0;
  bool stalled = abs(w.duty) >= STALL_DUTY && fabsf(w.meas) < STALL_CPS;
  bool wrong = w.target != 0 && w.meas * w.target < 0 && fabsf(w.meas) > WRONG_DIR_CPS;
  bool over = fabsf(w.meas) > OVERSPEED_CPS;
  w.stallSince = stalled ? (w.stallSince ? w.stallSince : now) : 0;
  w.wrongSince = wrong ? (w.wrongSince ? w.wrongSince : now) : 0;
  w.overSince = over ? (w.overSince ? w.overSince : now) : 0;
  if (w.stallSince && now - w.stallSince > STALL_MS) f |= 1;
  if (w.wrongSince && now - w.wrongSince > WRONG_DIR_MS) f |= 2;
  if (w.overSince && now - w.overSince > 100) f |= 4;
  return f;
}

void control(Wheel& w, float dt) {
  // ramp the target towards the command
  float step = ACCEL_CPS2 * dt;
  w.target += constrain(w.cmd - w.target, -step, step);
  if (mode != SPEED) return;
  if (w.target == 0 && w.cmd == 0) {
    w.integ = 0;
    writeMotor(w, 0);
    return;
  }
  float err = w.target - w.meas;
  w.integ += err * dt;
  float iMax = INTEG_DUTY_MAX / max(KI, 1e-6f);
  w.integ = constrain(w.integ, -iMax, iMax);
  float duty = KFF * w.target + KOFS * sgn(w.target) + KP * err + KI * w.integ;
  writeMotor(w, lroundf(duty));
}

void measure(Wheel& w, float dt) {
  int32_t c = readEncoder(w.enc);
  float raw = (c - w.lastCount) / dt;
  w.lastCount = c;
  w.meas = 0.5f * w.meas + 0.5f * raw;
}

// ---------- serial protocol ----------
void replyEncoders() {
  Serial.printf("e %ld %ld %.0f %.0f %d\n", (long)readEncoder(left.enc), (long)readEncoder(right.enc),
                left.meas, right.meas, fault);
}

void arm(uint32_t ms) {
  deadline = millis() + ms;
  digitalWrite(STBY, HIGH);
}

void handle(char* line) {
  char* argv[5];
  int argc = 0;
  for (char* t = strtok(line, " \t"); t && argc < 5; t = strtok(nullptr, " \t")) argv[argc++] = t;
  if (argc == 0) return;
  String c = argv[0];
  if (fault && (c == "v" || c == "pwm" || c == "drive")) { replyEncoders(); return; }   // until x
  if (c == "v" && argc == 3) {
    left.cmd = constrain(atof(argv[1]), -MAX_CPS, MAX_CPS);
    right.cmd = constrain(atof(argv[2]), -MAX_CPS, MAX_CPS);
    if (mode != SPEED) { left.integ = right.integ = 0; }
    mode = SPEED;
    arm(CMD_TIMEOUT_MS);
    replyEncoders();
  } else if (c == "drive" && argc == 4) {
    left.cmd = constrain(atof(argv[1]), -MAX_CPS, MAX_CPS);
    right.cmd = constrain(atof(argv[2]), -MAX_CPS, MAX_CPS);
    mode = SPEED;
    arm(min((uint32_t)atol(argv[3]), DRIVE_MAX_MS));
    replyEncoders();
  } else if (c == "pwm" && argc == 3) {
    mode = OPEN_LOOP;
    arm(CMD_TIMEOUT_MS);
    writeMotor(left, atoi(argv[1]));
    writeMotor(right, atoi(argv[2]));
    replyEncoders();
  } else if (c == "e") {
    replyEncoders();
  } else if (c == "x") {
    stopAll();
    fault = 0;
    Serial.println("ok stopped");
  } else if (c == "k") {
    if (argc == 5) { KP = atof(argv[1]); KI = atof(argv[2]); KFF = atof(argv[3]); KOFS = atof(argv[4]); }
    else if (argc != 1) { Serial.println("err usage: k [kp ki kff kofs]"); return; }
    Serial.printf("ok k %.5f %.5f %.6f %.2f\n", KP, KI, KFF, KOFS);
  } else if (c == "?") {
    Serial.printf("s mode %d target %.0f %.0f meas %.0f %.0f duty %d %d fault %d\n", (int)mode,
                  left.target, right.target, left.meas, right.meas, left.duty, right.duty, fault);
  } else if (c == "ver") {
    Serial.printf("ver %s\n", VERSION);
  } else {
    Serial.println("err unknown command");
  }
}

void readSerial() {
  static char buf[80];
  static size_t n = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') { buf[n] = 0; handle(buf); n = 0; }
    else if (n < sizeof(buf) - 1) buf[n++] = ch;
    else n = 0;                                  // overlong line: drop it
  }
}

// ---------- main ----------
void setup() {
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);                       // motors off before anything else
  setupWheel(left);
  setupWheel(right);
  Serial.begin(115200);
  bool ok = setupEncoder(left.enc, 34, 35) && setupEncoder(right.enc, 32, 39);
  stopAll();
  Serial.printf("# %s ready%s\n", VERSION, ok ? "" : " (ENCODER SETUP FAILED)");
}

void loop() {
  readSerial();
  static uint32_t last = micros();
  uint32_t nowUs = micros();
  if (nowUs - last < LOOP_US) return;
  float dt = (nowUs - last) * 1e-6f;
  last = nowUs;
  uint32_t now = millis();

  measure(left, dt);
  measure(right, dt);
  if (mode != IDLE && (int32_t)(now - deadline) > 0) stopAll();   // host went quiet
  control(left, dt);
  control(right, dt);

  if (mode != IDLE) {
    int fl = checkFaults(left, now), fr = checkFaults(right, now);
    // protocol bits: stall 1/2, wrong way 4/8, overspeed 16/32 (left/right)
    int bits = ((fl & 1) ? 1 : 0) | ((fr & 1) ? 2 : 0) | ((fl & 2) ? 4 : 0) | ((fr & 2) ? 8 : 0) |
               ((fl & 4) ? 16 : 0) | ((fr & 4) ? 32 : 0);
    if (bits) { fault |= bits; stopAll(); Serial.printf("# fault %d\n", fault); }
  }
}
