// Encoder bench test. Motors do NOT need power: the encoders run from the ESP32's 3.3 V.
//
// Wiring per the motor's label (blue VCC -> 3.3 V, green GND -> GND; never 5 V, the ESP32 inputs are 3.3 V):
//   Motor A (left):  ENA (yellow) -> GPIO 34, ENB (white) -> GPIO 35
//   Motor B (right): ENA (yellow) -> GPIO 32, ENB (white) -> GPIO 39
//   Red (M+) and black (M-) are the motor's power wires: leave them unconnected for this test.
//
// What it checks:
//   1. Are the two channels true 90-degree quadrature? Every edge must change exactly one channel.
//      "invalid" counts steps where both channels changed at once; it must stay at 0.
//   2. Ticks per wheel revolution: turn a wheel by hand exactly N revolutions and read "net".
//      ticks_per_rev = net / N. This is with 4x decoding (every edge of both channels).
//
// Serial (115200): 'z' zeroes the counters.
#include <Arduino.h>

struct Enc {
  uint8_t pinA, pinB;
  volatile int32_t net = 0;
  volatile uint32_t fwd = 0, back = 0, invalid = 0;
  volatile uint32_t edgesA = 0, edgesB = 0;   // per-wire edge counts: healthy wires give about equal numbers
  volatile uint8_t prev = 0;
};

Enc encA{34, 35};
Enc encB{32, 39};

// Gray-code step table indexed by (prev << 2) | now: +1, -1, 0 (no change), 2 (invalid double step)
static const int8_t STEP[16] = {0, +1, -1, 2, -1, 0, 2, +1, +1, 2, 0, -1, 2, -1, +1, 0};

void IRAM_ATTR onEdge(Enc* e) {
  uint8_t now = (digitalRead(e->pinA) << 1) | digitalRead(e->pinB);
  int8_t s = STEP[(e->prev << 2) | now];
  if (s == 2) e->invalid = e->invalid + 1;
  else if (s > 0) { e->net = e->net + 1; e->fwd = e->fwd + 1; }
  else if (s < 0) { e->net = e->net - 1; e->back = e->back + 1; }
  e->prev = now;
}
void IRAM_ATTR isrAa() { encA.edgesA = encA.edgesA + 1; onEdge(&encA); }
void IRAM_ATTR isrAb() { encA.edgesB = encA.edgesB + 1; onEdge(&encA); }
void IRAM_ATTR isrBa() { encB.edgesA = encB.edgesA + 1; onEdge(&encB); }
void IRAM_ATTR isrBb() { encB.edgesB = encB.edgesB + 1; onEdge(&encB); }

void attach(Enc& e, void (*ia)(), void (*ib)()) {
  pinMode(e.pinA, INPUT);   // GPIO 34-39 are input-only with no internal pull-ups
  pinMode(e.pinB, INPUT);
  e.prev = (digitalRead(e.pinA) << 1) | digitalRead(e.pinB);
  attachInterrupt(digitalPinToInterrupt(e.pinA), ia, CHANGE);
  attachInterrupt(digitalPinToInterrupt(e.pinB), ib, CHANGE);
}

void snapshot(Enc& e, int32_t& net, uint32_t& fwd, uint32_t& back, uint32_t& inv, uint32_t& ea, uint32_t& eb) {
  noInterrupts();
  net = e.net; fwd = e.fwd; back = e.back; inv = e.invalid; ea = e.edgesA; eb = e.edgesB;
  interrupts();
}

void zero(Enc& e) {
  noInterrupts();
  e.net = 0; e.fwd = 0; e.back = 0; e.invalid = 0; e.edgesA = 0; e.edgesB = 0;
  interrupts();
}

void setup() {
  Serial.begin(115200);
  delay(300);
  attach(encA, isrAa, isrAb);
  attach(encB, isrBa, isrBb);
  Serial.println("\nEncoder test. Turn a wheel by hand. 'z' zeroes the counters.");
  Serial.println("A = left (yellow 34, white 35), B = right (yellow 32, white 39)");
  Serial.println("A: net fwd back invalid yellow white | B: net fwd back invalid yellow white");
}

void loop() {
  if (Serial.available() && Serial.read() == 'z') { zero(encA); zero(encB); Serial.println("zeroed"); }
  static uint32_t last = 0;
  if (millis() - last >= 500) {
    last = millis();
    int32_t na, nb; uint32_t fa, ba, ia, fb, bb, ib, ya, wa, yb, wb;
    snapshot(encA, na, fa, ba, ia, ya, wa);
    snapshot(encB, nb, fb, bb, ib, yb, wb);
    Serial.printf("A: %7ld %6lu %6lu %3lu %6lu %6lu | B: %7ld %6lu %6lu %3lu %6lu %6lu\n",
                  (long)na, (unsigned long)fa, (unsigned long)ba, (unsigned long)ia,
                  (unsigned long)ya, (unsigned long)wa,
                  (long)nb, (unsigned long)fb, (unsigned long)bb, (unsigned long)ib,
                  (unsigned long)yb, (unsigned long)wb);
  }
}
