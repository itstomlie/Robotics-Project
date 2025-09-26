#include <Servo.h>

// -------- Pins --------
const int PIN_LDR = A1;   // LDR + 10k divider
const int PIN_LM35 = A0;  // LM35 temperature sensor
const int PIN_AIRQ = A2;  // 1k potentiometer (air quality)
const int PIN_SERVO = 9;  // Vent flap servo
const int PIN_LED = 4;    // Status LED (vent on)

// -------- Thresholds --------
const int LIGHT_ON = 15;      // Vent opens when light (%) > 15
const float TEMP_ON = 35.0;   // Vent opens when temperature ≥ 35 °C
const int AIRQ_GOODMIN = 80;  // If airQ < 80% => bad air => open

// -------- State --------
Servo vent;
bool ventOn = false;
int currentAngle = 0;
unsigned long lastUI = 0;

// ===== Utilities =====
int smoothAnalog(int pin) {
  long acc = 0;
  for (int i = 0; i < 8; i++) acc += analogRead(pin);
  return acc / 8;  // 8-sample average
}

int readPercentSmooth(int pin) {
  int raw = smoothAnalog(pin);         // 0..1023
  int pct = (raw * 100 + 511) / 1023;  // 0..100 (rounded)
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return pct;
}

float readTempCSmooth() {
  long acc = 0;
  for (int i = 0; i < 8; i++) acc += analogRead(PIN_LM35);
  float v = ((acc / 8.0f) * 5.0f) / 1023.0f;  // ADC -> volts (5V ref)
  return v * 100.0f;                          // LM35: 10mV/°C
}

void safeServoWrite(int targetAngle) {
  targetAngle = constrain(targetAngle, 0, 180);
  if (abs(targetAngle - currentAngle) >= 3) {  // deadband to avoid twitch
    vent.write(targetAngle);
    currentAngle = targetAngle;
  }
}

void attachIfNeeded() {
  if (!vent.attached()) vent.attach(PIN_SERVO);
}

void detachIfAttached() {
  if (vent.attached()) vent.detach();
}

void setVent(bool on) {
  if (ventOn == on) return;  // no change
  ventOn = on;
  digitalWrite(PIN_LED, on ? HIGH : LOW);

  attachIfNeeded();
  // SERVO LOGIC: 
  // - On startup: servo resets to 0° (closed)
  // - If ANY condition met: servo opens to 180° (fully open)
  // - If NO conditions met: servo closes to 0° (fully closed)
  safeServoWrite(on ? 180 : 0);  // On=180° (open), Off=0° (closed)
  
  if (!on) {
    delay(30);           // brief settle
    detachIfAttached();  // reduce idle jitter
  }
}

// median helper
static int median5(int a, int b, int c, int d, int e) {
  int v[5] = { a, b, c, d, e };
  for (int i = 1; i < 5; i++) {
    int x = v[i], j = i - 1;
    while (j >= 0 && v[j] > x) {
      v[j + 1] = v[j];
      j--;
    }
    v[j + 1] = x;
  }
  return v[2];
}

int readAirQPercentQuiet() {
  // Do NOT detach when ventOn; keep pulses continuous
  // Throwaway read after mux switch, then median:
  analogRead(PIN_AIRQ);
  delayMicroseconds(50);
  int m = median5(analogRead(PIN_AIRQ), analogRead(PIN_AIRQ),
                  analogRead(PIN_AIRQ), analogRead(PIN_AIRQ), analogRead(PIN_AIRQ));
  long pct = (long)m * 100 / 1023;
  if (pct < 0) pct = 0; if (pct > 100) pct = 100;
  return (int)pct;
}


// ===== Setup/Loop =====
void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  // RESET SERVO TO BASE 0 DEGREES ON STARTUP
  attachIfNeeded();
  vent.write(0);      // Start at 0° (fully closed)
  currentAngle = 0;   // Track current position

  Serial.begin(9600);
  delay(300);
  Serial.println(F("Vent Control: light + temperature + air quality"));
  Serial.println(F("Servo starts at 0° (closed), opens to 180° when triggered"));
  Serial.println(F("T[C], L[%], AQ[%], Vent"));
}

void loop() {
  int lightP = readPercentSmooth(PIN_LDR);
  float tempC = readTempCSmooth();
  int airQP = 100 - readAirQPercentQuiet();  // Inverted so lower = worse air

  // SERVO CONTROL LOGIC:
  // Open vent (180°) if ANY trigger is active:
  // - Light > 15%
  // - Temperature ≥ 35°C  
  // - Air quality < 80% (bad air)
  // Otherwise close vent (0°)
  bool needVent = (lightP > LIGHT_ON) || (tempC >= TEMP_ON) || (airQP < AIRQ_GOODMIN);
  setVent(needVent);

  // Status output (every ~1 s)
  unsigned long now = millis();
  if (now - lastUI >= 1000UL) {
    Serial.print(F("T="));
    Serial.print(tempC, 1);
    Serial.print(F("C | L="));
    Serial.print(lightP);
    Serial.print(F("% | AQ="));
    Serial.print(airQP);
    Serial.print(F("% | Vent="));
    Serial.println(ventOn ? F("On") : F("Off"));
    lastUI = now;
  }
}
