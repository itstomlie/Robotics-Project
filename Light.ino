#include <Wire.h>
#include <RTClib.h>

RTC_DS1307 rtc;

// Pin mapping
const int PIN_R = 6;   // Red
const int PIN_G = 3;   // Green
const int PIN_B = 5;   // Blue

// MPU6050 at 0x69
const uint8_t MPU_ADDR = 0x69;

bool lightOn = false;

// ----------- Intensity Settings -----------
// Lower values = brighter (common anode logic)
int redLevelVeg = 180;   // ~30% brightness (higher value = dimmer)
int blueLevelVeg = 50;   // ~80% brightness

int redLevelFlower = 50; // ~80% brightness
int blueLevelFlower = 180; // ~30% brightness

// ----------- MPU6050 helpers -----------
int16_t read16(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 2);
  while (Wire.available() < 2) {}
  int16_t hi = Wire.read();
  int16_t lo = Wire.read();
  return (hi << 8) | lo;
}

float readAccelG() {
  int16_t ax = read16(0x3B);
  int16_t ay = read16(0x3D);
  int16_t az = read16(0x3F);
  float gx = ax / 16384.0f;
  float gy = ay / 16384.0f;
  float gz = az / 16384.0f;
  float vz = gz - 1.0f;
  return sqrt(gx*gx + gy*gy + vz*vz);
}

// ----------- LED control -----------
// Mode: 0 = Vegetative, 1 = Flowering
void setRGB(bool on, int mode = 0) {
  if (on) {
    if (mode == 0) { // Vegetative stage
      analogWrite(PIN_R, redLevelVeg);
      analogWrite(PIN_B, blueLevelVeg);
    } else { // Flowering stage
      analogWrite(PIN_R, redLevelFlower);
      analogWrite(PIN_B, blueLevelFlower);
    }
    analogWrite(PIN_G, 255); // Green OFF
  } else {
    analogWrite(PIN_R, 255);
    analogWrite(PIN_B, 255);
    analogWrite(PIN_G, 255);
  }
}

// ----------- MPU setup -----------
void setupMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();
}

// ----------- Setup -----------
void setup() {
  pinMode(PIN_R, OUTPUT);
  pinMode(PIN_G, OUTPUT);
  pinMode(PIN_B, OUTPUT);

  analogWrite(PIN_R, 255);
  analogWrite(PIN_G, 255);
  analogWrite(PIN_B, 255);

  Serial.begin(9600);
  Wire.begin();

  if (!rtc.begin()) {
    Serial.println("ERROR: RTC not found.");
    while (1);
  }
  if (!rtc.isrunning()) {
    Serial.println("RTC not running; setting to compile time.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  setupMPU();

  Serial.println("RTC + MPU6050 + Grow Light demo with intensity control.");
}

// ----------- Main loop -----------
void loop() {
  DateTime now = rtc.now();

  // Switch light ON during even minutes
  bool shouldOn = (now.minute() % 2 == 0);

  // Switch mode based on hour (veg in morning, flower in evening)
  int mode = (now.hour() < 12) ? 0 : 1;

  if (shouldOn != lightOn) {
    lightOn = shouldOn;
    setRGB(lightOn, mode);
  }

  // Read vibration
  float vib = readAccelG();

  // Print clean status
  char buf[20];
  sprintf(buf, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());

  Serial.print("TIME: ");
  Serial.print(buf);
  Serial.print(" | LIGHT: ");
  Serial.print(lightOn ? (mode == 0 ? "ON (VEG)" : "ON (FLOWER)") : "OFF");
  Serial.print(" | VIB: ");
  Serial.print(vib, 2);
  if (vib > 0.20) {
    Serial.print("  ALERT: High vibration!");
  }
  Serial.println();

  delay(1000);
}
