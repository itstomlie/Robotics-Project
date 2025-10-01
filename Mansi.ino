#include <Servo.h>

// Pin definitions
const int trigPin = 9;     // Ultrasonic trigger pin
const int echoPin = 10;    // Ultrasonic echo pin
const int tempPin = A0;    // Temperature sensor pin (LM35)
const int servoPin = 6;    // Servo motor pin

// Servo object
Servo pumpServo;

// Tank dimensions (in cm)
const int tankHeight = 30; // Height of the reservoir

// Thresholds
const int minLevel = 5;    // Minimum water level (cm from bottom)
const int maxLevel = 25;   // Maximum water level (cm from bottom)
const float minTemp = 15.0; // Minimum temperature for irrigation (°C)
const float maxTemp = 35.0; // Maximum temperature for irrigation (°C)

// Function to measure water level
long getWaterLevel() {
  // Send trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read echo time
  long duration = pulseIn(echoPin, HIGH);

  // Convert to distance (cm)
  long distance = duration * 0.034 / 2;

  // Water level = Tank height - distance from sensor
  long waterLevel = tankHeight - distance;
  return waterLevel;
}

// Function to measure temperature
float getTemperature() {
  int sensorValue = analogRead(tempPin);
  float voltage = sensorValue * (5.0 / 1023.0);
  float tempC = voltage * 100; // LM35: 10mV per °C
  return tempC;
}

void setup() {
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pumpServo.attach(servoPin);
  pumpServo.write(0); // Pump off initially
}

void loop() {
  long waterLevel = getWaterLevel();
  float temperature = getTemperature();

  Serial.print("Water Level: ");
  Serial.print(waterLevel);
  Serial.print(" cm | Temp: ");
  Serial.print(temperature);
  Serial.println(" °C");

  // Control logic
  if (waterLevel <= minLevel && (temperature >= minTemp && temperature <= maxTemp)) {
    // Water too low & temperature suitable → Turn pump ON
    pumpServo.write(90); // Rotate servo to ON position
    Serial.println("Pump ON");
  } 
  else if (waterLevel >= maxLevel) {
    // Tank is full → Turn pump OFF
    pumpServo.write(0); // Rotate servo to OFF position
    Serial.println("Pump OFF - Tank Full");
  } 
  else {
    // Default OFF
    pumpServo.write(0);
    Serial.println("Pump OFF - Conditions not met");
  }

  delay(2000); // Check every 2 seconds
}

