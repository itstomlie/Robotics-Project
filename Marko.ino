/*
  assignment_project.ino
  Author: Marko Koiduste (22563501)
  Institution: Curtin University

  Purpose:  Part of smart-farming simulation and exploration of automation opportunities.
            This code is part of sprinkler system automation based on moisture level. In 
            real life situation the input would come as voltage from moisture sensor (in 
            our example we use 1K Potentiometer to simulate the input), the actuator would 
            be electric valve. 

  Changes:  1) Milestone 1 - initial functionality for valve operation, data and status output
            to LCD. (-30/09/2025) 

  TODO:     - Slave / Master functionality
            - Task / Command system to schedule sprinklers & submit to master commands
*/

#include <LiquidCrystal.h>
#include <Servo.h>

// -- CONFIG -- //
namespace CFG {
  // LCD pins
  constexpr int LCD_RS = 12;
  constexpr int LCD_E = 11;
  constexpr int LCD_D4 = 5;
  constexpr int LCD_D5 = 4;
  constexpr int LCD_D6 = 3;
  constexpr int LCD_D7 = 2;

  // LCD config (row / column matrix)
  constexpr int LCD_COLS = 16; // LCD matrix column count
  constexpr int LCD_ROWS = 2; // LCD matrix row count
  constexpr unsigned LCD_RR = 500; // LCD refresh rate

  // Sensors / Actuators
  constexpr int PIN_MOISTURE = A0; // 1K Potentiometer (Moisture sensor voltage simulation)
  constexpr int PIN_SERVO = 8; // Servo signal PIN (Electric valve position simulation)

  // Servo angles (open/close position)
  constexpr int SERVO_ANGLE_OPEN = 90;
  constexpr int SERVO_ANGLE_MID = 45;
  constexpr int SERVO_ANGLE_CLOSE = 0;

  // Default valve behavior - hysteresis LOW/HIGH
  // SETTINGS: Moisture management low and high levels.
  constexpr int OPEN_BELOW_PCT = 30;     // LOW level (Moisture %)
  constexpr int CLOSE_ABOVE_PCT = 70;    // HIGH level (Moisture %)


}

// -- GLOBALS -- //
unsigned MOISTURE = 0; // Moisture level (%)
bool VALVE_OPEN = false; // VALVE OPEN/CLOSE status
unsigned LCD_TIMER = 0; // LCD refresh timer (to track refresh rate)
char LCD_ROW_0[17] = "SPRINKLER SYSTEM"; // LCD row 0 string (16 chars + stop)
char LCD_ROW_1[17] = "Starting..."; // LCD row 1 string (16 chars + stop)

// -- LCD -- //
LiquidCrystal lcd(CFG::LCD_RS, CFG::LCD_E, CFG::LCD_D4, CFG::LCD_D5, CFG::LCD_D6, CFG::LCD_D7);

void refreshLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(LCD_ROW_0);
  lcd.setCursor(0, 1);
  lcd.print(LCD_ROW_1);
}

void initLCD() {
  lcd.begin(CFG::LCD_COLS, CFG::LCD_ROWS);
  refreshLCD();
}

// -- Water Valve -- //
Servo valve;

void setValve(bool open) {
  valve.write(open ? CFG::SERVO_ANGLE_OPEN : CFG::SERVO_ANGLE_CLOSE);
  VALVE_OPEN = open; 
}

void initValve() {
  valve.attach(CFG::PIN_SERVO);
  setValve(false); // Initializes valve in closed position
}

void resolveValve() {
  if (!VALVE_OPEN && MOISTURE < CFG::OPEN_BELOW_PCT) {
    setValve(true);
  } else if (VALVE_OPEN && MOISTURE > CFG::CLOSE_ABOVE_PCT) {
    setValve(false);
  }
}

// -- Moisture sensor -- //
void initMoistureSensor() {
  pinMode(CFG::PIN_MOISTURE, INPUT);
}

int readMoisture() {
  // Reads 0-1023 value from moisture sensor and converts to 0-100% 
  int read = analogRead(CFG::PIN_MOISTURE); 
  int moisture = map(read, 0, 1023, 0, 100); 
  return moisture;
}

// -- SETUP -- //
void setup() {
  initValve(); // Moisture sensor init
  initLCD(); // LCD init
  initValve();

  Serial.begin(9600);
}

// -- LOOP -- //
void loop() {
  MOISTURE = readMoisture(); // Update moisture %
  resolveValve(); // Resolve valve position

  // Refresh LCD at given rate
  unsigned long now = millis();
  if (now - LCD_TIMER >= CFG::LCD_RR) {
    LCD_TIMER = now;
    // snprintf ensures LCD row size limit is followed (truncates before overflow errors)
    snprintf(LCD_ROW_0, sizeof(LCD_ROW_0), "Moist:%3d%% %s", MOISTURE, VALVE_OPEN ? "OPEN" : "CLOSE");
    snprintf(LCD_ROW_1, sizeof(LCD_ROW_1), "Operating...");
    refreshLCD();
  }
}


