/*
  Marko.ino
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
#include <Wire.h>

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
  constexpr int LCD_ROW_LEN = 17; // 16 chars + stop. 
  constexpr unsigned LCD_RR = 500; // LCD refresh rate
  constexpr unsigned DISPLAY_RR = 5000; // LCD data display row iteration interval (changes context every 5 seconds)

  // Sensors / Actuators
  constexpr int PIN_MOISTURE = A0; // 1K Potentiometer (Moisture sensor voltage simulation)
  constexpr int PIN_SERVO = 8; // Servo signal PIN (Electric valve position simulation)

  // Servo angles (open/close position)
  constexpr int SERVO_ANGLE_OPEN = 90;
  constexpr int SERVO_ANGLE_MID = 45;
  constexpr int SERVO_ANGLE_CLOSE = 0;

  // Default valve behavior - hysteresis LOW/HIGH - SET default levels here
  constexpr int OPEN_BELOW_PCT = 30;     // LOW level (Moisture %)
  constexpr int CLOSE_ABOVE_PCT = 70;    // HIGH level (Moisture %)
  
  constexpr int I2C_RR = 1000; 
  constexpr int TOM_ADDR = 0x08;
  constexpr int MANSI_ADDR = 0x09;
  constexpr int KRISHNA_ADDR = 0x0A;
}

// -- GLOBALS -- //
unsigned MOISTURE = 0; // Moisture level (%)
bool VALVE_OPEN = false; // VALVE OPEN/CLOSE status

unsigned long LCD_TIMER = 0; // LCD refresh timer (to track refresh rate)
unsigned long DISPLAY_TIMER = 0; // LCD bottom row display interval timer
char LCD_ROW_0[CFG::LCD_ROW_LEN] = "SPRINKLER SYSTEM"; 
char LCD_ROW_1[CFG::LCD_ROW_LEN] = "Starting..."; 

char ROW_SYS[CFG::LCD_ROW_LEN];
char ROW_ENV[CFG::LCD_ROW_LEN]; // Tom's data display row
char ROW_TANK[CFG::LCD_ROW_LEN]; // Mansi's data display row

const int DATA_ROW_COUNT = 2; // Update this as list of display data changes
char DATA_ROWS[DATA_ROW_COUNT][CFG::LCD_ROW_LEN];
int DATA_IDX = 0;
unsigned long I2C_TIMER = 0;

// -- DISPLAY DATA formatting -- //
void formatEnvDataRow(int temp, int air, int light){
  char t[3] = "XX"; // temp
  char a[4] = "XXX"; // air 
  char l[4] = "XXX"; // light
  if (temp >= 0) snprintf(t, 3, "%2d", temp);
  if (air >= 0) snprintf(a, 4, "%3d", air);
  if (light >= 0) snprintf(l, 4, "%3d", light);
  snprintf(ROW_ENV, CFG::LCD_ROW_LEN, "T%sC A%s%% L%s%%", t, a, l);
}

void formatTankDataRow(int temp, int level) {
  char t[3] = "XX"; // temp
  char l[4] = "XXX"; // level
  if (temp >= 0) snprintf(t, 3, "%2d", temp);
  if (level >= 0) snprintf(l, 4, "%3d", level);
  snprintf(ROW_TANK, CFG::LCD_ROW_LEN, "TANK %s%% T%sC", l, t);
}

void formatSystemDataRow(int moisture, bool open){
  char m[4] = "XXX";
  if (moisture >= 0) snprintf(m, 4, "%3d", moisture);
  const char* s = open ? " OPEN" : "CLOSE"; 
  snprintf(ROW_SYS, CFG::LCD_ROW_LEN, "MOIST:%s%% %s", m, s);
}

void refreshDataRows() {
  snprintf(DATA_ROWS[0], CFG::LCD_ROW_LEN, "%-16.16s", ROW_ENV);
  snprintf(DATA_ROWS[1], CFG::LCD_ROW_LEN, "%-16.16s", ROW_TANK);
}

void initDataRows() {
  formatSystemDataRow(-1, false); // INITS ROW_SYS
  formatEnvDataRow(-1, -1, -1); // INITS ROW_ENV
  formatTankDataRow(-1, -1); // INITS ROW_TANK
  refreshDataRows();
}

// -- I2C / Wire communication -- //
enum class ReadingType : int {
  ENV_TEMP, // Environment temperature
  ENV_AIR, // Environment air quality 
  ENV_LIGHT, // Environment light intensity 
  TANK_LEVEL, // Water tank level
  TANK_TEMP, // Water tank temperature
  COUNT // COUNT needs to be last type always (non reading type)
};

struct ReadingData {
  unsigned addr; // Slave address (your designated addr)
  int reading; // Reading value (value to be displayed)
  ReadingType type; // Type of reading (will be used for correct formatting / context)
};

// -- I2C MASTER -- //
int READING_VALUES[(int)ReadingType::COUNT];

void initI2C() {
  I2C_TIMER = millis();
  for (int i = 0; i < (int)ReadingType::COUNT; i++) READING_VALUES[i] = -1;
}

void pollEnvData() {
  Wire.requestFrom((int)CFG::TOM_ADDR, 3);
  if (Wire.available() == 3) {
    int temp = Wire.read();
    int air = Wire.read();
    int light = Wire.read();
    formatEnvDataRow(temp, air, light);
    refreshDataRows();
  }
}

void pollTankData() {
  Wire.requestFrom((int)CFG::MANSI_ADDR, 2);
  if (Wire.available() == 2) {
    int level = Wire.read();
    int temp = Wire.read();
    formatTankDataRow(temp, level);
    refreshDataRows();
  }
}

void i2cTick(unsigned long now) {
  if (now - I2C_TIMER >= (int)CFG::I2C_RR) {
    pollEnvData();
    pollTankData();
  }
}

// -- LCD -- //
LiquidCrystal lcd(CFG::LCD_RS, CFG::LCD_E, CFG::LCD_D4, CFG::LCD_D5, CFG::LCD_D6, CFG::LCD_D7);

void refreshLCD() {
  lcd.setCursor(0, 0);
  lcd.print(LCD_ROW_0);
  lcd.setCursor(0, 1);
  lcd.print(LCD_ROW_1);
}

void initLCD() {
  DISPLAY_TIMER = millis();
  lcd.begin(CFG::LCD_COLS, CFG::LCD_ROWS);
  lcd.clear();
  refreshLCD();
}

void lcdTick(unsigned long now) {
  if (now - DISPLAY_TIMER >= CFG::DISPLAY_RR) {
    DISPLAY_TIMER = now;
    DATA_IDX = (DATA_IDX + 1) % DATA_ROW_COUNT;
  }

  formatSystemDataRow(MOISTURE, VALVE_OPEN);
  refreshDataRows(); //Refresh data rows each LCD tick in case there has been updates

  snprintf(LCD_ROW_0, CFG::LCD_ROW_LEN, "%-16.16s", ROW_SYS);
  snprintf(LCD_ROW_1, CFG::LCD_ROW_LEN, "%-16.16s", DATA_ROWS[DATA_IDX]); 
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

void updateMoisture() {
  // Reads 0-1023 value from moisture sensor and converts to 0-100% 
  int read = analogRead(CFG::PIN_MOISTURE); 
  MOISTURE = map(read, 0, 1023, 0, 100);
}

// -- SETUP -- //
void setup() {
  initMoistureSensor();
  initDataRows();
  initLCD(); 
  initValve();
  initI2C();

  Wire.begin(); // I2C Master
  Wire.setClock(100000);
  Serial.begin(9600);
}

// -- LOOP -- //
void loop() {
  unsigned long now = millis(); // Loop tick timestamp
  updateMoisture(); // Reads & updates global MOISTURE %
  resolveValve(); // Resolve valve position


  i2cTick(now);

  // LCD Tick
  if (now - LCD_TIMER >= CFG::LCD_RR) {
    LCD_TIMER = now;
    lcdTick(now);
  }
}


