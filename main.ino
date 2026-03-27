#include <Wire.h>
#include <math.h>
#include "RTClib.h"
#include "DHT.h"

// ==================== CONFIG ====================
#define SERIAL_BAUD 115200

// Pins
const int PIN_PIR   = 2;
const int PIN_DHT   = 4;
const int PIN_BUZZ  = 6;
const int PIN_RGB_R = 9;
const int PIN_RGB_G = 10;
const int PIN_RGB_B = 11;
const int PIN_LDR   = A0;

// I2C addresses
const uint8_t RTC_ADDR = 0x68;   // DS1307 fixed
const uint8_t IMU_ADDR = 0x69;   // MPU must be 0x69 (AD0 -> 3.3V)

// DHT
#define DHTTYPE DHT11
DHT dht(PIN_DHT, DHTTYPE);

// RTC
RTC_DS1307 rtc;

// MPU-6500 registers
static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t REG_WHO_AM_I     = 0x75;

// UI refresh
const uint16_t UI_MS = 250;

// Face detection tuning
const float ALPHA = 0.22f;
const float DOMINANCE = 0.22f;
const uint16_t FACE_STABLE_MS = 300;

// Timer tilt tuning
const float TILT_STEP_G = 0.35f;
const uint16_t TILT_COOLDOWN_MS = 650;

// ==================== ENUMS (MUST BE ABOVE FUNCTIONS) ====================
enum Mode { DASHBOARD, TIMER, MEDITATION, WORKOUT, REMINDERS, SLEEP_ALARM };
enum Face { FRONT, RIGHT, TOP, LEFT, BACK, BOTTOM, UNKNOWN };

// ==================== NAME HELPERS ====================
const char* modeLabel(Mode m){
  switch(m){
    case DASHBOARD: return "DASHBOARD";
    case TIMER: return "TIMER";
    case MEDITATION: return "MEDITATION";
    case WORKOUT: return "WORKOUT";
    case REMINDERS: return "REMINDERS";
    case SLEEP_ALARM: return "SLEEP/ALARM";
  }
  return "?";
}

const char* faceLabel(Face f){
  switch(f){
    case FRONT: return "FRONT";
    case RIGHT: return "RIGHT";
    case TOP: return "TOP";
    case LEFT: return "LEFT";
    case BACK: return "BACK";
    case BOTTOM: return "BOTTOM";
    default: return "UNKNOWN";
  }
}

// ==================== STATE ====================
Mode mode = DASHBOARD;

Face candFace = UNKNOWN;
Face lockedFace = UNKNOWN;
unsigned long candSince = 0;

float axg=0, ayg=0, azg=1; // smoothed accel

bool present = false;
int ldrRaw = 0;

float tempC = NAN, hum = NAN;
unsigned long lastDhtMs = 0;

unsigned long lastUiMs = 0;

// TIMER
int timerMinutes = 25;
long timerRemainingMs = 0;
bool timerRunning = false;
unsigned long lastTiltMs = 0;

// MEDITATION
int medMinutes = 5;
long medRemainingMs = 0;
bool medRunning = false;

// WORKOUT
int workSec = 45;
int restSec = 15;
int rounds = 6;
int currentRound = 1;
bool inWork = true;
long phaseRemainingMs = 0;
bool workoutRunning = false;

// ALARM
int alarmHour = 7;
int alarmMinute = 0;
bool alarmEnabled = true;
bool alarmRinging = false;

// REMINDERS
const char* reminder1 = "Drink water";
const char* reminder2 = "Stand up";

// ==================== I2C HELPERS ====================
void i2cWrite(uint8_t addr, uint8_t reg, uint8_t val){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(addr, n);
  if (Wire.available() < n) return false;
  for(uint8_t i=0;i<n;i++) buf[i]=Wire.read();
  return true;
}

// ==================== IMU ====================
bool imuInit(){
  i2cWrite(IMU_ADDR, REG_PWR_MGMT_1, 0x00);
  delay(50);
  i2cWrite(IMU_ADDR, REG_ACCEL_CONFIG, 0x00);
  delay(10);

  uint8_t who=0;
  if(!i2cReadBytes(IMU_ADDR, REG_WHO_AM_I, &who, 1)) return false;
  return (who == 0x70); // MPU-6500 expected
}

bool imuReadAccelRaw(int16_t &ax, int16_t &ay, int16_t &az){
  uint8_t b[6];
  if(!i2cReadBytes(IMU_ADDR, REG_ACCEL_XOUT_H, b, 6)) return false;
  ax = (int16_t)(b[0]<<8 | b[1]);
  ay = (int16_t)(b[2]<<8 | b[3]);
  az = (int16_t)(b[4]<<8 | b[5]);
  return true;
}

// ==================== FEEDBACK ====================
void setRgb(uint8_t r, uint8_t g, uint8_t b){
  analogWrite(PIN_RGB_R, r);
  analogWrite(PIN_RGB_G, g);
  analogWrite(PIN_RGB_B, b);
}

void beep(int freq, int ms){
  tone(PIN_BUZZ, freq, ms);
}

void modeFeedback(Mode m){
  switch(m){
    case DASHBOARD:  setRgb(0, 0, 60);  beep(900, 35); break;
    case TIMER:      setRgb(80, 30, 0); beep(950, 35); break;
    case MEDITATION: setRgb(40, 0, 70); beep(780, 55); break;
    case WORKOUT:    setRgb(0, 70, 0);  beep(980, 35); break;
    case REMINDERS:  setRgb(0, 45, 70); beep(880, 35); break;
    case SLEEP_ALARM:setRgb(0, 0, 0);   beep(700, 25); break;
  }
}

// ==================== FACE DETECTION ====================
bool dominantEnough(float x, float y, float z){
  float ax=fabs(x), ay=fabs(y), az=fabs(z);
  float maxv=ax, midv=ay;
  if(ay>maxv){ midv=maxv; maxv=ay; } else midv=ay;
  if(az>maxv){ midv=maxv; maxv=az; }
  else if(az>midv) midv=az;
  return (maxv - midv) >= DOMINANCE;
}

Face computeFaceUp(float x, float y, float z){
  float ax=fabs(x), ay=fabs(y), az=fabs(z);
  if(az >= ax && az >= ay) return (z >= 0) ? FRONT : BACK;
  if(ax >= ay && ax >= az) return (x >= 0) ? RIGHT : LEFT;
  return (y >= 0) ? TOP : BOTTOM;
}

Mode modeForFace(Face f){
  switch(f){
    case FRONT:  return DASHBOARD;
    case RIGHT:  return TIMER;
    case TOP:    return MEDITATION;
    case LEFT:   return WORKOUT;
    case BACK:   return REMINDERS;
    case BOTTOM: return SLEEP_ALARM;
    default:     return mode;
  }
}

void updateFaceAndMode(){
  int16_t rx, ry, rz;
  if(!imuReadAccelRaw(rx, ry, rz)) return;

  float x = rx / 16384.0f;
  float y = ry / 16384.0f;
  float z = rz / 16384.0f;

  axg += ALPHA * (x - axg);
  ayg += ALPHA * (y - ayg);
  azg += ALPHA * (z - azg);

  Face newCand = dominantEnough(axg, ayg, azg) ? computeFaceUp(axg, ayg, azg) : UNKNOWN;

  unsigned long now = millis();

  if(newCand != candFace){
    candFace = newCand;
    candSince = now;
  }

  if(candFace != lockedFace && (now - candSince) >= FACE_STABLE_MS){
    lockedFace = candFace;
    Mode newMode = modeForFace(lockedFace);
    if(newMode != mode){
      mode = newMode;

      // reset mode-specific state on entry
      if(mode != TIMER) timerRunning = false;
      if(mode != MEDITATION) medRunning = false;
      if(mode != WORKOUT) workoutRunning = false;
      if(mode != SLEEP_ALARM) alarmRinging = false;

      modeFeedback(mode);
    }
  }
}

// ==================== SENSOR UPDATES ====================
void updatePresence(){
  present = (digitalRead(PIN_PIR) == HIGH);
}

void updateBrightness(){
  ldrRaw = analogRead(PIN_LDR);
}

void updateDht(){
  unsigned long now = millis();
  if(now - lastDhtMs < 2000) return;
  lastDhtMs = now;

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if(!isnan(h)) hum = h;
  if(!isnan(t)) tempC = t;
}

// ==================== MODE LOGIC ====================
void updateTimerLogic(long dtMs){
  if(mode != TIMER) return;

  unsigned long now = millis();
  if(now - lastTiltMs >= TILT_COOLDOWN_MS){
    if(ayg > TILT_STEP_G){
      timerMinutes += 5;
      if(timerMinutes > 180) timerMinutes = 180;
      beep(1050, 25);
      lastTiltMs = now;
    } else if(ayg < -TILT_STEP_G){
      timerMinutes -= 5;
      if(timerMinutes < 5) timerMinutes = 5;
      beep(900, 25);
      lastTiltMs = now;
    }
  }

  if(!timerRunning){
    timerRemainingMs = (long)timerMinutes * 60L * 1000L;
    timerRunning = true;
    beep(980, 40);
  }

  if(timerRunning && timerRemainingMs > 0){
    timerRemainingMs -= dtMs;
    if(timerRemainingMs <= 0){
      timerRemainingMs = 0;
      timerRunning = false;
      beep(880, 120); delay(60);
      beep(988, 160);
    }
  }
}

void updateMeditationLogic(long dtMs){
  if(mode != MEDITATION) return;

  if(!medRunning){
    medRemainingMs = (long)medMinutes * 60L * 1000L;
    medRunning = true;
    beep(660, 120);
  }

  if(medRunning && medRemainingMs > 0){
    medRemainingMs -= dtMs;
    if(medRemainingMs <= 0){
      medRemainingMs = 0;
      medRunning = false;
      beep(660, 200); delay(60);
      beep(550, 250);
    }
  }
}

void updateWorkoutLogic(long dtMs){
  if(mode != WORKOUT) return;

  if(!workoutRunning){
    workoutRunning = true;
    currentRound = 1;
    inWork = true;
    phaseRemainingMs = workSec * 1000L;
    beep(1000, 80);
  }

  if(workoutRunning && phaseRemainingMs > 0){
    phaseRemainingMs -= dtMs;
    if(phaseRemainingMs <= 0){
      if(inWork){
        inWork = false;
        phaseRemainingMs = restSec * 1000L;
        beep(750, 80);
      } else {
        currentRound++;
        if(currentRound > rounds){
          workoutRunning = false;
          beep(900, 200); delay(60);
          beep(1200, 200);
        } else {
          inWork = true;
          phaseRemainingMs = workSec * 1000L;
          beep(1000, 80);
        }
      }
    }
  }
}

void updateSleepAlarmLogic(){
  if(mode != SLEEP_ALARM) return;

  DateTime now = rtc.now();

  if(alarmEnabled && !alarmRinging &&
     now.hour() == alarmHour && now.minute() == alarmMinute && now.second() < 2){
    alarmRinging = true;
  }

  if(alarmRinging){
    beep(800, 200); delay(250);
    beep(950, 200); delay(350);
  }
}

// ==================== SERIAL UI ====================
void renderSerial(){
  DateTime now = rtc.now();

  Serial.println();
  Serial.println("========================================");
  Serial.print("MODE: "); Serial.print(modeLabel(mode));
  Serial.print("   FACE_UP: "); Serial.println(faceLabel(lockedFace));

  Serial.print("TIME: ");
  Serial.print(now.year()); Serial.print("-");
  if(now.month()<10) Serial.print("0"); Serial.print(now.month()); Serial.print("-");
  if(now.day()<10) Serial.print("0"); Serial.print(now.day()); Serial.print("  ");
  if(now.hour()<10) Serial.print("0"); Serial.print(now.hour()); Serial.print(":");
  if(now.minute()<10) Serial.print("0"); Serial.print(now.minute()); Serial.print(":");
  if(now.second()<10) Serial.print("0"); Serial.println(now.second());

  Serial.print("PRESENCE (PIR): "); Serial.println(present ? "YES" : "NO");
  Serial.print("BRIGHTNESS (LDR A0): "); Serial.println(ldrRaw);

  Serial.print("ROOM: ");
  if(isnan(tempC)) Serial.print("Temp ?  ");
  else { Serial.print(tempC,1); Serial.print("C  "); }
  if(isnan(hum)) Serial.println("Hum ?");
  else { Serial.print(hum,0); Serial.println("%"); }

  Serial.print("ACCEL g: x="); Serial.print(axg,2);
  Serial.print(" y="); Serial.print(ayg,2);
  Serial.print(" z="); Serial.println(azg,2);

  if(mode == TIMER){
    long s = timerRemainingMs/1000L;
    int mm = (int)(s/60L);
    int ss = (int)(s%60L);
    Serial.print("TIMER SET: "); Serial.print(timerMinutes); Serial.println(" min (tilt +/- 5)");
    Serial.print("REMAIN: "); Serial.print(mm); Serial.print("m "); Serial.print(ss); Serial.println("s");
  } else if(mode == MEDITATION){
    long s = medRemainingMs/1000L;
    Serial.print("MEDITATION: "); Serial.print((int)(s/60)); Serial.print("m ");
    Serial.print((int)(s%60)); Serial.println("s");
  } else if(mode == WORKOUT){
    Serial.print("WORKOUT: round "); Serial.print(currentRound); Serial.print("/"); Serial.print(rounds);
    Serial.print(" phase "); Serial.print(inWork ? "WORK" : "REST");
    Serial.print(" remain "); Serial.print(phaseRemainingMs/1000L); Serial.println("s");
  } else if(mode == REMINDERS){
    Serial.print("REMINDER 1: "); Serial.println(reminder1);
    Serial.print("REMINDER 2: "); Serial.println(reminder2);
  } else if(mode == SLEEP_ALARM){
    Serial.print("ALARM: ");
    if(alarmHour<10) Serial.print("0"); Serial.print(alarmHour); Serial.print(":");
    if(alarmMinute<10) Serial.print("0"); Serial.print(alarmMinute);
    Serial.print(" enabled="); Serial.print(alarmEnabled ? "YES":"NO");
    Serial.print(" ringing="); Serial.println(alarmRinging ? "YES":"NO");
  }

  Serial.println("========================================");
}

// ==================== SETUP / LOOP ====================
void setup(){
  Serial.begin(SERIAL_BAUD);
  Wire.begin();
  Wire.setClock(100000);

  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_BUZZ, OUTPUT);
  pinMode(PIN_RGB_R, OUTPUT);
  pinMode(PIN_RGB_G, OUTPUT);
  pinMode(PIN_RGB_B, OUTPUT);
  setRgb(0,0,0);

  dht.begin();

  // RTC
  if(!rtc.begin()){
    Serial.println("RTC not found. Check DS1307 wiring.");
    while(1) delay(10);
  }
  if(!rtc.isrunning()){
    Serial.println("RTC not running. Setting to compile time...");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // IMU
  if(!imuInit()){
    Serial.println("IMU init FAILED.");
    Serial.println("Check MPU is at 0x69 (AD0->3.3V) and WHO_AM_I should be 0x70.");
    while(1) delay(10);
  }

  // Prime accel smoothing
  int16_t rx,ry,rz;
  for(int i=0;i<10;i++){
    if(imuReadAccelRaw(rx,ry,rz)){
      axg = rx/16384.0f;
      ayg = ry/16384.0f;
      azg = rz/16384.0f;
    }
    delay(20);
  }

  candSince = millis();
  modeFeedback(mode);

  Serial.println("Boot OK. Rotate cube to switch modes. Serial Monitor = display for now.");
}

void loop(){
  static unsigned long lastMs = millis();
  unsigned long now = millis();
  long dt = (long)(now - lastMs);
  lastMs = now;

  updateFaceAndMode();
  updatePresence();
  updateBrightness();
  updateDht();

  updateTimerLogic(dt);
  updateMeditationLogic(dt);
  updateWorkoutLogic(dt);

  if(mode == SLEEP_ALARM) updateSleepAlarmLogic();

  if(now - lastUiMs >= UI_MS){
    lastUiMs = now;
    renderSerial();
  }
}