#include <Wire.h>
#include <math.h>

/*
  MPU-6500 (GY-521 style) minimal driver
  - Reads accelerometer only (enough for cube face detection)
  - No external MPU library (avoids “wrong chip / wrong ID” issues)
  - Designed to be pasted into a larger sketch

  Wiring (UNO/Nano):
    SDA -> A4
    SCL -> A5
    VCC -> 3.3V (recommended)
    GND -> GND
    AD0 -> GND  => address 0x68
    AD0 -> 3.3V => address 0x69  (use this if DS1307 is also present at 0x68)
*/

// ===== Set address here =====
static const uint8_t MPU_ADDR = 0x69; // change to 0x68 if AD0=GND

// Registers (MPU-6500)
static const uint8_t REG_PWR_MGMT_1   = 0x6B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t REG_WHO_AM_I     = 0x75;

// Accel sensitivity for +/-2g (CONFIG=0)
static const float ACC_LSB_PER_G = 16384.0f;

// Face detection threshold:
// require the strongest axis to exceed the second-strongest by at least this many g
static const float FACE_DOMINANCE_G = 0.22f;

enum Face { FACE_FRONT, FACE_RIGHT, FACE_TOP, FACE_LEFT, FACE_BACK, FACE_BOTTOM, FACE_UNKNOWN };

// ---------- Low-level I2C helpers ----------
static inline void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

static inline bool mpuRead(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false; // repeated start
  Wire.requestFrom(MPU_ADDR, n);
  if (Wire.available() < n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

// ---------- Public API ----------
bool mpuBegin(bool printWhoAmI = true) {
  // Wake up
  mpuWrite(REG_PWR_MGMT_1, 0x00);
  delay(30);

  // Accel range = +/-2g (best for face detection)
  mpuWrite(REG_ACCEL_CONFIG, 0x00);
  delay(10);

  // Verify chip ID
  uint8_t who = 0;
  if (!mpuRead(REG_WHO_AM_I, &who, 1)) return false;

  if (printWhoAmI) {
    Serial.print("MPU WHO_AM_I @0x"); Serial.print(MPU_ADDR, HEX);
    Serial.print(" = 0x"); Serial.println(who, HEX);
  }

  // MPU-6500 commonly reports 0x70
  return (who == 0x70);
}

// Read accel in g. Returns false if I2C read fails.
bool mpuReadAccelG(float &ax_g, float &ay_g, float &az_g) {
  uint8_t b[6];
  if (!mpuRead(REG_ACCEL_XOUT_H, b, 6)) return false;

  int16_t ax = (int16_t)(b[0] << 8 | b[1]);
  int16_t ay = (int16_t)(b[2] << 8 | b[3]);
  int16_t az = (int16_t)(b[4] << 8 | b[5]);

  ax_g = ax / ACC_LSB_PER_G;
  ay_g = ay / ACC_LSB_PER_G;
  az_g = az / ACC_LSB_PER_G;
  return true;
}

// Determine which cube face is UP based on accel direction.
// Convention (adjust later to match your physical mounting):
//   +Z => FRONT,  -Z => BACK
//   +X => RIGHT,  -X => LEFT
//   +Y => TOP,    -Y => BOTTOM
Face mpuFaceUp(float ax_g, float ay_g, float az_g) {
  float ax = fabs(ax_g), ay = fabs(ay_g), az = fabs(az_g);

  // Reject ambiguous “edge/corner” orientations
  // by checking dominance margin (max axis - second axis)
  float maxv = ax, midv = ay;
  if (ay > maxv) { midv = maxv; maxv = ay; } else midv = ay;
  if (az > maxv) { midv = maxv; maxv = az; }
  else if (az > midv) midv = az;

  if ((maxv - midv) < FACE_DOMINANCE_G) return FACE_UNKNOWN;

  // Pick dominant axis and sign
  if (az >= ax && az >= ay) return (az_g >= 0) ? FACE_FRONT : FACE_BACK;
  if (ax >= ay && ax >= az) return (ax_g >= 0) ? FACE_RIGHT : FACE_LEFT;
  return (ay_g >= 0) ? FACE_TOP : FACE_BOTTOM;
}

// Optional helper: label for printing
const char* faceToStr(Face f) {
  switch (f) {
    case FACE_FRONT:  return "FRONT";
    case FACE_RIGHT:  return "RIGHT";
    case FACE_TOP:    return "TOP";
    case FACE_LEFT:   return "LEFT";
    case FACE_BACK:   return "BACK";
    case FACE_BOTTOM: return "BOTTOM";
    default:          return "UNKNOWN";
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);

  if (!mpuBegin(true)) {
    Serial.println("MPU init failed. Check address/wiring/AD0.");
    while (1) delay(10);
  }
}

void loop() {
  float ax, ay, az;
  if (mpuReadAccelG(ax, ay, az)) {
    Face f = mpuFaceUp(ax, ay, az);
    Serial.print("ax ay az: ");
    Serial.print(ax, 2); Serial.print(" ");
    Serial.print(ay, 2); Serial.print(" ");
    Serial.print(az, 2); Serial.print(" | UP=");
    Serial.println(faceToStr(f));
  } else {
    Serial.println("MPU read failed.");
  }
  delay(150);
}