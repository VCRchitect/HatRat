/*********************************************************************
 *  Head-Tracker  +  Sip-&-Puff Mouse
 *  – 200 Hz Madgwick (9-DOF), non-blocking HX711 FSM,
 *    drift-free yaw, and two-position 90 ° mount switch
 *********************************************************************/

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <QMC5883LCompass.h>
#include <HX711.h>
#include <Adafruit_AHRS_Madgwick.h>
#include <Mouse.h>

/* ─── USER KNOBS ─────────────────────────────────────────── */
const float SENSITIVITY = 75.0f;   // px / deg
const float HORIZ_GAIN = -1.0f;    // flip L/R
const float PITCH_SIGN = -1.0f;    // look-down → cursor-down
const float DEADZONE_DEG = 0.0825f;  // ignore < 0.03 °
const float SMOOTH_ALPHA = 0.3f;  // 0(off)…1(none)
const float FILTER_BETA = 0.12f;   // 0.10–0.15 snappy

/* Sip-&-Puff */
const long SIP_THR = -255000;
const long PUFF_THR = 175000;
const byte HX_OVERSAMPLE = 2;
const float BASELINE_A = 0.001f;
const unsigned long CLICK_GAP = 50;  // ms
/* ─────────────────────────────────────────────────────────── */

/* Pin map */
#define PIN_UPRIGHT 0  // LOW when upright
#define PIN_ROLL90 1   // LOW when rolled left 90°
#define PIN_REZERO 2   // LOW → manual re-center
#define HX_DOUT 9
#define HX_SCK 4

/* +90 ° roll quaternion (about X) */
struct Quat {
  float w, x, y, z;
  Quat(float W = 1, float X = 0, float Y = 0, float Z = 0)
    : w(W), x(X), y(Y), z(Z) {}
  Quat conjugate() const {
    return Quat(w, -x, -y, -z);
  }
  Quat operator*(const Quat& b) const {
    return Quat(w * b.w - x * b.x - y * b.y - z * b.z,
                w * b.x + x * b.w + y * b.z - z * b.y,
                w * b.y - x * b.z + y * b.w + z * b.x,
                w * b.z + x * b.y - y * b.x + z * b.w);
  }
};
const Quat Q_ROLL90(0.7071068f, 0.7071068f, 0, 0);

/* ─── GLOBALS ────────────────────────────────────────────── */
MPU6050 mpu;
QMC5883LCompass compass;
Adafruit_Madgwick filter;
HX711 hx;

Quat qRefInv(1, 0, 0, 0);
bool haveRef = false, modeRolled = false, prevModeRolled = false;
float lastYaw = 0, lastPitch = 0, accuX = 0, accuY = 0;

/* Sip-&-puff shared vars */
long baseline = 0;
long diffFilt = 0;

/* ─── helpers ────────────────────────────────────────────── */
inline float dz(float v) {
  return (abs(v) < DEADZONE_DEG) ? 0.0f : v;
}
inline float d2(float r) {
  return r * 57.2957795f;
}
inline float w180(float a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}
inline float dAng(float a, float b) {
  return w180(a - b);
}

void zeroOrientation() {
  float w, x, y, z;
  filter.getQuaternion(&w, &x, &y, &z);
  qRefInv = Quat(w, x, y, z).conjugate();
  haveRef = true;
}

/* ─── Sip-&-Puff FSM  (non-blocking) ────────────────────── */
void sipPuff() {
  static byte sCnt = 0;  // oversample counter
  static long sSum = 0;  // running total
  static enum { IDLE,
                SIP,
                PUFF } st = IDLE;
  static unsigned long lastClick = 0;

  if (!hx.is_ready()) return;          // HX711 still converting
  sSum += hx.read();                   // accumulate a sample
  if (++sCnt < HX_OVERSAMPLE) return;  // wait for N samples

  long avg = sSum / sCnt;  // --------- one complete reading ---------
  sSum = 0;
  sCnt = 0;

  baseline += (long)((avg - baseline) * BASELINE_A);
  diffFilt = diffFilt * 7 / 8 + (avg - baseline) / 8;

  /* ── OPTIONAL DEBUG OUTPUT ───────────────────────────── */
  static unsigned long lastDbg = 0;
  if (millis() - lastDbg > 50) {  // ~20 Hz is plenty
    Serial.print(F("raw="));
    Serial.print(avg);
    Serial.print(F("  base="));
    Serial.print(baseline);
    Serial.print(F("  diff="));
    Serial.println(diffFilt);
    lastDbg = millis();
  }
  /* ────────────────────────────────────────────────────── */

  unsigned long now = millis();
  switch (st) {
    case IDLE:
      if (diffFilt < SIP_THR && now - lastClick > CLICK_GAP) {
        Mouse.click(MOUSE_RIGHT);
        st = SIP;
        lastClick = now;
      } else if (diffFilt > PUFF_THR && now - lastClick > CLICK_GAP) {
        Mouse.click(MOUSE_LEFT);
        st = PUFF;
        lastClick = now;
      }
      break;

    case SIP:
      if (diffFilt > SIP_THR / 2) st = IDLE;
      break;
    case PUFF:
      if (diffFilt < PUFF_THR / 2) st = IDLE;
      break;
  }
}


/* ─── SETUP ──────────────────────────────────────────────── */
void setup() {
  Serial.begin(115200);
  pinMode(PIN_UPRIGHT, INPUT_PULLUP);
  pinMode(PIN_ROLL90, INPUT_PULLUP);
  pinMode(PIN_REZERO, INPUT_PULLUP);

  Wire.begin();
  Wire.setClock(400000UL);
  compass.init();
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  filter.begin(200);
  filter.setBeta(FILTER_BETA);

  hx.begin(HX_DOUT, HX_SCK);
  hx.set_gain(128);
  long s = 0;
  for (byte i = 0; i < 32; i++) {
    while (!hx.is_ready())
      ;
    s += hx.read();
  }
  baseline = s / 32L;

  Mouse.begin();
}

/* ─── LOOP ───────────────────────────────────────────────── */
void loop() {
  /* 1. IMU + Compass → Madgwick */
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  const float G = 1.0f / 16384.0f, D = 1.0f / 131.0f;
  compass.read();
  filter.update(gx * D, gy * D, gz * D,
                ax * G, ay * G, az * G,
                compass.getX(), compass.getY(), compass.getZ());

  if (!haveRef) zeroOrientation();

  /* 2. switch-selected roll correction */
  modeRolled = (digitalRead(PIN_ROLL90) == LOW);
  if (modeRolled != prevModeRolled) {
    zeroOrientation();
    prevModeRolled = modeRolled;
  }

  float w, x, y, z;
  filter.getQuaternion(&w, &x, &y, &z);
  Quat q = Quat(w, x, y, z) * qRefInv;
  if (modeRolled) q = q * Q_ROLL90;

  /* 3. Euler → mouse deltas */
  float yaw = d2(atan2f(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)));
  float pitch = d2(asinf(2 * (q.w * q.y - q.z * q.x)));

  accuX += dz(dAng(yaw, lastYaw)) * SENSITIVITY * HORIZ_GAIN;
  accuY += dz(pitch - lastPitch) * SENSITIVITY * PITCH_SIGN;
  lastYaw = yaw;
  lastPitch = pitch;

  int px = int(accuX);
  accuX -= px;
  int py = int(accuY);
  accuY -= py;
  static float sX = 0, sY = 0;
  sX = sX * (1 - SMOOTH_ALPHA) + px * SMOOTH_ALPHA;
  sY = sY * (1 - SMOOTH_ALPHA) + py * SMOOTH_ALPHA;
  int mX = int(sX);
  sX -= mX;
  int mY = int(sY);
  sY -= mY;
  Mouse.move(mX, mY, 0);

  if (digitalRead(PIN_REZERO) == LOW) zeroOrientation();

  /* 4. Sip-&-Puff */
  sipPuff();
}