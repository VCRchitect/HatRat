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
const float SENSITIVITY = 160.0f;       // px / deg
const float HORIZ_GAIN = -1.0f;         // flip L/R
const float PITCH_SIGN = -1.0f;         // look-down → cursor-down
const float DEADZONE_DEG = 0.03f;       // ignore < 0.03 °
const float SMOOTH_ALPHA = 0.3f;        // 0(off)…1(none)
const float FILTER_BETA = 0.11f;        // 0.10–0.15 snappy
const unsigned long LATCH_TIME = 2000;  // ms you must hold to “lock” drag


const float QUIET_DEG = 0.03f;  // hard dead-zone (º)
const float LIN_START = 0.10f;

/* Sip-&-Puff */
const long SIP_THR = -1205000;
const long PUFF_THR = 586000;
const byte HX_OVERSAMPLE = 2;
const float BASELINE_A = 0.005f;
const unsigned long CLICK_GAP = 100;  // ms
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
  static byte sCnt = 0;
  static long sSum = 0;
  static enum { IDLE,
                SIP_HOLD,
                PUFF_HOLD } st = IDLE;
  static unsigned long pressStart = 0;
  static unsigned long lastEdge = 0;
  static bool latchedRight = false;
  static bool latchedLeft = false;
  const long SIP_LOCK_THR = SIP_THR * 2.0;    // 70 % deeper
  const long PUFF_LOCK_THR = PUFF_THR * 2.0;  // 70 % higher

  /* 1. Read HX711 -------------------------------------------------- */
  if (!hx.is_ready()) return;
  sSum += hx.read();
  if (++sCnt < HX_OVERSAMPLE) return;

  long avg = sSum / sCnt;
  sSum = 0;
  sCnt = 0;
  baseline += (long)((avg - baseline) * BASELINE_A);
  diffFilt = diffFilt * 5 / 8 + (avg - baseline) * 3 / 8;

  /* 2. State machine ---------------------------------------------- */
  unsigned long now = millis();

  switch (st) {
    /* ---------- idle: waiting for a threshold crossing ------------- */
    case IDLE:
      /* --- SIP detected (right button) --- */
      if (diffFilt < SIP_THR && now - lastEdge > CLICK_GAP) {
        if (latchedRight) {  // unlock if already latched
          Mouse.release(MOUSE_RIGHT);
          latchedRight = false;
          lastEdge = now;
        } else {
          Mouse.press(MOUSE_RIGHT);  // start a potential click
          pressStart = now;
          st = SIP_HOLD;
        }
      }
      /* --- PUFF detected (left button) --- */
      else if (diffFilt > PUFF_THR && now - lastEdge > CLICK_GAP) {
        if (latchedLeft) {
          Mouse.release(MOUSE_LEFT);
          latchedLeft = false;
          lastEdge = now;
        } else {
          Mouse.press(MOUSE_LEFT);
          pressStart = now;
          st = PUFF_HOLD;
        }
      }
      break;

      /* ---------- right button is being held ----------------- */
    case SIP_HOLD:
      /* Lock only if you are STILL below the deeper line AND time met */
      if (!latchedRight && diffFilt < SIP_LOCK_THR && now - pressStart >= LATCH_TIME) {
        latchedRight = true;  // deliberate lock
        st = IDLE;
      }
      /* Release for ordinary click */
      else if (diffFilt > SIP_THR * 0.45f) {  // raise to 45 % so it lets go sooner
        Mouse.release(MOUSE_RIGHT);
        lastEdge = now;
        st = IDLE;
      }
      break;

    /* ---------- left button is being held ------------------ */
    case PUFF_HOLD:
      if (!latchedLeft && diffFilt > PUFF_LOCK_THR && now - pressStart >= LATCH_TIME) {
        latchedLeft = true;
        st = IDLE;
      } else if (diffFilt < PUFF_THR * 0.45f) {
        Mouse.release(MOUSE_LEFT);
        lastEdge = now;
        st = IDLE;
      }
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

inline float softGain(float deg) {
  float absd = fabsf(deg);
  if (absd <= QUIET_DEG)  // in the absolute dead-zone
    return 0.0f;

  float sign = (deg >= 0) ? 1.0f : -1.0f;

  if (absd <= LIN_START) {                                   // cubic easing segment
    float t = (absd - QUIET_DEG) / (LIN_START - QUIET_DEG);  // 0-1
    float eased = t * t * t;                                 // cubic ramp
    return sign * eased * LIN_START;                         // ≤ LIN_START so it blends
  } else {
    return deg;  // fully linear beyond 0.15°
  }
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

  accuX += softGain(dAng(yaw, lastYaw)) * SENSITIVITY * HORIZ_GAIN;
  accuY += softGain(pitch - lastPitch) * SENSITIVITY * PITCH_SIGN;
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