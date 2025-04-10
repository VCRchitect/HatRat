#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Mouse.h>

// Create MPU6050 instance
MPU6050 mpu;

// DMP-related variables
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// Orientation/motion vars
Quaternion q;
VectorFloat gravity;
float ypr[3]; // [yaw, pitch, roll] in radians from DMP

// ----- CURSOR MOVEMENT -----
float lastYaw   = 0.0f;
float lastPitch = 0.0f;
float sensitivity = 60.0f;
static float accuX = 0.0f;
static float accuY = 0.0f;

// ----- STILLNESS / LOCK LOGIC -----
bool locked = false;          
static unsigned long stillStartTime = 0;
static const float STILL_DEADZONE = 3.5f;   // degrees threshold for "still"
static const unsigned long STILL_THRESHOLD_MS = 1750; // hold still 2s => lock
float lockYaw   = 0.0f;  
float lockPitch = 0.0f;  

// ----- PRESS-AND-HOLD TILT LOGIC -----
static const float ROLL_TILT_THRESHOLD = 7.0f;  
static bool leftHeld  = false;
static bool rightHeld = false;

// ---------------------------------------------------------------------
// Press-and-Hold Tilt Function
// ---------------------------------------------------------------------
void handleTiltPress(float rollDeg) {
  // If roll < -15 => hold left button
  if (rollDeg < -ROLL_TILT_THRESHOLD) {
    // Press left if not already pressed
    if (!leftHeld) {
      Mouse.press(MOUSE_LEFT);
      leftHeld = true;
    }
    // If right was held, release it
    if (rightHeld) {
      Mouse.release(MOUSE_RIGHT);
      rightHeld = false;
    }
  }
  // If roll > +15 => hold right button
  else if (rollDeg > ROLL_TILT_THRESHOLD) {
    // Press right if not already pressed
    if (!rightHeld) {
      Mouse.press(MOUSE_RIGHT);
      rightHeld = true;
    }
    // If left was held, release it
    if (leftHeld) {
      Mouse.release(MOUSE_LEFT);
      leftHeld = false;
    }
  }
  // If roll is within ±15 => release any held buttons
  else {
    if (leftHeld) {
      Mouse.release(MOUSE_LEFT);
      leftHeld = false;
    }
    if (rightHeld) {
      Mouse.release(MOUSE_RIGHT);
      rightHeld = false;
    }
  }
}

// ---------------------------------------------------------------------
void setup() {
  Wire.begin();
  Serial.begin(115200);

  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // Example calibration offsets—adjust for your MPU
  mpu.setXAccelOffset(-126);
  mpu.setYAccelOffset(155);
  mpu.setZAccelOffset(1557);
  mpu.setXGyroOffset(165);
  mpu.setYGyroOffset(-22);
  mpu.setZGyroOffset(3);

  // Check if DMP init was successful
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("DMP ready!");
  } else {
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
    while (true) {
      delay(10);
    }
  }

  // Start as a USB mouse
  Mouse.begin();
  Serial.println("Setup complete!");
}

// ---------------------------------------------------------------------
void loop() {
  if (!dmpReady) return;

  // Check FIFO count
  fifoCount = mpu.getFIFOCount();
  if (fifoCount < packetSize) {
    return;  // Not enough data yet
  }
  if (fifoCount >= 1024) {
    // FIFO overflow; reset
    mpu.resetFIFO();
    Serial.println("FIFO overflow!");
    return;
  }

  // Read a packet from FIFO
  while (fifoCount >= packetSize) {
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
  }

  // Get updated yaw/pitch/roll
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  float yaw   = ypr[0] * 180.0f / M_PI;
  float pitch = ypr[1] * 180.0f / M_PI;
  float roll  = ypr[2] * 180.0f / M_PI;

  // -------------------------------------------------------------------
  // 1) If locked, check for enough movement to unlock
  // -------------------------------------------------------------------
  if (locked) {
    float diffYaw   = fabs(yaw   - lockYaw);
    float diffPitch = fabs(pitch - lockPitch);

    // If user moves beyond STILL_DEADZONE => unlock
    if (diffYaw > STILL_DEADZONE || diffPitch > STILL_DEADZONE) {
      locked = false;
      Serial.println("Unlocked: resuming mouse movement");

      // Reset references so cursor won't jump
      lastYaw   = yaw;
      lastPitch = pitch;
      accuX     = 0.0f;
      accuY     = 0.0f;

      // Reset stillness
      stillStartTime = 0;
    }

    // Even if locked, handle tilt-based press-and-hold
    handleTiltPress(roll);
    return; // Skip normal cursor movement while locked
  }

  // -------------------------------------------------------------------
  // 2) If not locked, measure stillness for auto-lock
  // -------------------------------------------------------------------
  float diffYaw   = fabs(yaw   - lastYaw);
  float diffPitch = fabs(pitch - lastPitch);

  if (diffYaw < STILL_DEADZONE && diffPitch < STILL_DEADZONE) {
    // Possibly still
    if (stillStartTime == 0) {
      // Just became still
      stillStartTime = millis();
    } else {
      unsigned long elapsed = millis() - stillStartTime;
      if (elapsed >= STILL_THRESHOLD_MS) {
        // Lock now
        locked    = true;
        lockYaw   = yaw;
        lockPitch = pitch;
        Serial.println("Locked: cursor frozen in place.");
        return; // skip normal movement this frame
      }
    }
  } else {
    // Not still
    stillStartTime = 0;
  }

  // -------------------------------------------------------------------
  // 3) Normal mouse movement if not locked
  // -------------------------------------------------------------------
  float realDiffYaw   = yaw   - lastYaw;
  float realDiffPitch = pitch - lastPitch;

  float moveXFloat = realDiffYaw    * sensitivity;
  float moveYFloat = -realDiffPitch * sensitivity; // negative => nod down => cursor down

  // Accumulate fractional parts
  accuX += moveXFloat;
  accuY += moveYFloat;

  // Convert to integer pixels
  int moveX = (int)accuX;
  int moveY = (int)accuY;

  // Remove used integer portion
  accuX -= moveX;
  accuY -= moveY;

  // Move the mouse
  Mouse.move(moveX, moveY, 0);

  // Update last angles
  lastYaw   = yaw;
  lastPitch = pitch;

  // -------------------------------------------------------------------
  // 4) Handle tilt-based button press/hold every loop
  // -------------------------------------------------------------------
  handleTiltPress(roll);
}
