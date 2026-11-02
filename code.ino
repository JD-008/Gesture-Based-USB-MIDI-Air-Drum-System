/* ============================================================
 *  ES_Special v2 — MEMS Drumstick Firmware
 *  IMU (LSM6DS3) + TLV493D Magnetometer sensor fusion
 *  USB-MIDI drum triggering
 *
 *  Key upgrades over v1:
 *   - Magnetometer added to correct yaw drift (gyro integration
 *     alone has no absolute reference -> unbounded drift)
 *   - Complementary filter fuses gyro (fast, noisy-free short term)
 *     with tilt-compensated magnetic heading (slow, drift-free)
 *   - Data organised into structs (Vector3f, KalmanState,
 *     HeadingData) instead of loose globals
 *   - A union (MidiEvent) packs a MIDI hit into one 32-bit word
 *   - A bitfield struct (SystemStatus) packs 4 boolean flags into
 *     a single byte instead of 4 separate bytes
 *   - enum + PROGMEM-friendly constants instead of #define soup
 *   - Kalman filter refactored into one reusable function that
 *     operates on a KalmanState struct (removes duplicated
 *     pitch/roll arrays and math from v1)
 * ============================================================ */

#include <LSM6DS3.h>
#include <Wire.h>
#include <Adafruit_TinyUSB.h>
#include <MIDI.h>
#include <Tlv493d.h>              // SmartElex TLV493D library

// ---------------- Hardware Interfaces ----------------
LSM6DS3   imu(I2C_MODE, 0x6A);
Tlv493d   mag = Tlv493d();
Adafruit_USBD_MIDI usb_midi;
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI);

// ============================================================
//                      DATA STRUCTURES
// ============================================================

// Generic 3-axis float vector — reused for accel, gyro and mag
// readings instead of 3 separate loose floats each. Also lets us
// pass one struct by reference into functions instead of 3 args.
struct Vector3f {
  float x, y, z;
};
unsigned long noteOffTime = 0;
uint8_t activeNote = 0;
bool noteIsActive = false;
// Everything a single-axis Kalman filter needs to persist between
// loop() calls, bundled together. v1 had pitchP[2][2]/rollP[2][2]
// as separate globals; bundling into a struct means kalmanUpdate()
// can be one function reused for pitch AND roll with zero duplicated
// code, and each axis's state lives in one contiguous block of RAM
// (better cache/locality on an MCU, easier to reset/inspect).
struct KalmanState {
  float angle;
  float bias;
  float P[2][2];
};

// Result of the magnetometer heading calculation, plus a validity
// flag so downstream code never trusts a heading computed from a
// garbage/saturated magnetometer read.
struct HeadingData {
  float heading;   // tilt-compensated heading, degrees
  bool  valid;
};

// System status flags packed into a single byte using C bitfields.
// 4 booleans stored as 4 separate bool globals would cost 4 bytes
// (compiler padding on some MCUs even makes it worse); this costs 1.
// This is a classic embedded interview talking point: bitfields
// trade a small amount of bit-masking cost for a real RAM saving,
// which matters on register-starved MCUs.
struct SystemStatus {
  uint8_t imuReady   : 1;
  uint8_t magReady   : 1;
  uint8_t calibrated : 1;
  uint8_t hitPending : 1;
  uint8_t reserved   : 4;   // pad to a full byte, room for future flags
};

// A MIDI hit event represented as a union: you can either address
// it field-by-field (note/velocity/channel) or as one raw 32-bit
// word. Useful when you want to log/queue/compare a whole hit event
// in a single instruction instead of 3 separate byte comparisons —
// a good example to bring up when asked "when would you use a union
// over a struct?" (answer: when you need two views of the same bytes).
union MidiEvent {
  struct {
    uint8_t note;
    uint8_t velocity;
    uint8_t channel;
    uint8_t padding;   // keeps the struct 4 bytes, matches raw's size
  } fields;
  uint32_t raw;
};

// ============================================================
//                    CONSTANTS / CONFIG
// ============================================================

// enum instead of #define: type-safe, scoped, shows up in a debugger
// by name instead of as a bare integer.
enum DrumNote : uint8_t {
  NOTE_KICK  = 36,
  NOTE_SNARE = 38,
  NOTE_HIHAT = 42,
  NOTE_TOM   = 45,
  NOTE_CRASH = 49
};

// Kalman filter tuning
static const float Q_ANGLE   = 0.001f;
static const float Q_BIAS    = 0.003f;
static const float R_MEASURE = 0.03f;

// Gyro conditioning
static const float GYRO_DEADBAND    = 0.05f;
static const float GYRO_SPIKE_LIMIT = 250.0f;

// Accelerometer "at rest" window (in g) used to gate bias correction
static const float ACC_STABLE_MIN = 0.95f;
static const float ACC_STABLE_MAX = 1.05f;

// Complementary filter weight: how much we trust the gyro vs the
// magnetometer each cycle. Close to 1 = mostly gyro (smooth, drifts).
// Closer to 0 = mostly mag (noisier, but self-correcting).
static const float YAW_FUSION_ALPHA = 0.90f;

// Hit detection
static const float HIT_THRESHOLD   = 1.5f;   // g (includes ~1.0g gravity baseline)
static const uint16_t DEBOUNCE_MS  = 150;

// ============================================================
//                     GLOBAL STATE
// ============================================================

KalmanState pitchState = { 0, 0, {{0,0},{0,0}} };
KalmanState rollState  = { 0, 0, {{0,0},{0,0}} };

float yaw        = 0.0f;   // fused yaw estimate, degrees
float yawBias    = 0.0f;   // gyro Z bias, calibrated at boot

unsigned long lastTime    = 0;
unsigned long lastHitTime = 0;

SystemStatus status = {0, 0, 0, 0, 0};

// ============================================================
//                     KALMAN FILTER
// ============================================================
// Standard 2-state (angle, bias) Kalman filter. Fuses an
// accelerometer-derived angle (accurate long-term, noisy/short-term
// unreliable during motion) with a gyro rate (accurate short-term,
// drifts long-term) to get a stable angle estimate for pitch/roll.
// Passing KalmanState& means one function body serves both axes.
float kalmanUpdate(KalmanState &s, float newAngle, float newRate, float dt) {
  float rate = newRate - s.bias;
  s.angle += dt * rate;

  s.P[0][0] += dt * (dt * s.P[1][1] - s.P[0][1] - s.P[1][0] + Q_ANGLE);
  s.P[0][1] -= dt * s.P[1][1];
  s.P[1][0] -= dt * s.P[1][1];
  s.P[1][1] += Q_BIAS * dt;

  float S    = s.P[0][0] + R_MEASURE;
  float K[2] = { s.P[0][0] / S, s.P[1][0] / S };

  float y = newAngle - s.angle;
  s.angle += K[0] * y;
  s.bias  += K[1] * y;

  float P00 = s.P[0][0];
  float P01 = s.P[0][1];

  s.P[0][0] -= K[0] * P00;
  s.P[0][1] -= K[0] * P01;
  s.P[1][0] -= K[1] * P00;
  s.P[1][1] -= K[1] * P01;

  return s.angle;
}

// ============================================================
//              TILT-COMPENSATED MAGNETIC HEADING
// ============================================================
// Raw atan2(my, mx) only gives a correct heading if the sensor is
// perfectly level. Since the drumstick tilts (that's the whole
// point of tracking pitch/roll), we first rotate the magnetometer
// reading into the horizontal plane using the current pitch/roll
// estimate, THEN take atan2. This is the standard tilt-compensated
// compass algorithm used in every AHRS/IMU+mag fusion stack.
HeadingData computeHeading(const Vector3f &m, float pitchDeg, float rollDeg) {
  HeadingData h = { 0.0f, false };

  float pitchRad = pitchDeg * DEG_TO_RAD;
  float rollRad  = rollDeg  * DEG_TO_RAD;
float magMag = sqrt(m.x * m.x + m.y * m.y + m.z * m.z);
// After hard-iron cal, compensated field ≈ Earth's field (0.02–0.15 mT)
if (magMag < 0.005f || magMag > 0.30f) return h;
  float xh =  m.x * cos(pitchRad) + m.z * sin(pitchRad);
  float yh =  m.x * sin(rollRad) * sin(pitchRad)
            + m.y * cos(rollRad)
            - m.z * sin(rollRad) * cos(pitchRad);

  h.heading = atan2(yh, xh) * RAD_TO_DEG;
  h.valid   = true;
  return h;
}

// ============================================================
//           COMPLEMENTARY FILTER FOR YAW (DRIFT FIX)
// ============================================================
// This is the actual fix for v1's yaw drift problem. Pure gyro
// integration (yaw += gz * dt) accumulates error forever because
// there's no absolute reference — any tiny bias or noise integrates
// into a permanently wrong heading. The magnetometer gives an
// absolute (if noisier) heading, so we blend:
//   new_yaw = alpha * (old_yaw + gyro_delta) + (1 - alpha) * mag_heading
// Gyro dominates on short timescales (smooth, responsive to fast
// hits); the magnetometer term continuously pulls the estimate back
// toward truth, so error can't accumulate unbounded.
float fuseYaw(float currentYaw, float gz, float dt, const HeadingData &h) {
  float gyroYaw = currentYaw + gz * dt;

  if (!h.valid) return gyroYaw;   // no good mag reading -> fall back to gyro only

  // Shortest angular distance handles the -180/+180 wraparound
  float diff = h.heading - gyroYaw;
  while (diff > 180.0f)  diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;

  return gyroYaw + (1.0f - YAW_FUSION_ALPHA) * diff;
}

// ============================================================
//                     NOTE MAPPING
// ============================================================
DrumNote yawToNote(float yawDeg) {
  if (yawDeg < -54) return NOTE_KICK;
  if (yawDeg < -18) return NOTE_SNARE;
  if (yawDeg <  18) return NOTE_HIHAT;
  if (yawDeg <  54) return NOTE_TOM;
  return NOTE_CRASH;
}
Vector3f magOffset = {0, 0, 0};

// Non-blocking magnetometer oversampling state
static Vector3f magAccum    = {0, 0, 0};
static int      magCount    = 0;
static Vector3f magFiltered = {0, 0, 0};

void calibrateMagnetometer() {
  float minX = 1e6, maxX = -1e6;
  float minY = 1e6, maxY = -1e6;
  float minZ = 1e6, maxZ = -1e6;

  Serial.println(F("Rotate the stick through all orientations (figure-8)..."));
  unsigned long start = millis();
  while (millis() - start < 12000) {   // 12 second calibration window
    if (mag.updateData() == TLV493D_NO_ERROR) {
      float x = mag.getX(), y = mag.getY(), z = mag.getZ();
      minX = min(minX, x); maxX = max(maxX, x);
      minY = min(minY, y); maxY = max(maxY, y);
      minZ = min(minZ, z); maxZ = max(maxZ, z);
    }
    delay(5);
  }

  magOffset.x = (maxX + minX) / 2.0f;
  magOffset.y = (maxY + minY) / 2.0f;
  magOffset.z = (maxZ + minZ) / 2.0f;

  Serial.print(F("Mag offset: "));
  Serial.print(magOffset.x, 4); Serial.print(",");
  Serial.print(magOffset.y, 4); Serial.print(",");
  Serial.println(magOffset.z, 4);
}


// ============================================================
//                          SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  MIDI.begin(MIDI_CHANNEL_OMNI);
  while (!TinyUSBDevice.mounted()) delay(1);

  status.imuReady = (imu.begin() == 0);
  if (!status.imuReady) {
    Serial.println(F("IMU Error"));
    while (1);
  }

  mag.begin();
  status.magReady = true;

  // Gyro bias calibration — keep the stick still at boot.
  for (int i = 0; i < 500; i++) {
    yawBias += imu.readFloatGyroZ();
    delay(5);
  }
  yawBias /= 500.0f;
  status.calibrated = true;

  Serial.println(F("System Ready!"));
  lastTime = millis();
  calibrateMagnetometer();
}

// ============================================================
//                          LOOP
// ============================================================
void loop() {
  Vector3f accel = { imu.readFloatAccelX(), imu.readFloatAccelY(), imu.readFloatAccelZ() };
  Vector3f gyro  = { imu.readFloatGyroX(),  imu.readFloatGyroY(),  imu.readFloatGyroZ() };


  // --- Non-blocking mag oversampling: one sample per loop iteration ---
  if (mag.updateData() == TLV493D_NO_ERROR) {
    magAccum.x += mag.getX() - magOffset.x;
    magAccum.y += mag.getY() - magOffset.y;
    magAccum.z += mag.getZ() - magOffset.z;
    magCount++;
    if (magCount >= 16) {
      magFiltered.x = magAccum.x / 16.0f;
      magFiltered.y = magAccum.y / 16.0f;
      magFiltered.z = magAccum.z / 16.0f;
      magAccum = {0, 0, 0};
      magCount = 0;
    }
  }

  Serial.print("magX,Y,Z: "); Serial.print(magFiltered.x, 4);
  Serial.print(","); Serial.print(magFiltered.y, 4);
  Serial.print(","); Serial.println(magFiltered.z, 4);

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  if (dt <= 0) dt = 0.001f;
  lastTime = now;

  // ---------- Gyro conditioning ----------
  if (abs(gyro.z) > GYRO_SPIKE_LIMIT) gyro.z = 0;
  float gz = gyro.z - yawBias;
  if (abs(gz) < GYRO_DEADBAND) gz = 0;

  // ---------- Slow bias re-trim only while stationary ----------
  float accelMag = sqrt(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
  if (accelMag > ACC_STABLE_MIN && accelMag < ACC_STABLE_MAX) {
    yawBias += 0.01f * gz;
  }

  // ---------- Pitch / Roll via Kalman ----------
  float accPitch = atan2(accel.y, accel.z) * RAD_TO_DEG;
  float accRoll  = atan2(-accel.x, sqrt(accel.y * accel.y + accel.z * accel.z)) * RAD_TO_DEG;

  float pitch = kalmanUpdate(pitchState, accPitch, gyro.x, dt);
  float roll  = kalmanUpdate(rollState,  accRoll,  gyro.y, dt);

  // ---------- Yaw: gyro + magnetometer fusion (drift fix) ----------
  HeadingData heading = computeHeading(magFiltered, pitch, roll);
  yaw = fuseYaw(yaw, gz, dt, heading);
  // Modular wrap so yaw continuously cycles -180 → +180 (constrain pins at rail)
  while (yaw >  180.0f) yaw -= 360.0f;
  while (yaw < -180.0f) yaw += 360.0f;
  // Stream yaw data for Arduino Serial Plotter visualization
  Serial.print("Yaw:");
  Serial.println(yaw);
  // ---------- Hit detection ----------
  // Use total acceleration magnitude — catches hits regardless of stick orientation
  float hitMag = sqrt(accel.x*accel.x + accel.y*accel.y + accel.z*accel.z);
  Serial.print("HitMag:"); Serial.println(hitMag);

  if (hitMag > HIT_THRESHOLD && (millis() - lastHitTime) > DEBOUNCE_MS) {
    lastHitTime = millis();

    MidiEvent evt;
    evt.fields.note     = yawToNote(yaw);
    evt.fields.velocity = constrain((uint8_t)(hitMag * 40), 40, 127);
    evt.fields.channel  = 1;
    evt.fields.padding  = 0;
// Send the hit immediately
MIDI.sendNoteOn(evt.fields.note, evt.fields.velocity, evt.fields.channel);

// Schedule the NoteOff for 5ms in the future without blocking the loop
activeNote = evt.fields.note;
noteOffTime = millis() + 5;
noteIsActive = true;
    // evt.raw is available here as a single 32-bit id if you want to
    // log/queue the whole hit event in one write instead of 3 fields.
    Serial.print(F("HIT | Note: "));
    Serial.print(evt.fields.note);
    Serial.print(F(" | Yaw: "));
    Serial.print(yaw);
    Serial.print(F(" | MagValid: "));
    Serial.println(heading.valid);
  }
  if (noteIsActive && millis() >= noteOffTime) {
  MIDI.sendNoteOff(activeNote, 0, 1);
  noteIsActive = false;
}
}
