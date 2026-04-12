#include <LSM6DS3.h> 
#include <Wire.h> 
#include <Adafruit_TinyUSB.h> 
#include <MIDI.h> 
 
// ---------------- IMU & USB MIDI ---------------- 

LSM6DS3 imu(I2C_MODE, 0x6A);  
Adafruit_USBD_MIDI usb_midi; 
MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI); 
 
// ---------------- Filter & Tracking ---------------- 

unsigned long lastTime = 0; float pitch = 0, roll = 0, yaw = 0; float pitchBias = 0, rollBias = 0, yawBias = 0; 
 
float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03; float pitchP[2][2] = {{0,0},{0,0}}, rollP[2][2] = {{0,0},{0,0}}; 
 
// ---------------- Yaw Stability Parameters ---------------- 

float gyroDeadband = 0.05; float gyroSpikeLimit = 250.0; float biasCorrectionGain = 0.01; 
 
// Acc magnitude stable window float accStableMin = 0.95; float accStableMax = 1.05; 
 
// ---------------- Drum Settings ---------------- 

float hitThreshold = 2.5;  unsigned long lastHitTime = 0; int debounceTime = 150;  
 
#define KICK   36 
#define SNARE  38 
#define HIHAT  42 
#define TOM    45 
#define CRASH  49 
 
// ============================================================ 
//                     KALMAN FILTER 
// ============================================================ 

float kalmanUpdate(float newAngle, float newRate, float dt,                    float &angle, float &bias, float P[2][2]) 
{   float rate = newRate - bias;   angle += dt * rate; 
 
  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle); 
  P[0][1] -= dt * P[1][1]; 
  P[1][0] -= dt * P[1][1]; 
  P[1][1] += Q_bias * dt; 
 
  float S = P[0][0] + R_measure;   float K[2] = { P[0][0] / S, P[1][0] / S }; 
 
  float y = newAngle - angle;   angle += K[0] * y;   bias  += K[1] * y; 
 
  float P00_temp = P[0][0];   float P01_temp = P[0][1]; 
 
  P[0][0] -= K[0] * P00_temp; 
  P[0][1] -= K[0] * P01_temp; 
  P[1][0] -= K[1] * P00_temp; 
  P[1][1] -= K[1] * P01_temp; 
 
  return angle; 
} 
 
// ============================================================ 
//                          SETUP 
// ============================================================ void setup() 
{ 
  Serial.begin(115200); 
 
  MIDI.begin(MIDI_CHANNEL_OMNI); 
 
  while (!TinyUSBDevice.mounted()) delay(1); 
 
  if (imu.begin() != 0) { 
    Serial.println("IMU Error");     while (1); 
  } 
 
  // Initial gyro bias calibration   for(int i=0; i<500; i++){     yawBias += imu.readFloatGyroZ();     delay(5); 
  } 
  yawBias /= 500.0; 
 
  Serial.println("System Ready!");   lastTime = millis(); 
} 
 

// ============================================================ 
//                          LOOP 
// ============================================================ 

void loop() {   float ax = imu.readFloatAccelX();   float ay = imu.readFloatAccelY();   float az = imu.readFloatAccelZ(); 
 
  float gx = imu.readFloatGyroX();   float gy = imu.readFloatGyroY();   float gzRaw = imu.readFloatGyroZ(); 
 
  unsigned long now = millis();   float dt = (now - lastTime) / 1000.0;   if (dt <= 0) dt = 0.001;   lastTime = now; 
 
  // ---------- Gyro Conditioning ----------   

if (abs(gzRaw) > gyroSpikeLimit)     gzRaw = 0; 
 
  float gz = gzRaw - yawBias; 
 
  if (abs(gz) < gyroDeadband)     gz = 0; 
 
  // ---------- Acceleration Magnitude ----------   

float accelMag = sqrt(ax*ax + ay*ay + az*az);   // Bias correction only when stable   if (accelMag > accStableMin && accelMag < accStableMax) 
  { 
    yawBias += biasCorrectionGain * gz; 
  } 
 
  // ---------- Kalman Pitch/Roll ----------   float accPitch = atan2(ay, az) * 180 / PI;   float accRoll  = atan2(-ax, sqrt(ay*ay + az*az)) * 180 / PI; 
 
  pitch = kalmanUpdate(accPitch, gx, dt, pitch, pitchBias, pitchP);   roll  = kalmanUpdate(accRoll,  gy, dt, roll,  rollBias,  rollP); 
 
  // ---------- Yaw Integration ----------   yaw += gz * dt;   yaw = constrain(yaw, -90, 90); 
 
  // ---------- Hit Detection ----------   if (abs(az) > hitThreshold &&       millis() - lastHitTime > debounceTime) 
  {     lastHitTime = millis(); 
 
    byte velocity = constrain(abs(az) * 40, 40, 127);     int note = HIHAT; 
 
    if (yaw < -54)      note = KICK;     else if (yaw < -18) note = SNARE;     else if (yaw < 18)  note = HIHAT;     else if (yaw < 54)  note = TOM;     else                note = CRASH; 
 
    MIDI.sendNoteOn(note, velocity, 1);     delay(5); 
    MIDI.sendNoteOff(note, 0, 1); 
 
    Serial.print("HIT | Note: "); 
    Serial.print(note); 
    Serial.print(" | Yaw: "); 
    Serial.println(yaw); 
  } 
}  
