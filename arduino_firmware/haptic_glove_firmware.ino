/*
 * VR Haptic Glove Firmware
 * Right hand only, 9600 baud, simple and reliable
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

// Hardware setup
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);  // I2C address 0x29

// Servo objects for haptic feedback
Servo servos[5];

// Pin assignments
const int FINGER_PINS[5] = {A0, A1, A2, A3, A4};  // Thumb, Index, Middle, Ring, Pinky
const int SERVO_PINS[5] = {3, 5, 6, 9, 10};       // PWM pins for servos

// Data structure
struct GloveData {
  float qw, qx, qy, qz;           // Quaternion rotation
  float ax, ay, az;               // Accelerometer 
  int fingers[5];                 // Finger curl values (0-1023)
  bool calibrated;                // IMU calibration status
};

GloveData data;

void setup() {
  Serial.begin(9600);
  Serial.println("Starting VR Haptic Glove...");
  
  // Initialize IMU
  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 not found!");
    while(1) delay(1000);  // Stop here if no IMU
  }
  
  bno.setMode(OPERATION_MODE_NDOF);
  Serial.println("BNO055 initialized");
  
  // Initialize servos
  for(int i = 0; i < 5; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(90);  // Neutral position
  }
  Serial.println("Servos initialized");
  
  delay(1000);
  Serial.println("GLOVE_READY");
}

void loop() {
  readSensors();
  sendData();
  handleCommands();
  delay(20);  // 50Hz update rate
}

void readSensors() {
  // Read IMU quaternion
  imu::Quaternion q = bno.getQuat();
  data.qw = q.w();
  data.qx = q.x(); 
  data.qy = q.y();
  data.qz = q.z();
  
  // Handle NaN values during calibration
  if(isnan(data.qw)) {
    data.qw = 1.0;
    data.qx = data.qy = data.qz = 0.0;
  }
  
  // Read accelerometer
  sensors_event_t event;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  data.ax = event.acceleration.x;
  data.ay = event.acceleration.y; 
  data.az = event.acceleration.z;
  
  // Read finger positions with simple smoothing
  static int lastFingers[5] = {512, 512, 512, 512, 512};
  for(int i = 0; i < 5; i++) {
    int raw = analogRead(FINGER_PINS[i]);
    data.fingers[i] = (lastFingers[i] * 3 + raw) / 4;  // Simple filter
    lastFingers[i] = data.fingers[i];
  }
  
  // Check calibration status
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  data.calibrated = (sys >= 2 && gyro >= 2 && accel >= 2 && mag >= 2);
}

void sendData() {
  // Send in format: DATA:qw,qx,qy,qz,ax,ay,az,f0,f1,f2,f3,f4,cal
  Serial.print("DATA:");
  Serial.print(data.qw, 4); Serial.print(",");
  Serial.print(data.qx, 4); Serial.print(",");
  Serial.print(data.qy, 4); Serial.print(",");
  Serial.print(data.qz, 4); Serial.print(",");
  Serial.print(data.ax, 2); Serial.print(",");
  Serial.print(data.ay, 2); Serial.print(",");
  Serial.print(data.az, 2); Serial.print(",");
  Serial.print(data.fingers[0]); Serial.print(",");
  Serial.print(data.fingers[1]); Serial.print(",");
  Serial.print(data.fingers[2]); Serial.print(",");
  Serial.print(data.fingers[3]); Serial.print(",");
  Serial.print(data.fingers[4]); Serial.print(",");
  Serial.println(data.calibrated ? "1" : "0");
}

void handleCommands() {
  if(!Serial.available()) return;
  
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  
  if(cmd.startsWith("HAPTIC:")) {
    // Parse: HAPTIC:90,120,90,90,90
    cmd = cmd.substring(7);
    
    int pos[5];
    int idx = 0;
    int start = 0;
    
    // Simple comma parsing
    for(int i = 0; i <= cmd.length() && idx < 5; i++) {
      if(i == cmd.length() || cmd[i] == ',') {
        pos[idx] = cmd.substring(start, i).toInt();
        pos[idx] = constrain(pos[idx], 0, 180);
        start = i + 1;
        idx++;
      }
    }
    
    // Apply servo positions
    if(idx == 5) {
      for(int i = 0; i < 5; i++) {
        servos[i].write(pos[i]);
      }
    }
  }
  else if(cmd == "CALIBRATE") {
    // Reset calibration 
    bno.setMode(OPERATION_MODE_CONFIG);
    delay(25);
    bno.setMode(OPERATION_MODE_NDOF);
    Serial.println("Calibration reset - move glove in figure-8 patterns");
  }
  else if(cmd == "STATUS") {
    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);
    Serial.print("CAL:");
    Serial.print(sys); Serial.print(",");
    Serial.print(gyro); Serial.print(",");
    Serial.print(accel); Serial.print(",");
    Serial.println(mag);
  }
}