/*
 * VR Haptic Glove Firmware - Improved Version
 * Index Controller compatible, reliable communication
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

// Hardware setup
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

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
  unsigned long timestamp;        // For data validation
};

GloveData data;
unsigned long lastDataSend = 0;
const unsigned long DATA_INTERVAL = 20; // 50Hz data rate

// Finger smoothing
int fingerHistory[5][5]; // Last 5 readings per finger
int fingerIndex = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("VR Haptic Glove Starting...");
  
  // Initialize finger history
  for(int i = 0; i < 5; i++) {
    for(int j = 0; j < 5; j++) {
      fingerHistory[i][j] = 512; // Neutral position
    }
  }
  
  // Initialize IMU with retries
  int attempts = 0;
  while(!bno.begin() && attempts < 5) {
    Serial.println("BNO055 not found, retrying...");
    delay(1000);
    attempts++;
  }
  
  if(attempts >= 5) {
    Serial.println("ERROR: BNO055 failed to initialize!");
    while(1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
  }
  
  // Use IMUPLUS mode (no magnetometer calibration needed)
  bno.setMode(OPERATION_MODE_IMUPLUS);
  Serial.println("BNO055 initialized in IMUPLUS mode");
  
  // Initialize servos
  for(int i = 0; i < 5; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(90);  // Neutral position
    delay(100); // Stagger servo startup
  }
  Serial.println("Servos initialized");
  
  // Brief startup sequence
  for(int pos = 90; pos <= 120; pos += 10) {
    for(int i = 0; i < 5; i++) {
      servos[i].write(pos);
    }
    delay(100);
  }
  for(int pos = 120; pos >= 90; pos -= 10) {
    for(int i = 0; i < 5; i++) {
      servos[i].write(pos);
    }
    delay(100);
  }
  
  delay(1000);
  Serial.println("GLOVE_READY");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read sensors at full rate
  readSensors();
  
  // Send data at controlled rate (50Hz)
  if(currentTime - lastDataSend >= DATA_INTERVAL) {
    sendData();
    lastDataSend = currentTime;
  }
  
  // Handle commands immediately
  handleCommands();
  
  // Small delay to prevent overwhelming the loop
  delay(5);
}

void readSensors() {
  // Read IMU quaternion with error checking
  imu::Quaternion q = bno.getQuat();
  
  // Check for valid quaternion data
  if(!isnan(q.w()) && !isnan(q.x()) && !isnan(q.y()) && !isnan(q.z())) {
    data.qw = q.w();
    data.qx = q.x(); 
    data.qy = q.y();
    data.qz = q.z();
  } else {
    // Keep last valid values or use identity quaternion
    if(data.qw == 0 && data.qx == 0 && data.qy == 0 && data.qz == 0) {
      data.qw = 1.0;
      data.qx = data.qy = data.qz = 0.0;
    }
  }
  
  // Read accelerometer
  sensors_event_t event;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  if(!isnan(event.acceleration.x)) {
    data.ax = event.acceleration.x;
    data.ay = event.acceleration.y; 
    data.az = event.acceleration.z;
  }
  
  // Read finger positions with improved smoothing
  for(int i = 0; i < 5; i++) {
    int raw = analogRead(FINGER_PINS[i]);
    
    // Store in circular buffer
    fingerHistory[i][fingerIndex] = raw;
    
    // Calculate moving average
    long sum = 0;
    for(int j = 0; j < 5; j++) {
      sum += fingerHistory[i][j];
    }
    data.fingers[i] = sum / 5;
    
    // Constrain to valid range
    data.fingers[i] = constrain(data.fingers[i], 0, 1023);
  }
  
  // Update circular buffer index
  fingerIndex = (fingerIndex + 1) % 5;
  
  data.timestamp = millis();
}

void sendData() {
  // Send in format: DATA:qw,qx,qy,qz,ax,ay,az,f0,f1,f2,f3,f4
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
  Serial.println(data.fingers[4]);
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
    
    // Parse comma-separated values
    for(int i = 0; i <= cmd.length() && idx < 5; i++) {
      if(i == cmd.length() || cmd[i] == ',') {
        String valueStr = cmd.substring(start, i);
        pos[idx] = valueStr.toInt();
        pos[idx] = constrain(pos[idx], 0, 180);
        start = i + 1;
        idx++;
      }
    }
    
    // Apply servo positions if we got all 5 values
    if(idx == 5) {
      for(int i = 0; i < 5; i++) {
        servos[i].write(pos[i]);
      }
      Serial.println("HAPTIC_OK");
    } else {
      Serial.println("HAPTIC_ERROR");
    }
  }
  else if(cmd == "STATUS") {
    Serial.print("IMU_OK, Fingers: ");
    for(int i = 0; i < 5; i++) {
      Serial.print(data.fingers[i]);
      if(i < 4) Serial.print(",");
    }
    Serial.print(", Quat: ");
    Serial.print(data.qw, 3); Serial.print(" ");
    Serial.print(data.qx, 3); Serial.print(" ");
    Serial.print(data.qy, 3); Serial.print(" ");
    Serial.println(data.qz, 3);
  }
  else if(cmd == "CALIBRATE") {
    Serial.println("Calibration not needed in IMUPLUS mode");
  }
  else if(cmd == "RESET") {
    Serial.println("Resetting servos...");
    for(int i = 0; i < 5; i++) {
      servos[i].write(90);
    }
    Serial.println("RESET_OK");
  }
  else if(cmd.startsWith("TEST")) {
    // Test command for debugging
    Serial.println("Test mode - cycling servos");
    for(int cycle = 0; cycle < 2; cycle++) {
      for(int pos = 90; pos <= 150; pos += 20) {
        for(int i = 0; i < 5; i++) {
          servos[i].write(pos);
        }
        delay(200);
      }
      for(int pos = 150; pos >= 90; pos -= 20) {
        for(int i = 0; i < 5; i++) {
          servos[i].write(pos);
        }
        delay(200);
      }
    }
    Serial.println("TEST_COMPLETE");
  }
}