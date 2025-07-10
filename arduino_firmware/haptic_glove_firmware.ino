#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

// BNO055 IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55,0x29);

// Servo objects for force feedback
Servo thumbServo;
Servo indexServo;
Servo middleServo;
Servo ringServo;
Servo pinkyServo;

// Pin definitions
const int THUMB_POT = A0;
const int INDEX_POT = A1;
const int MIDDLE_POT = A2;
const int RING_POT = A3;
const int PINKY_POT = A4;

const int THUMB_SERVO = 3;
const int INDEX_SERVO = 5;
const int MIDDLE_SERVO = 6;
const int RING_SERVO = 9;
const int PINKY_SERVO = 10;

// Data structure for sending data
struct GloveData {
  float quat_w, quat_x, quat_y, quat_z;  // Quaternion from BNO055
  float accel_x, accel_y, accel_z;       // Accelerometer data
  int thumb_curl, index_curl, middle_curl, ring_curl, pinky_curl;  // Finger positions (0-1023)
  bool calibrated;
} gloveData;

// Force feedback values (0-180 degrees)
int servoPositions[5] = {90, 90, 90, 90, 90};  // Default to middle position

void setup() {
  Serial.begin(9600);
  
  // Initialize BNO055
  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 not detected");
    while (1);
  }
  
  // Set BNO055 to NDOF mode for full sensor fusion
  bno.setMode(OPERATION_MODE_NDOF);
  
  // Attach servos
  thumbServo.attach(THUMB_SERVO);
  indexServo.attach(INDEX_SERVO);
  middleServo.attach(MIDDLE_SERVO);
  ringServo.attach(RING_SERVO);
  pinkyServo.attach(PINKY_SERVO);
  
  // Initialize servos to neutral position
  thumbServo.write(90);
  indexServo.write(90);
  middleServo.write(90);
  ringServo.write(90);
  pinkyServo.write(90);
  
  delay(1000);
  Serial.println("GLOVE_READY");
}

void loop() {
  // Read IMU data
  sensors_event_t event;
  bno.getEvent(&event);
  
  // Get quaternion data
  imu::Quaternion quat = bno.getQuat();
  gloveData.quat_w = quat.w();
  gloveData.quat_x = quat.x();
  gloveData.quat_y = quat.y();
  gloveData.quat_z = quat.z();
  
  // Get accelerometer data
  sensors_event_t accelEvent;
  bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  gloveData.accel_x = accelEvent.acceleration.x;
  gloveData.accel_y = accelEvent.acceleration.y;
  gloveData.accel_z = accelEvent.acceleration.z;
  
  // Read finger curl potentiometers
  gloveData.thumb_curl = analogRead(THUMB_POT);
  gloveData.index_curl = analogRead(INDEX_POT);
  gloveData.middle_curl = analogRead(MIDDLE_POT);
  gloveData.ring_curl = analogRead(RING_POT);
  gloveData.pinky_curl = analogRead(PINKY_POT);
  
  // Check calibration status
  uint8_t sys, gyro, accel, mag;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  gloveData.calibrated = (sys >= 2 && gyro >= 2 && accel >= 2 && mag >= 2);
  
  // Send data packet
  Serial.print("DATA:");
  Serial.print(gloveData.quat_w, 4); Serial.print(",");
  Serial.print(gloveData.quat_x, 4); Serial.print(",");
  Serial.print(gloveData.quat_y, 4); Serial.print(",");
  Serial.print(gloveData.quat_z, 4); Serial.print(",");
  Serial.print(gloveData.accel_x, 2); Serial.print(",");
  Serial.print(gloveData.accel_y, 2); Serial.print(",");
  Serial.print(gloveData.accel_z, 2); Serial.print(",");
  Serial.print(gloveData.thumb_curl); Serial.print(",");
  Serial.print(gloveData.index_curl); Serial.print(",");
  Serial.print(gloveData.middle_curl); Serial.print(",");
  Serial.print(gloveData.ring_curl); Serial.print(",");
  Serial.print(gloveData.pinky_curl); Serial.print(",");
  Serial.print(gloveData.calibrated ? "1" : "0");
  Serial.println();
  
  // Check for incoming haptic feedback commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("HAPTIC:")) {
      // Parse haptic command: HAPTIC:thumb,index,middle,ring,pinky
      command = command.substring(7);  // Remove "HAPTIC:"
      
      int values[5];
      int valueIndex = 0;
      int startIndex = 0;
      
      for (int i = 0; i <= command.length() && valueIndex < 5; i++) {
        if (i == command.length() || command[i] == ',') {
          values[valueIndex] = command.substring(startIndex, i).toInt();
          startIndex = i + 1;
          valueIndex++;
        }
      }
      
      // Apply servo positions (constrain to 0-180 range)
      thumbServo.write(constrain(values[0], 0, 180));
      indexServo.write(constrain(values[1], 0, 180));
      middleServo.write(constrain(values[2], 0, 180));
      ringServo.write(constrain(values[3], 0, 180));
      pinkyServo.write(constrain(values[4], 0, 180));
    }
  }
  
  delay(20);  // 50Hz update rate
}