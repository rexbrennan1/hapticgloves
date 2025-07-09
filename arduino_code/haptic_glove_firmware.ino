#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

// BNO055 IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55);

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

// Data structure for sending data to PC
struct GloveData {
  float quat_w, quat_x, quat_y, quat_z;  // Quaternion from BNO055
  float accel_x, accel_y, accel_z;       // Accelerometer data
  float gyro_x, gyro_y, gyro_z;          // Gyroscope data
  uint16_t finger_curl[5];               // Finger curl values (0-1023)
  uint8_t checksum;
};

// Data structure for receiving force feedback commands
struct ForceData {
  uint8_t forces[5];  // Force values for each finger (0-255)
  uint8_t checksum;
};

GloveData gloveData;
ForceData forceData;

void setup() {
  Serial.begin(115200);
  
  // Initialize BNO055
  if (!bno.begin()) {
    Serial.println("BNO055 not detected");
    while (1);
  }
  
  // Use external crystal for better accuracy
  bno.setExtCrystalUse(true);
  
  // Initialize servos
  thumbServo.attach(THUMB_SERVO);
  indexServo.attach(INDEX_SERVO);
  middleServo.attach(MIDDLE_SERVO);
  ringServo.attach(RING_SERVO);
  pinkyServo.attach(PINKY_SERVO);
  
  // Set servos to neutral position
  thumbServo.write(90);
  indexServo.write(90);
  middleServo.write(90);
  ringServo.write(90);
  pinkyServo.write(90);
  
  delay(1000);
}

void loop() {
  // Read IMU data
  sensors_event_t event;
  bno.getEvent(&event);
  
  // Get quaternion
  imu::Quaternion quat = bno.getQuat();
  gloveData.quat_w = quat.w();
  gloveData.quat_x = quat.x();
  gloveData.quat_y = quat.y();
  gloveData.quat_z = quat.z();
  
  // Get accelerometer data
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  gloveData.accel_x = accel.x();
  gloveData.accel_y = accel.y();
  gloveData.accel_z = accel.z();
  
  // Get gyroscope data
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  gloveData.gyro_x = gyro.x();
  gloveData.gyro_y = gyro.y();
  gloveData.gyro_z = gyro.z();
  
  // Read finger curl potentiometers
  gloveData.finger_curl[0] = analogRead(THUMB_POT);
  gloveData.finger_curl[1] = analogRead(INDEX_POT);
  gloveData.finger_curl[2] = analogRead(MIDDLE_POT);
  gloveData.finger_curl[3] = analogRead(RING_POT);
  gloveData.finger_curl[4] = analogRead(PINKY_POT);
  
  // Calculate checksum
  gloveData.checksum = calculateChecksum((uint8_t*)&gloveData, sizeof(gloveData) - 1);
  
  // Send data to PC
  Serial.write((uint8_t*)&gloveData, sizeof(gloveData));
  
  // Check for incoming force feedback data
  if (Serial.available() >= sizeof(ForceData)) {
    Serial.readBytes((uint8_t*)&forceData, sizeof(ForceData));
    
    // Verify checksum
    uint8_t calculatedChecksum = calculateChecksum((uint8_t*)&forceData, sizeof(ForceData) - 1);
    if (calculatedChecksum == forceData.checksum) {
      // Apply force feedback
      applyForces();
    }
  }
  
  delay(10); // ~100Hz update rate
}

uint8_t calculateChecksum(uint8_t* data, int length) {
  uint8_t sum = 0;
  for (int i = 0; i < length; i++) {
    sum += data[i];
  }
  return sum;
}

void applyForces() {
  // Map force values (0-255) to servo positions
  // Adjust these mappings based on your servo setup
  int thumbPos = map(forceData.forces[0], 0, 255, 70, 110);
  int indexPos = map(forceData.forces[1], 0, 255, 70, 110);
  int middlePos = map(forceData.forces[2], 0, 255, 70, 110);
  int ringPos = map(forceData.forces[3], 0, 255, 70, 110);
  int pinkyPos = map(forceData.forces[4], 0, 255, 70, 110);
  
  thumbServo.write(thumbPos);
  indexServo.write(indexPos);
  middleServo.write(middlePos);
  ringServo.write(ringPos);
  pinkyServo.write(pinkyPos);
}
