
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
Servo servos[5];
const int FINGER_PINS[5] = {A0, A1, A2, A3, A6}; 
const int SERVO_PINS[5] = {3, 4, 5, 6, 7};  

struct GloveData {
  float qw, qx, qy, qz;    
  float ax, ay, az;          
  int fingers[5];                   
};

GloveData data;
unsigned long lastDataSend = 0;
const unsigned long DATA_INTERVAL = 20; //50hz
unsigned long currentTime = 0;

void setup() {
  Serial.begin(19200);
  
  if (!bno.begin()) Serial.println("Bno failed");

  bno.setMode(OPERATION_MODE_IMUPLUS);
  
  for(int i = 0; i < 5; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(90);
    delay(10);
  }
}

void loop() {
  currentTime = millis();

  readSensors();
  
  if(currentTime - lastDataSend >= DATA_INTERVAL) { sendData(); lastDataSend = currentTime; }
  
  handleCommands();
  
  delay(5);
}

void readSensors() {
  imu::Quaternion q = bno.getQuat();
  data.qw = q.w();
  data.qx = q.x(); 
  data.qy = q.y();
  data.qz = q.z();

  sensors_event_t event;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  data.ax = event.acceleration.x;
  data.ay = event.acceleration.y; 
  data.az = event.acceleration.z;

  for(int i = 0; i < 5; i++) { data.fingers[i] = constrain(analogRead(FINGER_PINS[i]), 0, 1023); }
}

void sendData() {
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
  
  String cmd = Serial.readStringUntil('\n').trim();
  
  if(cmd.startsWith("HAPTIC:")) {
    cmd = cmd.substring(7);
    
    int idx = 0;
    int start = 0;
    
    for(int i = 0; i <= cmd.length() && idx < 5; i++) {
      if(i == cmd.length() || cmd[i] == ',') {
        servos[idx].write(constrain(cmd.substring(start, i).toInt(pos[idx]), 0, 180));
        start = i + 1;
        idx++;
      }
    }
  }
  else if(cmd.startsWith("TEST")) {
      for(int pos = 90; pos <= 150; pos += 20) {
        for(int i = 0; i < 5; i++) { servos[i].write(pos); }
        delay(200);
      }
      for(int pos = 150; pos >= 90; pos -= 20) {
        for(int i = 0; i < 5; i++) { servos[i].write(pos); }
        delay(200);
      }
    Serial.println("TEST_COMPLETE");
  }
}