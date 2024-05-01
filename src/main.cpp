#include <Arduino.h>
#include <QTRSensors.h>
#include <NewPing.h>
#include <esp_now.h>
#include <WiFi.h>

// put function declarations here:
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
int calculateError(uint16_t *sensorValues);
void adjustMotorSpeeds(int error);
void setMotorSpeed(int pwmPin, int dirPin, int speed);
void stopAllMotors();
bool shouldStop(uint16_t *sensorValues);
void rotateRight();

// uint8_t broadcastAddress[] = {0x08, 0xD1, 0xF9, 0xA3, 0xED, 0x68};
uint8_t broadcastAddress[] = {0x40, 0x91, 0x51, 0xFD, 0x54, 0xC4};

typedef struct struct_message {
    char msg[50];
} struct_message;

struct_message outgoingReadings;
struct_message incomingReadings;
String success;

bool lineLost = true;
bool wasLineDetected = true;
bool roverReached = false;
bool bothRoversReady = false;

// Pin assignments for motor control
const int motor1ULeftPWM = 25;  
const int motor1ULeftDir = 33; 
const int motor2URightPWM = 18;
const int motor2URightDir = 19;
const int motor3LLeftPWM = 26;
const int motor3LLeftDir = 27;
const int motor4LRightPWM = 2;
const int motor4LRightDir = 4;

// PWM Channels for ESP32
const int motor1ULeftPWMChannel = 0;
const int motor2URightPWMChannel = 1;
const int motor3LLeftPWMChannel = 2;
const int motor4LRightPWMChannel = 3;

QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
int baseSpeed = 5; // Base motor speed

bool roverStopped = false; // Flag to track rover status

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0) {
    success = "Delivery Success :)";
  }
  else {
    success = "Delivery Fail :(";
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len)
{
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Data: ");Serial.println(incomingReadings.msg);

  if ((String(incomingReadings.msg) == "Stop")){
    roverReached = true;
    strcpy(outgoingReadings.msg, "ACK");
    esp_now_send(broadcastAddress, (uint8_t *)&outgoingReadings, sizeof(outgoingReadings));
  }else if (String(incomingReadings.msg) == "ACK") {
    // Set another flag to indicate that both rovers are ready to rotate
    bothRoversReady = true;
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  for (int ii = 0; ii < 6; ++ii )
  {
    peerInfo.peer_addr[ii] = (uint8_t) broadcastAddress[ii];
  }
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);
  // Initialize sensor and motor pins
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){32,14,12,23,22,21,5,15}, SensorCount);
  
  pinMode(motor1ULeftPWM, OUTPUT);
  pinMode(motor1ULeftDir, OUTPUT);
  pinMode(motor2URightPWM, OUTPUT);
  pinMode(motor2URightDir, OUTPUT);
  pinMode(motor3LLeftPWM, OUTPUT);
  pinMode(motor3LLeftDir, OUTPUT);
  pinMode(motor4LRightPWM, OUTPUT);
  pinMode(motor4LRightDir, OUTPUT);

  // PWM setup for motors
  ledcSetup(motor1ULeftPWMChannel, 5000, 8); // 5 kHz PWM, 8-bit resolution
  ledcSetup(motor2URightPWMChannel, 5000, 8);
  ledcSetup(motor3LLeftPWMChannel, 5000, 8);
  ledcSetup(motor4LRightPWMChannel, 5000, 8);

  ledcAttachPin(motor1ULeftPWM, motor1ULeftPWMChannel);
  ledcAttachPin(motor2URightPWM, motor2URightPWMChannel);
  ledcAttachPin(motor3LLeftPWM, motor3LLeftPWMChannel);
  ledcAttachPin(motor4LRightPWM, motor4LRightPWMChannel);

  // Calibration process for sensors
  Serial.begin(115200);
  for (int i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(20);
  }
  Serial.println("Calibration complete");
}

int allSensorsMaxCounter = 0; // Counter for occurrences

void loop() {
  uint16_t position = qtr.readLineWhite(sensorValues);
  bool allSensorsMax = true;
  if (shouldStop(sensorValues)) {
    if (!roverStopped) { // Only stop the rover if it's currently moving
      stopAllMotors();
      wasLineDetected = false; // Update the flag
    }
  } else {
    roverStopped = false; // Reset the flag when any sensor does not read 1000
    // Proceed with error calculation and motor speed adjustment
    int error = calculateError(sensorValues);
    adjustMotorSpeeds(error);
  }

  // Debugging: print sensor values and position
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
    if (sensorValues[i] != 1000) { // Check if any sensor does not read 1000
      allSensorsMax = false;
    }
  }
  Serial.println(position);

  if (allSensorsMax && (position == 0 || position == 7000)) {
    allSensorsMaxCounter++; // Increment the counter if condition is met
    Serial.println(allSensorsMaxCounter);
  } else {
    allSensorsMaxCounter = 0; // Reset the counter if condition is not met
  }

  // Check if the condition has been met for 50 occurrences
  if (allSensorsMaxCounter >= 50) {
    // Perform action here: send a message or any other action
    strcpy(outgoingReadings.msg, "Stop");
    esp_err_t result2 = esp_now_send(broadcastAddress, (uint8_t *) &outgoingReadings, sizeof(outgoingReadings));
    while(!roverReached){
      delay(100);
    }
    while (!bothRoversReady) {
      delay(100); // Wait for the ACK from the other rover
    }
    rotateRight();
    roverReached = false;
    bothRoversReady = false;
    allSensorsMaxCounter = 0; // Reset the counter after sending the message
  }

  delay(100); // Adjust delay as needed
}

int calculateError(uint16_t *sensorValues) {
  int leftCount = 0, rightCount = 0;
  for (int i = 0; i < SensorCount/2; i++) {
    if (sensorValues[i] < 1000) leftCount++;
  }
  for (int i = SensorCount/2; i < SensorCount; i++) {
    if (sensorValues[i] < 1000) rightCount++;
  }
  return leftCount - rightCount;
}

void adjustMotorSpeeds(int error) {

  // Calculate base adjustment from error, ensuring responsiveness
  int adjustment = abs(error) * 20; // Adjust this multiplier to tune responsiveness

  // Calculate motor speeds based on error direction
  int leftMotorSpeed = baseSpeed;
  int rightMotorSpeed = baseSpeed;

  // Adjust speeds based on error direction
  if (error < 0) { // Line is to the right
    leftMotorSpeed += adjustment;
    rightMotorSpeed -= adjustment;
  } else if (error > 0) { // Line is to the left
    leftMotorSpeed -= adjustment;
    rightMotorSpeed += adjustment;
  }
  
  // Ensure motor speeds are within valid range
  leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
  rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

  // Apply the speeds
  setMotorSpeed(motor1ULeftPWM, motor1ULeftDir, leftMotorSpeed);
  setMotorSpeed(motor2URightPWM, motor2URightDir, rightMotorSpeed);
  setMotorSpeed(motor3LLeftPWM, motor3LLeftDir, leftMotorSpeed);
  setMotorSpeed(motor4LRightPWM, motor4LRightDir, rightMotorSpeed);
}

void setMotorSpeed(int pwmPin, int dirPin, int speed) {
  bool direction = speed >= 0;
  digitalWrite(dirPin, direction ? HIGH : LOW);
  
  // Determine the appropriate PWM channel based on the PWM pin
  int channel;
  if(pwmPin == motor1ULeftPWM) channel = motor1ULeftPWMChannel;
  else if(pwmPin == motor2URightPWM) channel = motor2URightPWMChannel;
  else if(pwmPin == motor3LLeftPWM) channel = motor3LLeftPWMChannel;
  else if(pwmPin == motor4LRightPWM) channel = motor4LRightPWMChannel;
  else return; // Safety check

  // Convert speed (0-255) to PWM duty cycle (0-255 for 8-bit resolution)
  uint8_t dutyCycle = abs(speed) * 255 / 255;

  // Write to the appropriate PWM channel
  ledcWrite(channel, dutyCycle);
}

void stopAllMotors() {
  // Use ledcWrite to set PWM duty cycle to 0, stopping the motors
  ledcWrite(motor1ULeftPWMChannel, 0);
  ledcWrite(motor2URightPWMChannel, 0);
  ledcWrite(motor3LLeftPWMChannel, 0);
  ledcWrite(motor4LRightPWMChannel, 0);

  // It's not strictly necessary to change the direction pins when stopping the motors, but reset if needed
  digitalWrite(motor1ULeftDir, LOW);
  digitalWrite(motor2URightDir, LOW);
  digitalWrite(motor3LLeftDir, LOW);
  digitalWrite(motor4LRightDir, LOW);

  roverStopped = true; // Flag the rover as stopped
  Serial.println("All motors stopped.");
}

bool shouldStop(uint16_t *sensorValues) {
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] < 1000) { // If any sensor does not read 1000
      return false; // do not stop
    }
  }
  return true; // All sensors read 1000, so stop
}

void rotateRight(){
  Serial.println("Turning Right");
  setMotorSpeed(motor1ULeftPWM, motor1ULeftDir, 50);
  //setMotorSpeed(motor2URightPWM, motor2URightDir, 0);
  setMotorSpeed(motor3LLeftPWM, motor3LLeftDir, 50);
  //setMotorSpeed(motor4LRightPWM, motor4LRightDir, 0);
  delay(3000);
  setMotorSpeed(motor1ULeftPWM, motor1ULeftDir, 50);
  setMotorSpeed(motor2URightPWM, motor2URightDir, 50);
  setMotorSpeed(motor3LLeftPWM, motor3LLeftDir, 50);
  setMotorSpeed(motor4LRightPWM, motor4LRightDir, 50);

  delay(2000);

  stopAllMotors();
  while(true){
    delay(1000);
  }

}