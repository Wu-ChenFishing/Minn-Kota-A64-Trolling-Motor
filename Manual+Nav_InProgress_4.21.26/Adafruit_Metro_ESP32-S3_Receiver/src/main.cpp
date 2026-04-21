//==========================
//1.  Includes & Defines
//==========================

//Libaries
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_BNO055.h>

// Define Remote Control MAC address
uint8_t broadcastAddress[] = {0xb8, 0xf8, 0x62, 0xd6, 0x5b, 0xd8};

#define Motor_Power_Pin 3
#define Motor_Left_Pin 5
#define Motor_Right_Pin 8

//X9C103P Digipot
#define Digipot_CS_Pin 9
#define Digipot_Inc_Pin 11
#define Digipot_UD_Pin 12

//Define Control Modes
#define mode_manual 0 //Manual control
#define mode_nav 1 //hold heading
#define mode_anchor 2 //spotlock

//==========================
//2.  Types & Structures
//==========================
// Structure example to receive data
// Must match the controller structure
typedef struct {
  uint8_t mode;
  bool powerToggle;
  bool Up;
  bool Down;
  bool left;
  bool right;
  bool sleep;
} ControlData;

// Structure to send data to remote control
// Must match controller structure
typedef struct {
  uint8_t mode;
  int speed = 0;
  int compass_boat = 0;         // direction that the boat is facing according to boat compass
  int compass_motor = 0;        // direction that the motor is facing according to the motor compass
  int nav_heading = 0;          // Set heading from boat compass when nav button is pushed
  int nav_dist_heading = 0;     //Distance to heading in degrees based from nav_heading
} ReceiverData;


//=========================
//BNO055 uses I2C interface
//=========================
//Green wire -> SCL
//White wire -> SDA
//Red wire -> 3.3V
//Black wire -> GND
Adafruit_BNO055 boat_dir = Adafruit_BNO055(55, 0x28);   // create boat compass object
Adafruit_BNO055 motor_dir = Adafruit_BNO055(55, 0x29);  // create motor compass object

//========================
//BNO055 Read Calibration
//========================
uint8_t sys, gyro, accel, mag;


//=========================
//3.  Global Variables
//=========================
//Timer for stepping speed
static bool manualmode_locked = false;
static bool navmode_locked = false;
static bool anchormode_locked = false;

unsigned long lastSpeedStep = 0;
const unsigned long speedInterval = 100;
bool doSpeedUp = false;
bool doSpeedDown = false;
bool newDataReceived = false;


String success;
bool sendError = false;  //ESP-NOW send error

float compass_boat = 0;         // direction that the boat is facing according to boat compass
float compass_motor = 0;        // direction that the motor is facing according to the motor compass
int boat_motor_angle = 0;     // angle between motor and boat
float set_nav_heading = 0;    //set nav heading

float deg_to_rad = 0.01745;       // for converting degrees to radians
int motor_boat_angle = 0;         // angle difference between the motor and the boat

double lastSendUpdate = millis();

//NAV mode globals
unsigned long motorPulseStart = 0;
unsigned long lastPulseEnd = 0;
const unsigned long pulseCooldown = 150; //Tune this 
static int directionConfirm = 0;
bool motorTurnActive = false;  //for NAV mode
int motorDirection = 0; //for NAV mode
int motorPulseDuration = 0; //for NAV mode

//===========
uint8_t currentMode = mode_manual; //start in manual mode

//Fail-safe timeout
unsigned long lastPacketTime = 0;
const unsigned long failSafeTime = 300000; //5 minutes, might need to change failsafe for nav/anchor unless we add in signal sending to make sure remote is alive

int locTap = 0; // This variable will track the current position of the digipot

//Last state tracking
ControlData lastState = {0, false,false, false, false, false, false};

ControlData rxData;  //Stucture for received data from remote control

ReceiverData txData;  //Structure to send data to remote control

esp_now_peer_info_t peerInfo;

void send_data(){
  esp_now_send(broadcastAddress, (uint8_t*)&txData, sizeof(txData));
  //Serial.println(txData.compass_boat);
  Serial.println("Sending");
}

void motorfulloff() {
//Set speed to fully off - safety
  for (locTap=0; locTap<100; locTap++)
    {
      digitalWrite(Digipot_CS_Pin, LOW);  //select Pot
      digitalWrite(Digipot_UD_Pin, LOW); //Decrease speed
      delayMicroseconds(20);  //Wait for IC Stablize
      digitalWrite(Digipot_Inc_Pin, LOW); //Need to trigger Low to High to step
      delayMicroseconds(20);
      digitalWrite(Digipot_Inc_Pin, HIGH);
      //Serial.println(locTap);
    }
    digitalWrite(Digipot_CS_Pin, HIGH); //Deselect pot
    locTap = 0; //Reset locTap to 0 for tracking
}
//=========================
//4.  Callback Functions
//=========================
// Callback when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&rxData, incomingData, sizeof(rxData));
  newDataReceived = true; // Set flag to indicate new data has been received

  lastPacketTime = millis(); //For failsafe
  }

  /*
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  //  Serial.print("\r\nLast Packet Send Status:\t");
  //  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0) {
    success = "Delivery Success :)";
  }
  else {
    success = "Delivery Fail :(";
  }
}
*/


//==========================
//4.  Function Prototypes
//==========================
void handleMode();
void handleManualMode();
void handleNavMode();
void handleAnchorMode();
void stepDigipotUp();
void stepDigipotDown();
void motorfullloff();
void compass_output();
void compass_calibration();
void send_data_to_controller();

//==========================
//5.  Setup Program & Main Loop
//==========================
void setup() {
  Serial.begin(115200);

  // Set pins as outputs
  pinMode(Motor_Power_Pin, OUTPUT);
  pinMode(Motor_Left_Pin, OUTPUT);
  pinMode(Motor_Right_Pin, OUTPUT);

  //X9C103P
  pinMode (Digipot_CS_Pin, OUTPUT);
  digitalWrite(Digipot_CS_Pin, HIGH); //Probably not needed but deselect before any changing other pins; deselect pot
  pinMode (Digipot_UD_Pin, OUTPUT);
  pinMode (Digipot_Inc_Pin, OUTPUT);
  
  // Set all outputs LOW initially
  digitalWrite(Motor_Power_Pin, LOW);
  digitalWrite(Motor_Left_Pin, LOW);
  digitalWrite(Motor_Right_Pin, LOW);

//Start BNO055
if (!boat_dir.begin()){
  Serial.println("Boat IMU failed!");
  while (1);
};

/*
if (!motor_dir.begin()){
  Serial.println("Motor IMU failed!");
  while (1);
};
*/

delay(1000);
boat_dir.setExtCrystalUse(true);
boat_dir.setMode(OPERATION_MODE_NDOF);

//Start motor off
motorfulloff();
  
    // Initialize WiFi
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }


  // Register send callback
  //esp_now_register_send_cb(OnDataSent);

  //Register Peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);
  
  rxData.powerToggle = 0; //make sure motor is off

  Serial.println("Receiver Ready");
}

void loop() {
  unsigned long now = millis();
  
  //Calibrate Check
 /* boat_dir.getCalibration(&sys, &gyro, &accel, &mag);
  if (mag<2){
    Serial.println("Compass not calibrated!");
    compass_calibration();
  }
*/
compass_output();
  
  //Only send data every 2.5 seconds this is only for testing
  if (millis() - lastSendUpdate > 250) { // Send data to remote control every 2.5 seconds
    txData.mode = currentMode;
    txData.nav_heading = set_nav_heading;
    txData.speed = locTap;
    send_data();
    lastSendUpdate = millis();
    //Serial.print("Mode: "); Serial.println(txData.mode);
    //Serial.print("Boat :"); Serial.println(txData.compass_boat);
    //Serial.print("Motor: "); Serial.println(txData.compass_motor);
    Serial.println("Sending Updated Data");
    }


  if(newDataReceived){
    currentMode = rxData.mode;
    handleMode();
    newDataReceived = false; // Reset flag after handling new data
  }

  switch(currentMode)
 {

    case mode_manual:
    handleManualMode();
    break;

    case mode_nav:
    handleNavMode();
    break;
    //Hold heading, so only update speed and power 

    case mode_anchor:
    handleAnchorMode();
    break;
}

// Speed Up (HOLD behavior)
if (doSpeedUp && (now - lastSpeedStep > speedInterval) && locTap < 100) {
    stepDigipotUp();
    locTap++;
    lastSpeedStep = now;

    Serial.print("Speed Up, locTap: ");
    Serial.println(locTap);
}

// Speed Down (HOLD behavior)
if (doSpeedDown && (now - lastSpeedStep > speedInterval) && locTap > 0) {
    stepDigipotDown();
    locTap--;
    lastSpeedStep = now;

    Serial.print("Speed Down, locTap: ");
    Serial.println(locTap);
}

//if no information received for  turn everything off
if (millis() - lastPacketTime > failSafeTime){
  digitalWrite(Motor_Power_Pin, LOW);
  digitalWrite(Motor_Left_Pin, LOW);
  digitalWrite(Motor_Right_Pin, LOW);
}



//delay(5000);
}

//==========================
//6.  Helper Functions
//==========================
void handleMode(){
  switch (rxData.mode){

    case mode_manual:
    navmode_locked = false;
    anchormode_locked = false;
    break;


    case mode_nav:
    manualmode_locked = false;
    anchormode_locked = false;
    break;
    //Hold heading, so only update speed and power 

    case mode_anchor:
    manualmode_locked = false;
    navmode_locked = false;
    break;
    //Hold position, so only update steering
 } 
}

void handleManualMode(){
  if (!manualmode_locked){
    manualmode_locked = true;
  }

  // --- Motor power
  if (rxData.powerToggle != lastState.powerToggle) {
    //This is ternary operator for if/else, if received.powerToggle is true, it will set HIGH, otherwise LOW
    digitalWrite(Motor_Power_Pin, rxData.powerToggle ? HIGH : LOW);
    lastState.powerToggle = rxData.powerToggle;
    Serial.print("Motor Power: "); Serial.println(rxData.powerToggle);
  }
 
  //Speed Section
  if ((rxData.Up && rxData.Down) || (rxData.powerToggle == LOW)) {
  doSpeedUp = false;
  doSpeedDown = false;
} else {
  doSpeedUp = rxData.Up;
  doSpeedDown = rxData.Down;
}

  //Turning Section
if (rxData.left && rxData.right) {
  digitalWrite(Motor_Left_Pin, LOW);
  digitalWrite(Motor_Right_Pin, LOW);
} else {
  digitalWrite(Motor_Left_Pin, rxData.left ? HIGH : LOW);
  digitalWrite(Motor_Right_Pin, rxData.right ? HIGH : LOW);
}
}

void handleNavMode(){
    //4.18 placeholder for nav mode
    //Will only use (2) BNO055 for heading hold
    //When GPS is added we can add more functionality
    //1st BNO will set heading, 2nd BNO can rotate from that heading and do not rotate past 90 degrees in each direction
    //Testing with only (1) BNO055 and doing easy adjusting
    //Looks like newer motors only use 1 compass and relies on GPS as well.  Adding (2) should improve this function
//Serial.println("In Nav Mode");

if (!navmode_locked){
  set_nav_heading = compass_boat;  // Set heading from boat compass when nav button is pushed
  navmode_locked = true;
  //Serial.print("Nav Locked Headin: "); Serial.println(set_nav_heading);
}

  // --- Motor power
  if (rxData.powerToggle != lastState.powerToggle) {
    //This is ternary operator for if/else, if received.powerToggle is true, it will set HIGH, otherwise LOW
    digitalWrite(Motor_Power_Pin, rxData.powerToggle ? HIGH : LOW);
    lastState.powerToggle = rxData.powerToggle;
    Serial.print("Motor Power: "); Serial.println(rxData.powerToggle);
  }

//Heading error calacuation
float heading_error = set_nav_heading - compass_boat;
//Serial.print("Locked Heading: "); Serial.println(set_nav_heading);
//Serial.print("Current Heading: "); Serial.println(compass_boat);
//Serial.print("Heading Error: "); Serial.println(heading_error);

if (heading_error > 180) heading_error -= 360;
if (heading_error < -180) heading_error += 360;
float abs_error = abs(heading_error);

int pulse_time = 0;
int newDir = (heading_error > 0) ? 1 : -1;

if (!motorTurnActive && abs_error > 10 && (millis() - lastPulseEnd > pulseCooldown)) {
  if (abs_error <15)
  {
  pulse_time = 60;
  }
  else{
  pulse_time = map(abs_error,15,60,80,300);
  }
    //motorPulseDuration = map(abs_error, 5, 60, 80, 300);
    //motorPulseDuration = constrain(motorPulseDuration, 80, 300);

//Direction stability filter
    if (newDir == directionConfirm){
      motorDirection = newDir;
    }
    else{
      directionConfirm = newDir;
      motorDirection = directionConfirm;
    }

    motorPulseStart = millis();
    motorTurnActive = true;

    if (motorDirection == 1) {
        digitalWrite(Motor_Left_Pin, HIGH);
    } else {
        digitalWrite(Motor_Right_Pin, HIGH);
    }
}

if (motorTurnActive) {
  if (millis() - motorPulseStart >= pulse_time){
    digitalWrite(Motor_Left_Pin, LOW);
    digitalWrite(Motor_Right_Pin, LOW);
    motorTurnActive = false;
    lastPulseEnd = millis();
  }
} 

//Speed Section
  if ((rxData.Up && rxData.Down) || (rxData.powerToggle == LOW)) {
  doSpeedUp = false;
  doSpeedDown = false;
} else {
  doSpeedUp = rxData.Up;
  doSpeedDown = rxData.Down;
}
}

void handleAnchorMode(){
    //placeholder for anchor mode
    if (!anchormode_locked){
    anchormode_locked = true;
  }
}

void stepDigipotUp() {
  digitalWrite(Digipot_CS_Pin, LOW);
  digitalWrite(Digipot_UD_Pin, HIGH);
  digitalWrite(Digipot_Inc_Pin, LOW);
  delayMicroseconds(20);
  digitalWrite(Digipot_Inc_Pin, HIGH);
  digitalWrite(Digipot_CS_Pin, HIGH);
}

void stepDigipotDown() {
  digitalWrite(Digipot_CS_Pin, LOW);
  digitalWrite(Digipot_UD_Pin, LOW);
  digitalWrite(Digipot_Inc_Pin, LOW);
  delayMicroseconds(20);
  digitalWrite(Digipot_Inc_Pin, HIGH);
  digitalWrite(Digipot_CS_Pin, HIGH);
}



void compass_output(){
 // get data from digital compasses
  sensors_event_t motor_mag_data, motor_grav_data, boat_mag_data, boat_grav_data;

  //boat_dir.getEvent(&boat_mag_data, Adafruit_BNO055::VECTOR_MAGNETOMETER);    // boat magnetometer data
 //motor_dir.getEvent(&motor_mag_data, Adafruit_BNO055::VECTOR_MAGNETOMETER);  // motor magnetometer data
/*
  boat_dir.getEvent(&boat_grav_data, Adafruit_BNO055::VECTOR_GRAVITY);        // boat gravity data
  motor_dir.getEvent(&motor_grav_data, Adafruit_BNO055::VECTOR_GRAVITY);      // motor gravity data

  float boat_grav_x = boat_grav_data.acceleration.x;
  float boat_grav_y = boat_grav_data.acceleration.y;
  float boat_grav_z = boat_grav_data.acceleration.z;

  float motor_grav_x = motor_grav_data.acceleration.x;
  float motor_grav_y = motor_grav_data.acceleration.y;
  float motor_grav_z = motor_grav_data.acceleration.z;

  // get boat pitch and yaw for tilt compensation
  float boat_roll = atan2(boat_grav_y, boat_grav_z);
  float boat_pitch = atan2(boat_grav_x, boat_grav_z * cos(boat_roll) + boat_grav_y * sin(boat_roll));

  // get motor pitch and yaw for tilt compensation
  float motor_roll = atan2(motor_grav_y, motor_grav_z);                                                       // phi
  float motor_pitch = atan2(motor_grav_x, motor_grav_z * cos(motor_roll) + motor_grav_y * sin(motor_roll));   // theta

  // calculate boat magnetometer values with tilt compensation
  float boat_mag_z = boat_mag_data.magnetic.z;
  float boat_mag_y = boat_mag_z * sin(boat_roll) - boat_mag_data.magnetic.y * cos(boat_roll);
  float boat_mag_x = boat_mag_data.magnetic.x * cos(boat_pitch) - boat_mag_y * sin(boat_roll) * sin(boat_pitch) - boat_mag_z * cos(boat_roll) * sin(boat_pitch);

  // calculate motor magnetometer values with tilt compensation
  float motor_mag_z = motor_mag_data.magnetic.z;
  float motor_mag_y = motor_mag_z * sin(motor_roll) - motor_mag_data.magnetic.y * cos(motor_roll);
  float motor_mag_x = motor_mag_data.magnetic.x * cos(motor_pitch) - motor_mag_y * sin(motor_roll) * sin(motor_pitch) - motor_mag_z * cos(motor_roll) * sin(motor_pitch);

  if (boat_mag_x == 0) {
    if (boat_mag_y > 0) {
      compass_boat = 0;
    }
    else {
      compass_boat = 180;
    }
  }

  compass_boat = 360 - atan2(boat_mag_y, boat_mag_x) / deg_to_rad;
  if (compass_boat > 360) {
    compass_boat = compass_boat - 360;
  }

  if (motor_mag_x == 0) {
    if (motor_mag_y > 0) {
      compass_motor = 0;
    }
    else {
      compass_motor = 180;
    }
  }

  compass_motor = 360 - atan2(motor_mag_y, motor_mag_x) / deg_to_rad;
  if (compass_motor > 360) {
    compass_motor = compass_motor - 360;
  }

  motor_boat_angle = compass_motor - compass_boat;
  if (motor_boat_angle < 0) {
    motor_boat_angle = motor_boat_angle + 360;
  }
*/

sensors_event_t boatorientationData;
sensors_event_t motororientationData;

boat_dir.getEvent(&boatorientationData, Adafruit_BNO055::VECTOR_EULER);
motor_dir.getEvent(&motororientationData, Adafruit_BNO055::VECTOR_EULER);

compass_boat = boatorientationData.orientation.x;
compass_motor = motororientationData.orientation.x;

  txData.compass_boat = compass_boat;
  txData.compass_motor = compass_motor;

  //Serial.print("Boat orientation angle: "); Serial.print(compass_boat); Serial.print(" degrees,     ");
  //Serial.print("Motor orientation angle: "); Serial.print(compass_motor); Serial.println(" degrees");
}

void send_data_to_controller()
{
      // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &txData, sizeof(txData));

  if (result == ESP_OK) {
    //    Serial.println("Sent with success");
    sendError = false;
  }
  else {
    Serial.println("Error sending the data");
    sendError = true;
  }
}

void compass_calibration(){

Serial.print("SYS: "); Serial.print(sys);
Serial.print(" G: "); Serial.print(gyro);
Serial.print(" A: "); Serial.print(accel);
Serial.print(" M: "); Serial.println(mag);
}