//==========================
//1.  Includes & Defines
//==========================

//Libaries
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// Define receiver MAC address
uint8_t broadcastAddress[] = {0xb8, 0xf8, 0x62, 0xd6, 0xb8, 0xd8};

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
// Structure example to send data
// Must match the receiver structure
typedef struct {
  uint8_t mode;
  bool powerToggle;
  bool Up;
  bool Down;
  bool left;
  bool right;
  bool sleep;
} ControlData;

//=========================
//3.  Global Variables
//=========================
//Timer for stepping speed
unsigned long lastSpeedStep = 0;
const unsigned long speedInterval = 100;
bool doSpeedUp = false;
bool doSpeedDown = false;

bool newDataReceived = false;


//Fail-safe timeout
unsigned long lastPacketTime = 0;
const unsigned long failSafeTime = 300000; //5 minutes, might need to change failsafe for nav/anchor unless we add in signal sending to make sure remote is alive

int locTap = 0; // This variable will track the current position of the digipot

//Last state tracking
ControlData lastState = {0, false,false, false, false, false, false};
ControlData rxData;

//=========================
//4.  Receiver Callback Function
//=========================
// Callback when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&rxData, incomingData, sizeof(rxData));
  newDataReceived = true; // Set flag to indicate new data has been received

  lastPacketTime = millis(); //For failsafe
  }
//==========================
//4.  Function Prototypes
//==========================
void handleMode();
void handleManualMode();
void handleNavMode();
void handleAnchorMode();
void stepDigipotUp();
void stepDigipotDown();

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
  
    // Initialize WiFi
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  // Register receive callback
  esp_now_register_recv_cb(OnDataRecv);

  rxData.mode = mode_manual; //start in manual mode
  rxData.powerToggle = 0; //make sure motor is off

  Serial.println("Receiver Ready");
}

void loop() {
  if(newDataReceived){
    handleMode();
    newDataReceived = false; // Reset flag after handling new data
  }

  unsigned long now = millis();

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



}

//==========================
//6.  Helper Functions
//==========================
void handleMode(){

  switch (rxData.mode){

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
    //Hold position, so only update steering
 } 
}

void handleManualMode(){
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
    //placeholder for nav mode
}

void handleAnchorMode(){
    //placeholder for anchor mode
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