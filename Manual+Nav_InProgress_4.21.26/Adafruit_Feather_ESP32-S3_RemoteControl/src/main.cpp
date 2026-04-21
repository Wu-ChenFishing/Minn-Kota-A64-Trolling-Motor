//==========================
//1.  Includes & Defines
//==========================

//Libaries
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_SSD1327.h> //LCD Library
//#include <Adafruit_LC709203F.h> //Battery Library, this was discontinued in 2023, this does not have thermistor
#include <Adafruit_MAX1704X.h> //Battery Library

// Define receiver MAC address for esp-now communication
uint8_t broadcastAddress[] = {0x30, 0x30, 0xf9, 0x6c, 0x57, 0xc8};

//Define Pins
#define nav_pin 17
#define power_pin 6
#define up_pin 16
#define down_pin 14
#define left_pin 5
#define right_pin 8
#define manual_pin 18
#define anchor_pin 15
#define OLED_CLK 36 //OLED CLK
#define OLED_MOSI 35 //OLED MOSI
#define OLED_CS 10 //OLED CS
#define OLED_DC 37 //OLED DC
#define OLED_RESET -1 //OLED Reset

//Define Control Modes
#define mode_manual 0 //Manual control
#define mode_nav 1 //hold heading
#define mode_anchor 2 //spotlock

//Constants
Adafruit_SSD1327 display(128, 128, OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

// Keep track of motor state locally
bool motorState = true; // Current ON/OFF state, start off (inversion)
bool sleepState = false;
bool inputLocked = true;
unsigned long bootTime;

//Button debounce
const int debounceDelay = 30;

Adafruit_MAX17048 batt;  // battery management sensor

//==========================
//2.  Types & Structures
//==========================

//Button Objects
 struct Button {
    bool raw;
    bool stable;
    bool lastStable;
    unsigned long lastDebounceTime;
};

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

// Structure to receive data from receiver
// Must match the receiver structure
typedef struct {
  uint8_t mode;
  int speed = 0;
  int compass_boat = 0;         // direction that the boat is facing according to boat compass
  int compass_motor = 0;        // direction that the motor is facing according to the motor compass
  int nav_heading = 0;              // heading set in NAV mode
} ReceiverData;


//=========================
//3.  Global Variables
//=========================

  // Create a struct_message called txData to send to base unit
  ControlData txData;

  // Create a struct_message called rxData to receive from base unit
  ReceiverData rxData;

  //Create ESP-NOW structure
 esp_now_peer_info_t peerInfo;

  Button powerBtn;
  Button UpBtn;
  Button DownBtn;
  Button leftBtn;
  Button rightBtn;
  Button manualBtn;
  Button navBtn;
  Button anchorBtn;

  //Constant Variables
  unsigned long screen_sleep_timer = 0;
  float CurrentVoltage = 0;
  float soc = 0; //State of charge %
  float lowVoltageCutoff = 3.3; // Set your desired cutoff voltage here
  float MaxVoltage = 4.12; //This is max battery voltage when charged.  Maybe battery is old because typical voltage should be 4.2 -JSC 4.7.26
  bool newDataReceived = false;
  
//Fail-safe timeout
  unsigned long lastPacketTime = 0;

//Data received variables
  int compass_boat = 0;         // direction that the boat is facing according to boat compass
  int compass_motor = 0;        // direction that the motor is facing according to the motor compass

//==========================
//4.  Function Prototypes
//==========================

void readInputs();
void handleMode(bool &changed);
void handleManualMode(bool &changed);
void handleNavMode(bool &changed);
void handleAnchorMode(bool &changed);
void resolveSafety();
void updateButton(Button &btn, int pin);
bool wasPressed(Button &btn);
bool wasReleased(Button &btn);
bool isHeld(Button &btn);
void updateBattery();
void primeButtons();
void handle_display();
void handle_displayManualMode();
void handle_displayNavMode();
void handle_displayAnchorMode();

//Data send function
void send_data(){
  esp_now_send(broadcastAddress, (uint8_t*)&txData, sizeof(txData));
}

//=======================
//Callback function
//=======================
// Callback when data is received
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&rxData, incomingData, sizeof(rxData));
  newDataReceived = true; // Set flag to indicate new data has been received

  lastPacketTime = millis(); //For failsafe
  }

//==========================
//5.  Setup Program & Main Loop
//==========================

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  pinMode(power_pin,INPUT);
  pinMode(up_pin,INPUT);
  pinMode(down_pin,INPUT);
  pinMode(left_pin,INPUT);
  pinMode(right_pin,INPUT);
  pinMode(manual_pin,INPUT);
  pinMode(nav_pin,INPUT);
  pinMode(anchor_pin,INPUT);

  powerBtn = {false, false, 0};
  UpBtn = {false, false, 0};
  DownBtn = {false, false, 0};
  leftBtn = {false, false, 0};
  rightBtn = {false, false, 0};
  manualBtn = {false, false, 0};
  navBtn = {false, false, 0};
  anchorBtn = {false, false, 0};

  bootTime = millis();
  inputLocked = true;
  

  screen_sleep_timer = millis(); //Start screen sleep timer

  //Set WiFi in Station Mode
  WiFi.mode(WIFI_STA);

  //Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
  Serial.println("Error initializing ESP-NOW");
  return;
  }

  //Register for send callback
 //esp_now_register_send_cb(OnDataSent);

  //Register for receive callback
  esp_now_register_recv_cb(OnDataRecv);
  

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  //Test OLED Display
   if (!display.begin(0x3D) ) {
    Serial.println("Unable to initialize OLED");
    while (1) yield();
  } 
  
  display.clearDisplay();
  display.display();

  #if defined (ARDUINO_ADAFRUIT_FEATHER_ESP32S3)
    pinMode(PIN_I2C_POWER, INPUT);
    delay(1);
    bool polarity = digitalRead(PIN_I2C_POWER);
    pinMode(PIN_I2C_POWER, OUTPUT);
    digitalWrite(PIN_I2C_POWER, !polarity);
  #endif

  if(!batt.begin()) {
    Serial.println(F("Couldn't find Adafruit MAX17048?\nMake sure the battery is plugged in!"));
    while (1){ 
      delay(10);
    };
  }

Serial.println(F("Found Max17048"));
//batt.quickStart();  ///Resets state of charge %, this will be top voltage at current state of battery, this is best used when battery is fully charged

txData.mode = mode_manual; //Default to manual mode on startup
Serial.println("Program Starting");
}

void loop() {
//Serial.println(!digitalRead(nav_pin));
//Serial.println(digitalRead(up_pin));

  if (inputLocked && millis() - bootTime > 300) {
  primeButtons();   // stabilize inputs
  inputLocked = false;
  }

  bool changed = false;

  readInputs();           //Hardware Only
  handleMode(changed);    //Logic

    if (changed){
      send_data();
      handle_display();
      Serial.println("Sent Data");
    }

  if(newDataReceived){
    handle_display();
    newDataReceived = false; // Reset flag after handling new data
  }

    static unsigned long lastBattUpdate = 0;

    if (millis() - lastBattUpdate > 60000) { // Update battery info every 60 seconds
    updateBattery();
    lastBattUpdate = millis();
    }

  delay(5);
}

//==========================
//6.  Helper Functions
//==========================
void readInputs(){
  updateButton(powerBtn, power_pin);
  updateButton(UpBtn, up_pin);
  updateButton(DownBtn, down_pin);
  updateButton(leftBtn, left_pin);
  updateButton(rightBtn, right_pin);
  updateButton(manualBtn, manual_pin);
  updateButton(navBtn, nav_pin);
  updateButton(anchorBtn, anchor_pin);
}

void handleMode(bool &changed){

// Mode switching (edge triggered)
if (!inputLocked && wasPressed(manualBtn)) {
  txData.mode = mode_manual;
  changed = true;
  Serial.println("Switched to Manual Mode");
}

if (!inputLocked && wasPressed(navBtn)) {
  txData.mode = mode_nav;
  changed = true;
  Serial.println("Switched to Navigation Mode");
}

if (!inputLocked && wasPressed(anchorBtn)) {
  txData.mode = mode_anchor;
  changed = true;
  Serial.println("Switched to Anchor Mode");
}

  switch (txData.mode){

    case mode_manual: //mode_manual
    handleManualMode(changed);
    break;

    case mode_nav:
    handleNavMode(changed);
    break;
    //Hold heading, so only update speed and power 

    case mode_anchor:
    handleAnchorMode(changed);
    break;
    //Hold position, so only update steering
 } 
}

void handleManualMode(bool &changed){

  if (wasPressed(powerBtn)) {
    // Toggle motor state
    motorState = !motorState;
    txData.powerToggle = motorState; // send current motor state
    changed = true;
    Serial.print("Motor toggled: "); Serial.println(motorState);
  }

//Speed Controls
bool newSpeedUp = isHeld(UpBtn);
bool newSpeedDown = isHeld(DownBtn);

bool newUp = false;
bool newDown = false;

if (!newSpeedUp&&newSpeedDown) newUp=true;
else if (!newSpeedDown&&newSpeedUp) newDown=true;

if(txData.Up != newUp || txData.Down != newDown){
  txData.Up = newUp;
  txData.Down = newDown;
  changed = true;
  if (newUp) Serial.println("Speed Up held");
  else Serial.println("Speed Up released");
  if (newDown) Serial.println("Speed Down held");
  else Serial.println("Speed Down released");
}

//Steering
bool left = isHeld(leftBtn);
bool right = isHeld(rightBtn);
int8_t steering = 0; // 0 = no turn, -1 = left, 1 = right

if (!left && right) steering = -1;
else if (!right && left) steering = 1;
else steering = 0;

bool newLeft = (steering == -1);
bool newRight = (steering == 1);

//Only trigger send if something changed
if (txData.left != newLeft || txData.right != newRight) {
    txData.left = newLeft;
    txData.right = newRight;
    changed = true;
    
    if (newLeft) Serial.println("Steering LEFT");
    else if (newRight) Serial.println("Steering RIGHT");
    else Serial.println("Steering NEUTRAL");  
  }
}

//Placeholder for now, will implement navigation mode after we get basic manual controls working, this will likely involve using magnotometer from the base unit to hold heading, so we will need to add magnotometer data to the esp-now communication
void handleNavMode(bool &changed){
  //Hold heading, so only update speed and power 
 if (wasPressed(powerBtn)) {
    // Toggle motor state
    motorState = !motorState;
    txData.powerToggle = motorState; // send current motor state
    changed = true;
    Serial.print("Motor toggled: "); Serial.println(motorState);
  }

//Speed Controls
bool newSpeedUp = isHeld(UpBtn);
bool newSpeedDown = isHeld(DownBtn);

bool newUp = false;
bool newDown = false;

if (!newSpeedUp&&newSpeedDown) newUp=true;
else if (!newSpeedDown&&newSpeedUp) newDown=true;

if(txData.Up != newUp || txData.Down != newDown){
  txData.Up = newUp;
  txData.Down = newDown;
  changed = true;
  if (newUp) Serial.println("Speed Up held");
  else Serial.println("Speed Up released");
  if (newDown) Serial.println("Speed Down held");
  else Serial.println("Speed Down released");
}

//Change heading
bool left = wasPressed(leftBtn);
bool right = wasPressed(rightBtn);
int8_t change_heading = 0; // 0 = no turn, -1 = left, 1 = right

if (!left && right) change_heading = -1;
else if (!right && left) change_heading = 1;
else change_heading = 0;

bool newLeft = (change_heading == -1);
bool newRight = (change_heading == 1);

//Only trigger send if something changed
if (txData.left != newLeft || txData.right != newRight) {
    txData.left = newLeft;
    txData.right = newRight;
    changed = true;
    
    if (newLeft) Serial.println("Move heading LEFT");
    else if (newRight) Serial.println("Move heading RIGHT");
    else Serial.println("Do nothing");  
  }


}


//Placeholder for now, will implement Anchor mode after we get basic manual controls working, this will likely involve using GPS data from the base unit to hold position, so we will need to add GPS data to the esp-now communication
void handleAnchorMode(bool &changed){
  //Hold position, so only update steering
}

/* Resolved steering in the manual mode function 4.16.26
void resolveSteering(bool left, bool right, bool &outLeft, bool &outRight)
{
  if (left && right) {
    outLeft = false;
    outRight = false;
    return;
  }

  outLeft = left;
  outRight = right;
}
*/

void primeButtons() {

  delay(50);

  updateButton(powerBtn, power_pin);
  updateButton(UpBtn, up_pin);
  updateButton(DownBtn, down_pin);
  updateButton(leftBtn, left_pin);
  updateButton(rightBtn, right_pin);
  updateButton(manualBtn, manual_pin);
  updateButton(navBtn, nav_pin);
  updateButton(anchorBtn, anchor_pin);

  // NOW lock baseline correctly
  powerBtn.lastStable = powerBtn.stable;
  UpBtn.lastStable = UpBtn.stable;
  DownBtn.lastStable = DownBtn.stable;
  leftBtn.lastStable = leftBtn.stable;
  rightBtn.lastStable = rightBtn.stable;
  manualBtn.lastStable = manualBtn.stable;
  navBtn.lastStable = navBtn.stable;
  anchorBtn.lastStable = anchorBtn.stable;
}

void updateButton(Button &btn, int pin) {

  bool reading = !digitalRead(pin);

  if (reading != btn.raw) {
    btn.lastDebounceTime = millis();
    btn.raw = reading;
  }

  if (millis() - btn.lastDebounceTime > debounceDelay) {
    btn.lastStable = btn.stable;
    btn.stable = btn.raw;
  }
}

bool wasPressed(Button &btn) {
  return btn.stable && !btn.lastStable;
}

bool wasReleased(Button &btn) {
 return !btn.stable && btn.lastStable;
}

bool isHeld(Button &btn) {
  return btn.stable;
}


/*Testing 4.7.26 on how to turn screen to sleep
//Serial.print("Timer:"); Serial.println(millis());
//Serial.print("Screen sleep time:"); Serial.println(screen_sleep_timer);
  if (millis() - screen_sleep_timer > 600000) {
    //  do nothing, just don't write the screen
    Serial.println("Screen is sleeping, nite nite");
    txData.sleep = true;
    sleepState = true;
    display.oled_command(SSD1327_DISPLAYOFF); // turn off display to save power
  }
    display.oled_command(SSD1327_DISPLAYON); // turn on display when button is pressed
*/

void updateBattery() {
  CurrentVoltage = batt.cellVoltage();
  soc = batt.cellPercent();
}

void handle_display(){
  //Serial.println(rxData.mode);
 switch (rxData.mode){

    case mode_manual:
    handle_displayManualMode();
    break;


    case mode_nav:
    handle_displayNavMode();
    break;
    //Hold heading, so only update speed and power 

    case mode_anchor:
    handle_displayAnchorMode();
    break;
    //Hold position, so only update steering
 } 
}

//Currently using this as my debug screen
void handle_displayManualMode(){
    Serial.println("In Manual Display");
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1327_WHITE);
    display.setCursor(1, 2);
    display.print("Mode: "); display.println("Manual");
    display.setCursor(1,12);
    if (CurrentVoltage > 3.4){
    display.print("Batt: "); display.print(CurrentVoltage, 3); display.println(" V");
    }
    else{
      display.print("Batt: "); display.println("Recharge");
    }

    display.setCursor(1,22);
    display.print("Speed: "); display.println(rxData.speed);
    display.setCursor(1,32);
    display.print("Heading:"); display.println(rxData.compass_boat);
    display.setCursor(1,42);
    display.print("Turn rate:"); display.println(rxData.compass_motor);
    display.display();
}

void handle_displayNavMode(){
  Serial.println("In Nav Display");
    int compass_center_X = 86;
    int compass_center_Y = 86;
    int compass_radius = 30;
    int sz1_ltr_offset_x = 3;   // offset to center letters on compass
    int sz1_ltr_offset_y = 4;   // offset to center letters on compass
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1327_WHITE);
    display.setCursor(1, 2);
    display.print("Mode: "); display.println("Nav"); 
    display.setCursor(1,12);
    if (CurrentVoltage > 3.4){
    display.print("Batt: "); display.print(CurrentVoltage, 3); display.println(" V");
    }
    else{
      display.print("Batt: "); display.println("Recharge");
    }
    display.setCursor(2, 22);
    display.print("Speed: "); display.println(rxData.speed);
    display.setCursor(2, 32);
    display.print("Set Heading: "); display.println(rxData.nav_heading);
    display.setCursor(1, 42);
    display.print("Dist. from heading: ");

    display.drawCircle(compass_center_X, compass_center_Y, compass_radius, SSD1327_WHITE);
    display.drawCircle(compass_center_X, compass_center_Y, compass_radius - 1, SSD1327_WHITE);
    display.drawCircle(compass_center_X, compass_center_Y, compass_radius + 1, SSD1327_WHITE);

    //Boat Info
    int sin_boat_angle = (compass_radius + 7) * sin(rxData.compass_boat*3.14/180);
    int cos_boat_angle = (compass_radius + 7) * cos(rxData.compass_boat*3.14/180);

    int north_X = compass_center_X - sin_boat_angle - sz1_ltr_offset_x;
    int north_Y = compass_center_Y - cos_boat_angle - sz1_ltr_offset_y;

    int south_X = compass_center_X + sin_boat_angle - sz1_ltr_offset_x;
    int south_Y = compass_center_Y + cos_boat_angle - sz1_ltr_offset_y;

    int east_X = compass_center_X + cos_boat_angle - sz1_ltr_offset_x;
    int east_Y = compass_center_Y - sin_boat_angle - sz1_ltr_offset_y;

    int west_X = compass_center_X - cos_boat_angle - sz1_ltr_offset_x;
    int west_Y = compass_center_Y + sin_boat_angle - sz1_ltr_offset_y;

    // put north/south/east/west on the compass
    display.setCursor(north_X, north_Y); display.println("N");
    display.setCursor(south_X, south_Y); display.println("S");
    display.setCursor(east_X, east_Y); display.println("E");
    display.setCursor(west_X, west_Y); display.println("W");
    
    display.display();
}

void handle_displayAnchorMode(){
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1327_WHITE);
    display.setCursor(1, 2);
    display.print("Mode: "); display.println("Anchor");
 
    display.setCursor(1,12);
    display.print("Voltage: "); display.print(CurrentVoltage, 3); display.println(" V");
    display.setCursor(1,22);
    display.print("Battery %: "); display.print(soc, 1); display.println(" %");
    display.setCursor(1,32);
    display.print("Boat:"); display.println(rxData.compass_boat);
    display.setCursor(1,42);
    display.print("Motor:"); display.println(rxData.compass_motor);
    display.print("Nav Heading:" ); display.println(rxData.nav_heading);
    display.display();
}