/*
 *  This sketch sends Tello commands over UDP from a ESP32 device
 *
 */
#include <Bluepad32.h>
#include "FS.h"
#include "SPIFFS.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include "Wire.h"
#include <MPU6050_light.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <EasyButton.h>

// LED configs
#define LED_CONN_RED         26
#define IN_FLIGHT            25
#define LED_CONN_GREEN       21
#define LED_BATT_RED         27
#define LED_BATT_YELLOW      15
#define LED_BATT_GREEN        4
#define COMMAND_TICK         13
#define UP_PIN               34
#define TAKEOFF_PIN          33
#define CW_PIN               32
#define CCW_PIN              39
#define KILL_PIN             36
#define DOWN_PIN             14
#define BATTERY_CHECK_LIMIT  10
#define VBATPIN              35

#define FORMAT_SPIFFS_IF_FAILED true

File flightFile;
const char* flightFilePath = "/flight_file.txt";

const float MAX_BATTERY_VOLTAGE = 4.2; // Max LiPoly voltage of a 3.7 battery is 4.2

//IP address to send UDP data to:
// either use the ip address of the server or 
// a network broadcast address
const char * udpAddress = "192.168.10.1";
const int udpPort = 8889;

MPU6050 mpu(Wire);
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);
GamepadPtr myGamepad;

WiFiManager wm; // global wm instance

EasyButton cwButton(CW_PIN);
EasyButton ccwButton(CCW_PIN);
EasyButton takeoffButton(TAKEOFF_PIN);
EasyButton upButton(UP_PIN);
EasyButton downButton(DOWN_PIN);
EasyButton killButton(KILL_PIN);

int Roll;
int AbsRoll;
int Pitch;
int AbsPitch;
int Yaw;

String tello_ssid = "";
String rcCmdBegin = "rc ";
String rcCmdEnd = " 0 0";
String pitchString = "0";
String rollString = "0";
String gestureCmd = "rc 0 0 0 0";
String lastGestureCmd = "rc 0 0 0 0";
String lastCommand;
unsigned long last_since_takeoff = 0;
unsigned long this_since_takeoff = 0;
unsigned long takeoff_time = 0;
unsigned long commandDelay = 0;

//The udp library class
WiFiUDP udp;

//Are we currently connected?
boolean connected;
boolean in_flight = false;
boolean in_rc_btn_motion = false;
boolean inSerialMotion = false;
boolean command_error = false;
boolean battery_checked = false;

int battery_check_tick = 0;
uint8_t buffer[50];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
  if (myGamepad == nullptr) {
    Serial.printf("CALLBACK: Gamepad is now connected\n");
    // Additionally, you can get certain gamepad properties like:
    // Model, VID, PID, BTAddr, flags, etc.
    GamepadProperties properties = gp->getProperties();
    Serial.printf("Gamepad model: %s, VID=0x%04x, PID=0x%04x\n",
                  gp->getModelName().c_str(), properties.vendor_id,
                  properties.product_id);
    myGamepad = gp;
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(gp->getModelName().c_str());
    display.println("connected");
    display.display();
  }
}

void onDisconnectedGamepad(GamepadPtr gp) {
  if (myGamepad == gp) {
    Serial.printf("CALLBACK: Gamepad is disconnected\n");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(gp->getModelName().c_str());
    display.println("disconnected");
    display.display();
    myGamepad = nullptr;
  }
}


//wifi event handler
void WiFiEvent(WiFiEvent_t event){
    switch(event) {
      case SYSTEM_EVENT_STA_GOT_IP:
          //When connected set 
          Serial.print("WiFi connected! IP address: ");
          Serial.println(WiFi.localIP());
          digitalWrite(LED_CONN_GREEN,HIGH);
          digitalWrite(LED_CONN_RED,LOW);  
          //initializes the UDP state
          //This initializes the transfer buffer
          udp.begin(WiFi.localIP(),udpPort);
          connected = true;
          run_command("command", 20);
          run_command("battery?", 20);
          battery_check_tick = 0;
          run_command("command", 10);
          display.clearDisplay();
          display.setCursor(0, 0);
          display.println("Tello SSID:");
          display.println(tello_ssid);
          display.println("");
          display.println("Connected!");
          display.display();
          delay(2000);
          run_command("battery?", 10);
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          Serial.println("WiFi lost connection");
          digitalWrite(LED_CONN_GREEN,LOW);
          digitalWrite(LED_CONN_RED,HIGH);
          digitalWrite(LED_BATT_YELLOW, HIGH);
          digitalWrite(LED_BATT_RED, LOW);
          digitalWrite(LED_BATT_GREEN, LOW);
          connected = false;
          break;
      default: break;
    }
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("- failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("- file written");
    } else {
        Serial.println("- write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\r\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("- failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("- message appended");
    } else {
        Serial.println("- append failed");
    }
    file.close();
}


void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\r\n", path);
    if(fs.remove(path)){
        Serial.println("- file deleted");
    } else {
        Serial.println("- delete failed");
    }
}

void appendLastCommand() {
   this_since_takeoff = (millis() - takeoff_time);
   commandDelay = this_since_takeoff - last_since_takeoff;
   last_since_takeoff = this_since_takeoff;
   lastCommand = lastCommand + "," + commandDelay + "\n" ;
   appendFile(SPIFFS, flightFilePath, lastCommand.c_str());
}

void processCommand(String command) {
  appendLastCommand();
  if (in_rc_btn_motion) {
    run_command("rc 0 0 0 0", 0);
    lastCommand = "rc 0 0 0 0";
    in_rc_btn_motion = false;
  } else {
    run_command(command, 0);
    lastCommand = command;
    in_rc_btn_motion = true;
  }
  battery_check_tick++;
}

void processSerialCommand(String command) {
  appendLastCommand();
//  Serial.println(command);
  run_command(command, 20);
  lastCommand = command;
  battery_check_tick++;
}

void processFlightReplay() {
  flightFile = SPIFFS.open(flightFilePath, FILE_READ);
  if (flightFile) {
    Serial.println("Start of Flight File...");
    digitalWrite(IN_FLIGHT,HIGH);
    in_flight = true;
    while(flightFile.available()) {
        String command = flightFile.readStringUntil('\n');
        int commaPosition = command.indexOf(',');
        if (commaPosition != -1) {
          run_command(command.substring(0, commaPosition), 20);
          commandDelay = command.substring(commaPosition + 1, command.length()).toInt();
          // Serial.println(command);
          // Serial.println(command.substring(0, commaPosition));
          // Serial.println(commandDelay);
          delay(commandDelay);
          // delay(500);
        } else {
          run_command(command, 40);
          // delay(500);
          // Serial.println(command);
        }
    }
    digitalWrite(IN_FLIGHT,LOW);
    in_flight = false;
    Serial.println("... end of Flight File");
    flightFile.close();
  }
}

// Callbacks

void onResetWiFiButtonPressed()
{ 
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Controller WiFi Reset");
  display.println("Use ManageTello AP");
  display.println("On Phone or Computer");
  display.println("To Connect to Tello");
  display.display();

  Serial.println("Kill Button Double Pressed");
  Serial.println("Erasing WiFi Config, restarting...");
  wm.resetSettings();
  ESP.restart();      
}

void onCWButtonPressed() {
  if (in_flight) {
    // Serial.println("CW button is pressed");
    processCommand("rc 0 0 0 50");
  }
}

void onCCWButtonPressed() {
  if (in_flight) {
    // Serial.println("CCW button is pressed");
    processCommand("rc 0 0 0 -50");
  }
}


void onUpButtonPressed() {
  if (in_flight) {
    // Serial.println("UP button is pressed");
    processCommand("rc 0 0 30 0");
  }
}

void onDownButtonPressed() {
  if (in_flight) {
    //Serial.println("DOWN button is pressed");
    processCommand("rc 0 0 -30 0");
  }
}

void onKillButtonPressed() {
    Serial.println("KILL button is pressed");
    if (!connected) {
      Serial.println("Kill Button Pressed, no connection");
      Serial.println("Enabling OTA Update");
      Serial.println("Perform Update in browser tab or window");
      Serial.println("Clearing recent Tello SSID and restarting.");
      wm.resetSettings();
      ESP.restart();  
    }
    if (in_flight) {
      run_command("emergency", 10);
      battery_check_tick++;
      digitalWrite(IN_FLIGHT,LOW);
      in_flight = false;
      deleteFile(SPIFFS, flightFilePath);
    } else {
      processFlightReplay();
    }
}

void processLand () {
  appendLastCommand();
  run_command("land", 20);
  appendFile(SPIFFS, flightFilePath, "land,2\n");
  appendFile(SPIFFS, flightFilePath, "battery?,2\n");
  digitalWrite(IN_FLIGHT,LOW);
  in_flight = false;
}

void processTakeoff () {
  writeFile(SPIFFS, flightFilePath, "command,2\n");
  appendFile(SPIFFS, flightFilePath, "battery?,2\n");
  run_command("takeoff", 40);
  digitalWrite(IN_FLIGHT,HIGH);
  takeoff_time = millis();
  last_since_takeoff = 0;
  in_flight = true;
  lastCommand = "takeoff";
  appendLastCommand(); // will be takeoff
  lastCommand = "rc 0 0 0 0"; // this is basic hover
}

void onTakeoffButtonPressed() {
  Serial.println("Takeoff button is pressed");
  if (in_flight) {
     processLand();
  } else {
     processTakeoff();
  }
  run_command("battery?", 10);
  battery_check_tick = 0;
}

void setup(){
  wm.setConfigPortalTimeout(75); // auto close configportal after 75 seconds
  
  // Initilize hardware serial:
  Serial.begin(115200);
  // Serial.setTimeout(0);

  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2],
                addr[3], addr[4], addr[5]);
  String manageTello = "ManageTello" + String(addr[5]);
  Serial.println(manageTello);
  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
  
  // BP32.forgetBluetoothKeys();

  if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
      Serial.println("SPIFFS Mount Failed");
      return;
  }

  Wire.begin();

  pinMode(LED_CONN_RED, OUTPUT);
  pinMode(LED_CONN_GREEN, OUTPUT);
  pinMode(LED_BATT_RED, OUTPUT);
  pinMode(LED_BATT_YELLOW, OUTPUT);
  pinMode(LED_BATT_GREEN, OUTPUT);
  pinMode(COMMAND_TICK, OUTPUT);
  pinMode(IN_FLIGHT, OUTPUT);
  digitalWrite(LED_CONN_GREEN, LOW);
  digitalWrite(IN_FLIGHT, LOW);
  digitalWrite(LED_CONN_RED, HIGH);
  digitalWrite(LED_BATT_RED, LOW);
  digitalWrite(LED_BATT_GREEN, LOW);
  digitalWrite(LED_BATT_YELLOW, LOW);
  digitalWrite(COMMAND_TICK, LOW);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }
  display.display();
  delay(500); // Pause for 0.5 seconds
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setRotation(0);
  display.clearDisplay();
  display.setCursor(0, 0);
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  display.println("MPU6050 status: ");
  Serial.println(status);
  display.println(status);
  display.display();
 
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  display.println("Hold still, don't move : ");
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
  display.println("Done!");
  display.display();
  
  Serial.println("");
  delay(100);

  int rawValue = analogRead(VBATPIN);
  float voltageLevel = (rawValue / 4095.0) * 2 * 1.1 * 3.3; // calculate voltage level
  int batteryFraction = voltageLevel / MAX_BATTERY_VOLTAGE * 100;
  Serial.print("Controller Battery %: " ); 
  Serial.println(batteryFraction);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Controller Batt %:");
  display.println(batteryFraction);
  display.display();

  delay(2000);
  
  cwButton.begin();
  ccwButton.begin();
  takeoffButton.begin();
  killButton.begin();
  upButton.begin();
  downButton.begin();
  cwButton.onPressed(onCWButtonPressed);
  ccwButton.onPressed(onCCWButtonPressed);
  takeoffButton.onPressed(onTakeoffButtonPressed);
  killButton.onPressed(onKillButtonPressed);
  killButton.onSequence(2, 2000, onResetWiFiButtonPressed);
  upButton.onPressed(onUpButtonPressed);
  downButton.onPressed(onDownButtonPressed);
  
  connected = false;
  WiFi.mode(WIFI_STA);
  WiFi.onEvent(WiFiEvent);

  // wm.resetSettings(); // uncomment to force new Tello Binding here

  bool res;
//  res = wm.autoConnect("ManageTello","telloadmin"); // password protected ap
  res = wm.autoConnect(manageTello.c_str(),"telloadmin"); // password protected ap
  if(!res) {
    Serial.println("Failed to connect or hit timeout");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Reset Controller");
    display.println("Use ManageTello AP");
    display.println("On Phone or Computer");
    display.println("To Connect to Tello");
    display.display();

    // ESP.restart();
  } 
  else {
    //if you get here you have connected to the WiFi    
    Serial.println("connected with DroneBlocks controller to Tello WiFi :)");
    tello_ssid = (String)wm.getWiFiSSID();
  }  
}

void loop() {

  mpu.update();
  Roll = mpu.getAngleX();
  Pitch = mpu.getAngleY();
  Yaw = mpu.getAngleZ();

  AbsPitch = abs(Pitch);
  AbsRoll = abs(Roll);

  takeoffButton.read();
  killButton.read();
  cwButton.read();
  ccwButton.read();
  upButton.read();
  downButton.read();
  
  if (AbsRoll <= 10) {
    rollString = "0";
  }
  if (AbsPitch <= 15) {
    pitchString = "0";
  }

  if (Pitch < -16) {
    AbsPitch = constrain(AbsPitch, 20, 40);
    switch (AbsPitch) {
      case 20:
        pitchString = "20";
        break;
      case 40:
        pitchString = "40";
        break;
      default:
        pitchString = "30";
        // pitchString = String(AbsPitch);
        break;          
    }
  }
  if (Pitch > 16) {
    AbsPitch = constrain(AbsPitch, 20, 40);
    switch (AbsPitch) {
      case 20:
        pitchString = "-20";
        break;
      case 40:
        pitchString = "-40";
        break;
      default:
        pitchString = "-30";
        // pitchString = "-" + AbsPitch;
        break;          
    }
   }
  if (Roll < -11) {
    AbsRoll = constrain(AbsRoll, 20, 40);
    switch (AbsRoll) {
      case 20:
        rollString = "20";
        break;
      case 40:
        rollString = "40";
        break;
      default:
        rollString = "30";
        // rollString = String(AbsRoll);
        break;          
    }
  }
  if (Roll > 11) {
    AbsRoll = constrain(AbsRoll, 20, 40);
    switch (AbsRoll) {
      case 20:
        rollString = "-20";
        break;
      case 40:
        rollString = "-40";
        break;
      default:
        rollString = "-30";
        // rollString = "-" + AbsRoll;
        break;          
    }
  }

  BP32.update();

  if (myGamepad && myGamepad->isConnected()) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...

    if (myGamepad->dpad() == 0x01) {
      //Serial.println("go Forward");
      pitchString = "30";
      rollString = "0";
    }

    if (myGamepad->dpad() == 0x02) {
      //Serial.println("go Back");
      pitchString = "-30";
      rollString = "0";
    }

    if (myGamepad->dpad() == 0x04) {
      //Serial.println("go Right");
      pitchString = "0";
      rollString = "30";
    }

    if (myGamepad->dpad() == 0x05) {
      //Serial.println("go Forward Right");
      pitchString = "30";
      rollString = "30";
    }

    if (myGamepad->dpad() == 0x06) {
      //Serial.println("go Back Right");
      pitchString = "-30";
      rollString = "30";
    }

    if (myGamepad->dpad() == 0x08) {
      //Serial.println("go Left");
      pitchString = "0";
      rollString = "-30";
    }

    if (myGamepad->dpad() == 0x09) {
      //Serial.println("go Forward Left");
      pitchString = "30";
      rollString = "-30";
    }

    if (myGamepad->dpad() == 0x0a) {
      //Serial.println("go Back Left");
      pitchString = "-30";
      rollString = "-30";
    }

    if (myGamepad->x()) {
      // Serial.println("Square button pressed");
      onCCWButtonPressed();
    }

    if (myGamepad->b()) {
      // Serial.println("Circle button pressed");
      onCWButtonPressed();
    }

    if (myGamepad->a()) {
      // Serial.println("Cross button pressed");
      onDownButtonPressed();
    }

    if (myGamepad->y()) {
      // Serial.println("Triangle button pressed");
      onUpButtonPressed();
    }

    if (myGamepad->l1()) {
      // Serial.println("Left Shoulder button pressed");
      onTakeoffButtonPressed();
    }

    if (myGamepad->r1()) {
      // Serial.println("Right Shoulder button pressed");
      onKillButtonPressed();
    }

  }

  lastGestureCmd = gestureCmd;
  gestureCmd = rcCmdBegin + rollString + " " + pitchString + rcCmdEnd;
  
  if (command_error) {
    Serial.println("Command Error: Attempt to Land");
    run_command("land", 40);
    run_command("battery?", 30);
    battery_check_tick = 0;
    if (in_flight) {
      digitalWrite(IN_FLIGHT,LOW);
      in_flight = false;      
    }
    command_error = false;
  }
  // Tello nose direction is pilot perspective.
  if (in_flight) {
    if (!gestureCmd.equals(lastGestureCmd) && !in_rc_btn_motion) {
      lastCommand = lastGestureCmd;
      appendLastCommand();
      run_command(gestureCmd, 0);
      Serial.println(gestureCmd);
    } else if (!in_rc_btn_motion && !inSerialMotion) {
      lastCommand = "rc 0 0 0 0"; //default last command
    }
  }  
  
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    char SSID[65];
    if (command.length() == 0) command = Serial.readStringUntil('\r');
    if (command.length() > 0) {
      command.trim();
      if (command.startsWith("connect")) {
        command.replace("connect", "");
        command.trim();
        strcpy(SSID, command.c_str());
        WiFi.begin(SSID);
      } else if (command.startsWith("start")) {
        inSerialMotion = true;
        onTakeoffButtonPressed();
      } else if (command.startsWith("stop")) {
        onTakeoffButtonPressed();
        inSerialMotion = false;
      } else if (command.startsWith("replay")) {
        processFlightReplay();
      } else if (command.startsWith("kill")) {
        onKillButtonPressed();
        inSerialMotion = false;
      } else if (connected) {
        processSerialCommand(command);
      }
    }
  }
  if(battery_check_tick == BATTERY_CHECK_LIMIT) {
    run_command("battery?",10);
    battery_check_tick = 0;
  }
  //delay(500);
  vTaskDelay(1);  
}

void toggle_led(int ledToToggle){
  // Toggle the state of the LED pin (write the NOT of the current state to the LED pin)
  digitalWrite(ledToToggle, !digitalRead(ledToToggle));
}

void run_command(String command, int udp_delay_ticks) {
  int packetSize = 0;
  boolean responseExpected = true;
  display.clearDisplay();
  display.setCursor(0, 0);
  digitalWrite(COMMAND_TICK,LOW);
  Serial.println(command);
  display.println("Command:");
  display.println(command);
  display.display();
  // Special delay cases
  if (command.indexOf("takeoff") >= 0) udp_delay_ticks = 40;
  if (command.indexOf("land") >= 0) udp_delay_ticks = 20;
  if (command.indexOf("rc ") >= 0) {
    udp_delay_ticks = 0;
    responseExpected = false;
    digitalWrite(COMMAND_TICK,HIGH);
  }
  memset(buffer, 0, 50);
  command.getBytes(buffer,command.length()+1);
  //only send data when connected
  //Send a packet
  udp.beginPacket(udpAddress,udpPort);
  udp.write(buffer, command.length()+1);
  udp.endPacket();
  // Serial.println("endPacket called");
  // allow for an rc command to not need any further processing after sending.
  memset(buffer, 0, 50);
  for (int x = 0; x < udp_delay_ticks; x++) {
    delay(500);
    toggle_led(COMMAND_TICK);
    // Serial.print(x);
    packetSize = udp.parsePacket();
    if (packetSize) break;
  }
  // Serial.println("packetSize: " + String(packetSize));
  if (packetSize && responseExpected) {
     if(udp.read(buffer, 50) > 0){
      digitalWrite(COMMAND_TICK,HIGH);
      String commandResponse = String((char *) buffer);
      Serial.println(commandResponse);
      display.println("Response:");
      display.println(commandResponse);
      display.display();
      bool parseResponse = (commandResponse.indexOf("error") == -1) && (commandResponse.indexOf("ok") == -1);
      if (command.equalsIgnoreCase("battery?")  && parseResponse) {
        int battery = commandResponse.toInt();
        if (battery > 60) {
          digitalWrite(LED_BATT_GREEN,HIGH);
          digitalWrite(LED_BATT_RED,LOW);
          digitalWrite(LED_BATT_YELLOW,LOW);
        } else if (battery > 20) {
          digitalWrite(LED_BATT_GREEN,LOW);
          digitalWrite(LED_BATT_RED,LOW);
          digitalWrite(LED_BATT_YELLOW,HIGH);
        } else {
          digitalWrite(LED_BATT_GREEN,LOW);
          digitalWrite(LED_BATT_RED,HIGH);
          digitalWrite(LED_BATT_YELLOW,LOW);
        }
      } else if (commandResponse.indexOf("timeout") >= 0) {
        digitalWrite(COMMAND_TICK,LOW);
        Serial.println("Command timed out, ignoring for now");
      }
    } else {
      digitalWrite(COMMAND_TICK,LOW);
      command_error = true;
    }
  } else if (in_flight && responseExpected) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("No command response: ");
    display.println("Landing NOW!");
    display.display();
    command_error = true;
  }
//  delay(100);   
}
