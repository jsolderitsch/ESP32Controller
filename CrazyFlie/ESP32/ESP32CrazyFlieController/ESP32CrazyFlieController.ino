/*
 *  This sketch can talk to a CrazyFlie
 *
 */
#include <WiFi.h>
#include <NetworkUdp.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <EasyButton.h>
#include "Wire.h"
#include <MPU6050_light.h>

// #define PCB
#define BBoard

// LED EEK configs

#ifdef BBoard
#define LED_CONN_RED 12
#define IN_FLIGHT 27
#define LED_CONN_GREEN 33
#define LED_BATT_RED 26
#define LED_BATT_YELLOW 25
#define LED_BATT_GREEN 4
#define COMMAND_TICK 12
#endif

// LED old PCB configs
#ifdef PCB
#define LED_CONN_RED 26
#define IN_FLIGHT 25
#define LED_CONN_GREEN 21
#define LED_BATT_RED 27
#define LED_BATT_YELLOW 15
#define LED_BATT_GREEN 4
#define COMMAND_TICK 26
#endif


// New Breadboard switch pins

#ifdef BBoard
#define UP_PIN 32
#define TAKEOFF_PIN 15
#define CW_PIN 14
#define CCW_PIN 17
#define KILL_PIN 16
#define DOWN_PIN 21
#endif

// Old PCB switch pins
#ifdef PCB
#define UP_PIN 34
#define TAKEOFF_PIN 33
#define CW_PIN 32
#define CCW_PIN 39
#define KILL_PIN 36
#define DOWN_PIN 14
#endif

#define BATTERY_CHECK_LIMIT 10


// WiFi network name and password:
const char *networkName = "Arlington";
const char *networkPswd = "MontgomeryBurns!6002ear";
// const char *networkName = "VUPlay";
// const char *networkPswd = "vuplay123";

//IP address to send UDP data to:
// either use the ip address of the server or
// a network broadcast address
const char *udpAddress = "192.168.1.199"; // replace with your host IP address where python is running
const int udpPort = 8889;
const int udpSrcPort = 3333;

MPU6050 mpu(Wire);
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

EasyButton takeoffButton(TAKEOFF_PIN);
EasyButton killButton(KILL_PIN);

int Roll;
int AbsRoll;
int Pitch;
int AbsPitch;
int Yaw;

String goCmd = "stop_v";
String lastGoCmd = "stop_v";

int straight;
int side;
int upState = 0;
int downState = 0;
int cwState = 0;
int ccwState = 0;

//Are we currently connected?
boolean connected = false;
boolean lastCommandOK = false;
boolean in_flight = false;
boolean command_error = false;
boolean battery_checked = false;

int battery_check_tick = 0;
uint8_t buffer[50];

//The udp library class
NetworkUDP udp;

void toggle_led(int ledToToggle) {
  // Toggle the state of the LED pin (write the NOT of the current state to the LED pin)
  digitalWrite(ledToToggle, !digitalRead(ledToToggle));
}

//wifi event handler
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      //When connected set
      Serial.print("WiFi connected! IP address: ");
      Serial.println(WiFi.localIP());
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("WiFiConnected!");
      display.println("IP Address:");
      display.println(WiFi.localIP());
      display.display();
      //initializes the UDP state
      //This initializes the transfer buffer
      udp.begin(WiFi.localIP(), udpSrcPort);
      connected = true;
      digitalWrite(LED_CONN_GREEN, HIGH);
      digitalWrite(LED_CONN_RED, LOW);
      digitalWrite(LED_BATT_YELLOW, HIGH);
      delay(1000);
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      digitalWrite(LED_CONN_GREEN, LOW);
      digitalWrite(LED_BATT_YELLOW, LOW);
      digitalWrite(LED_BATT_RED, LOW);
      digitalWrite(LED_BATT_GREEN, LOW);
      digitalWrite(LED_CONN_RED, HIGH);
      Serial.println("WiFi not connected");
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("WiFi not connected");
      display.display();
      connected = false;
      break;
    default:
      connected = false;
      break;
  }
}

void connectToWiFi(const char *ssid, const char *pwd) {
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  // WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);

  //Initiate connection
  WiFi.begin(ssid, pwd);
  //WiFi.begin(ssid);

  Serial.println("Waiting for WIFI connection...");
}

void run_command(String command, int udp_delay_ticks, int waitAfterDelay) {
  int udpResponseDelay = 0;
  int packetSize = 0;
  boolean responseExpected = true;
  lastCommandOK = false;

  display.clearDisplay();
  display.setCursor(0, 0);
  digitalWrite(COMMAND_TICK, LOW);
  display.println("Command:");
  display.println(command);
  display.display();

  // Special delay cases
  if (command.indexOf("takeoff") >= 0) udp_delay_ticks = 40;
  if (command.indexOf("land") >= 0) udp_delay_ticks = 20;
  if (command.indexOf("_v") >= 0) {  // Velocity commands return immediately
    udp_delay_ticks = 0;
    responseExpected = false;
    digitalWrite(COMMAND_TICK, LOW);
  } else {  // don't log _v commands to serial
    Serial.print("CrazyFlie Command: ");
    Serial.println(command);
  }

  memset(buffer, 0, 50);
  command.getBytes(buffer, command.length() + 1);
  //Send a packet
  udp.beginPacket(udpAddress, udpPort);
  udp.write(buffer, command.length() + 1);
  udp.endPacket();
  // Serial.println("endPacket called");
  memset(buffer, 0, 50);
  for (int x = 0; x < udp_delay_ticks; x++) {
    packetSize = udp.parsePacket();
    if (packetSize) break;
    toggle_led(COMMAND_TICK);
    delay(500);
    udpResponseDelay++;
  }
  if (packetSize && responseExpected) {
    if (udp.read(buffer, 50) > 0) {
      String commandResponse = String((char *)buffer);
      Serial.print("Command Response: ");
      Serial.println(commandResponse);
      display.println("Response:");
      display.println(commandResponse);
      display.display();
      digitalWrite(COMMAND_TICK, LOW);
      lastCommandOK = true;
      if (commandResponse.startsWith("battery")) {
        int battStateStart = commandResponse.indexOf(" ") + 1;
        int battery = commandResponse.substring(battStateStart).toInt();
        if (battery == 0) {  // battery state is good
          digitalWrite(LED_BATT_GREEN, HIGH);
          digitalWrite(LED_BATT_RED, LOW);
          digitalWrite(LED_BATT_YELLOW, LOW);
        } else if (battery == 1) {  // battery state is trending lower
          digitalWrite(LED_BATT_GREEN, LOW);
          digitalWrite(LED_BATT_RED, LOW);
          digitalWrite(LED_BATT_YELLOW, HIGH);
        } else if (battery == 2) {  // battery state nearing lowest allowed voltage
          digitalWrite(LED_BATT_GREEN, LOW);
          digitalWrite(LED_BATT_RED, HIGH);
          digitalWrite(LED_BATT_YELLOW, LOW);
        } else if (battery == 3) {  // low power signal or no battery left
          digitalWrite(LED_BATT_GREEN, LOW);
          digitalWrite(LED_BATT_RED, HIGH);
          digitalWrite(LED_BATT_YELLOW, HIGH);
          lastCommandOK = false;
        }
      }
    }
  } else if (responseExpected) {
    lastCommandOK = false;
    digitalWrite(COMMAND_TICK, LOW);
    Serial.println("No Response");
    display.println("No Response");
    display.display();
  }
  delay(waitAfterDelay);
}

bool run_flight_plan() {
  if (connected) {
    run_command("battery", 20, 0);
    run_command("takeoff", 40, 0);
    if (lastCommandOK) {
      digitalWrite(IN_FLIGHT, HIGH);
      in_flight = true;
    } else {
      return false;
    }
    run_command("up 50", 20, 4000);
    run_command("cw 90", 20, 4000);
    run_command("ccw 90", 20, 4000);
    run_command("down 50", 20, 4000);
    run_command("land", 20, 0);
    if (lastCommandOK) {
      digitalWrite(IN_FLIGHT, LOW);
      in_flight = false;
    }
    return true;
  }
}

// Callbacks

void onKillButtonPressed() {
  Serial.println("KILL button is pressed");
  if (in_flight) {
    in_flight = false;
    run_command("emergency", 20, 0);
    if (!lastCommandOK) run_command("emergency", 20, 0);
    digitalWrite(IN_FLIGHT, LOW);
  } else {
    bool flightPlanSuccess = run_flight_plan();
    if (!flightPlanSuccess && in_flight) {
      in_flight = false;
      run_command("emergency", 10, 0);
      if (!lastCommandOK) run_command("emergency", 10, 0);
      if (!lastCommandOK) run_command("emergency", 20, 0);
      digitalWrite(IN_FLIGHT, LOW);
    }
  }
}

void onTakeoffButtonPressed() {
  if (in_flight) {
    run_command("land", 20, 0);
    if (!lastCommandOK) {
      run_command("emergency", 20, 0);
    } else {
      digitalWrite(IN_FLIGHT, LOW);
      in_flight = false;
    }
  } else {
    run_command("takeoff", 40, 0);
    if (lastCommandOK) {
      digitalWrite(IN_FLIGHT, HIGH);
      in_flight = true;
    }
  }
}

void setup() {
  // Initilize hardware serial:
  Serial.begin(115200);

  Wire.begin();

// PCB board is not using the pullup resistors!!
#ifdef BBoard
  pinMode(UP_PIN, INPUT_PULLUP);
  pinMode(DOWN_PIN, INPUT_PULLUP);
  pinMode(CW_PIN, INPUT_PULLUP);
  pinMode(CCW_PIN, INPUT_PULLUP);
#endif

#ifdef PCB
  pinMode(UP_PIN, INPUT);
  pinMode(DOWN_PIN, INPUT);
  pinMode(CW_PIN, INPUT);
  pinMode(CCW_PIN, INPUT);
#endif

  pinMode(LED_CONN_RED, OUTPUT);
  pinMode(LED_CONN_GREEN, OUTPUT);
  pinMode(LED_BATT_RED, OUTPUT);
  pinMode(LED_BATT_YELLOW, OUTPUT);
  pinMode(LED_BATT_GREEN, OUTPUT);
  pinMode(COMMAND_TICK, OUTPUT);
  pinMode(IN_FLIGHT, OUTPUT);

  digitalWrite(LED_CONN_GREEN, LOW);
  digitalWrite(LED_CONN_RED, HIGH);
  digitalWrite(LED_BATT_RED, LOW);
  digitalWrite(LED_BATT_GREEN, LOW);
  digitalWrite(LED_BATT_YELLOW, LOW);
  digitalWrite(IN_FLIGHT, LOW);

  takeoffButton.begin();
  killButton.begin();
  takeoffButton.onPressed(onTakeoffButtonPressed);
  killButton.onPressed(onKillButtonPressed);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  display.display();
  delay(2000);  // Pause for 2 seconds
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setRotation(0);
  display.clearDisplay();
  display.setCursor(0, 0);

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  display.print("MPU6050 status: ");
  Serial.println(status);
  display.println(status);
  display.display();

  while (status != 0) {}  // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  display.println("Calculating offsets: ");
  display.println("do not move MPU");
  display.display();
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets();  // gyro and accelero
  Serial.println("Done!\n");
  display.println("Done!");
  display.display();
  delay(1000);

  connected = false;

  //Connect to the WiFi network
  // connected = false;
  WiFi.mode(WIFI_STA);
  connectToWiFi(networkName, networkPswd);
}

void loop() {

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    if (command.length() == 0) command = Serial.readStringUntil('\r');
    if (command.length() > 0) {
      command.trim();
      if (connected) {
        run_command(command, 40, 0);
      }
    }
  }

// change these states to !digitalRead for the 2024 form of the PCB
#ifdef PCB
  upState = digitalRead(UP_PIN);
  downState = digitalRead(DOWN_PIN);
  cwState = digitalRead(CW_PIN);
  ccwState = digitalRead(CCW_PIN);
#endif

#ifdef BBoard
  upState = !digitalRead(UP_PIN);
  downState = !digitalRead(DOWN_PIN);
  cwState = !digitalRead(CW_PIN);
  ccwState = !digitalRead(CCW_PIN);
#endif

  mpu.update();
  Roll = mpu.getAngleX();
  Pitch = mpu.getAngleY();
  Yaw = mpu.getAngleZ();

  AbsPitch = abs(Pitch);
  AbsRoll = abs(Roll);

  takeoffButton.read();
  killButton.read();

  lastGoCmd = goCmd;  // save last command
  goCmd = "stop_v"; // default is hover

  if (in_flight) {
    if (upState == HIGH) {
      goCmd = "up_v 20";
    } else if (downState == HIGH) {
      goCmd = "down_v 20";
    } else if (cwState == HIGH) {
      goCmd = "cw_v 45";
    } else if (ccwState == HIGH) {
      goCmd = "ccw_v 45";
    } else {             // no  buttons pressed
      // set dead zone for EEK tilt angle
      if (AbsRoll <= 10) {
        side = 0;
      } else {
        AbsRoll = constrain(AbsRoll, 11, 50);
        if (Roll < -10) {
          // go right
          side = -AbsRoll;
        } else if (Roll > 10) {
          // go left
          side = AbsRoll;
        }
      }
      // set dead zone for EEK tilt angle
      if (AbsPitch <= 15) {
        straight = 0;
      } else {
        AbsPitch = constrain(AbsPitch, 16, 50);
        if (Pitch < -15) {
          // go forward
          straight = AbsPitch;
        } else if (Pitch > 15) {
          // go back
          straight = -AbsPitch;
        }
      }
      if ((side != 0) || (straight != 0)) {
        goCmd = "go_v ";
        goCmd = goCmd + straight + " " + side + " 0 0";
      }
    }

    // CrazyFlie nose direction is pilot perspective.

    if (lastGoCmd != goCmd) {
      run_command(goCmd, 0, 0);
      //      Serial.println(goCommand);
    }

  } else { // not in_flight
    if (upState == HIGH) {
      run_command("battery", 20, 0);
    } else if (downState == HIGH) {
      run_flight_plan();
    }
  }
  // vTaskDelay(150);
}
