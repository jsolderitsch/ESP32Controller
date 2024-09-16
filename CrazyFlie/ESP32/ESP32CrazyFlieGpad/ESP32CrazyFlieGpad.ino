/*
 *  This sketch can talk to a CrazyFlie
 *
 */
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include "Wire.h"
#include <Bluepad32.h>

// #define PCB
#define BBoard
// #define Hat

// LED configs

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

// LED configs for Hat
#ifdef Hat
#define LED_CONN_RED 26
#define IN_FLIGHT 25
#define LED_CONN_GREEN 4
#define LED_BATT_RED 13
#define LED_BATT_YELLOW 33
#define LED_BATT_GREEN 27
#define COMMAND_TICK 26
#endif

#define BATTERY_CHECK_LIMIT 10

// WiFi network name and password:
const char *networkName = "YourNetwork";
const char *networkPswd = "YourNetworkPassword";
// const char *networkName = "VUPlay";
// const char *networkPswd = "vuplay123";

//IP address to send UDP data to:
// either use the ip address of the server or
// a network broadcast address
const char *udpAddress = "192.168.1.199"; // replace with your host IP address where python is running
const int udpPort = 8889;
const int udpSrcPort = 3333;

GamepadPtr myGamepad;
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

int gpYaw = 0;
int gpThrottle = 0;
int gpSide = 0;
int gpStraight = 0;

String lastGpadCommand = "stop_v";
String gpadCommand = "stop_v";

//Are we currently connected?
boolean connected = false;
boolean lastCommandOK = false;
boolean in_flight = false;
boolean command_error = false;
boolean battery_checked = false;

uint8_t buffer[50];

//The udp library class
WiFiUDP udp;

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
      delay(1000);
      break;
    default:
      connected = false;
      break;
  }
}

void connectToWiFi(const char *ssid, const char *pwd) {
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
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
    //    Serial.println("One Tick");
    //    Serial.println(udpResponseDelay);
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
        } else if (battery == 2) {  // battery state near lowest allowed voltage
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
    if (lastCommandOK) digitalWrite(IN_FLIGHT, HIGH);
    in_flight = true;
  }
}

void setup() {
  // Initilize hardware serial:
  Serial.begin(115200);

  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2],
                addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
  BP32.forgetBluetoothKeys();

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

  //Connect to the WiFi network
  // connected = false;
  WiFi.mode(WIFI_STA);
  connectToWiFi(networkName, networkPswd);
}

void loop() {

  BP32.update();

  if (myGamepad && myGamepad->isConnected()) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...

    if (myGamepad->a() && myGamepad->y()) {  //press up and down at the same time
      ESP.restart();
    }

    if (myGamepad->l1()) {
      // Serial.println("Left Shoulder button pressed");
      onTakeoffButtonPressed();
      delay(250);
    }

    if (myGamepad->l2()) {
      // Serial.println("Left Paddle button pressed");
      run_command("battery", 20, 0);
      delay(250);
    }

    if (myGamepad->r1()) {
      // Serial.println("Right Shoulder button pressed");
      onKillButtonPressed();
      delay(250);
    }

    if (myGamepad->r2()) {
      // Serial.println("Right Paddle button pressed");
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

  lastGpadCommand = gpadCommand;
  gpadCommand = "stop_v";

  // process dpad and button values before joystick values
  if (in_flight) {
    if (myGamepad->a()) {
      gpadCommand = "down_v 20";
    } else if (myGamepad->b()) {
      gpadCommand = "cw_v 45";
    } else if (myGamepad->x()) {
      gpadCommand = "ccw_v 45";
    } else if (myGamepad->y()) {
      gpadCommand = "up_v 20";
    } else if (myGamepad->dpad() == 0x01) {
      gpadCommand = "go_fo_v 30 0";
    } else if (myGamepad->dpad() == 0x02) {
      gpadCommand = "go_ba_v -30 0";
    } else if (myGamepad->dpad() == 0x04) {
      gpadCommand = "go_ri_v 0 -30";
    } else if (myGamepad->dpad() == 0x05) {
      gpadCommand = "go_fr_v 30 -30";
    } else if (myGamepad->dpad() == 0x06) {
      gpadCommand = "go_br_v -30 -30";
    } else if (myGamepad->dpad() == 0x08) {
      gpadCommand = "go_le_v 0 30";
    } else if (myGamepad->dpad() == 0x09) {
      gpadCommand = "go_fl_v 30 30";
    } else if (myGamepad->dpad() == 0x0a) {
      gpadCommand = "go_bl_v -30 30";
    } else {  // no buttons or Dpad
      // check joystick values
      gpYaw = map(myGamepad->axisX(), -512, 512, -90, 90);       // degrees per second
      gpThrottle = map(myGamepad->axisY(), -512, 512, 50, -50);  // cm per second
      gpSide = map(myGamepad->axisRX(), -512, 512, 50, -50);
      gpStraight = map(myGamepad->axisRY(), -512, 512, 50, -50);
      // experimental dead zone values
      if (abs(gpYaw) <= 20) {
        gpYaw = 0;
      }
      if (abs(gpThrottle) <= 15) {
        gpThrottle = 0;
      }
      if (abs(gpStraight) <= 10) {
        gpStraight = 0;
      }
      if (abs(gpSide) <= 10) {
        gpSide = 0;
      }
      if ((gpStraight != 0) || (gpSide != 0) || (gpThrottle != 0) || (gpYaw != 0)) {  // use joystick values if some are non-zero
        gpadCommand = "go_v ";
        gpadCommand = gpadCommand + gpStraight + " " + gpSide + " " + gpThrottle + " " + gpYaw;
      }
    }
    if (lastGpadCommand != gpadCommand) {
      run_command(gpadCommand, 0, 0);
      // Serial.println(gpadCommand);
    }
  }

  vTaskDelay(150);
}
