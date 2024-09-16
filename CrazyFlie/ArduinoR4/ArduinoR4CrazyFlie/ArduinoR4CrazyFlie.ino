#include <WiFiS3.h>

//ip address of host running python
#define ip_address "192.168.1.199"

int status = WL_IDLE_STATUS;

// command strings for velocity control

String goCmd = "stop_v";
String lastGoCmd = "stop_v";

//LED variables
const int connected_blue_led = A5;
const int not_connected_yellow_led = A4;

//button variables
const int First_routine_button = 2;
const int Second_routine_button = A3;
const int takeoff_button = A2;
const int land_button = A1;
const int up_button = 4;
const int down_button = A0;
const int forward_button = 5;
const int back_button = 7;
const int turn_left_button = 8;
const int turn_right_button = 6;
const int kill_button = 12;
const int batt_button = 11;
const int left_button = 10;
const int right_button = 9;

//flag to run certain commands once when connected to drone while in the main loop
bool wfi_connected = false;
bool connected = false;

//delay between button presses
const int pause_interval = 4000;

//wifi network name from host WiFi
const char* ssid = "YourNetwork";
const char* password = "YourNetworkPassword";


//udp port
const int port = 8889;
const int srcPort = 3333;
//attach object
WiFiUDP Udp;
uint8_t buffer[50];

boolean in_flight = false;

bool runCommand(String, int = 40);

void toggle_led(int ledToToggle) {
  // Toggle the state of the LED pin (write the NOT of the current state to the LED pin)
  digitalWrite(ledToToggle, !digitalRead(ledToToggle));
}

bool runCommand(String command, int delayTicks) {
  bool lastCommandOK = false;
  digitalWrite(not_connected_yellow_led, HIGH);
  int packetSize = 0;
  boolean responseExpected = true;

  // Special delay cases
  if (command.indexOf("_v") >= 0) {  // Velocity commands return immediately
    delayTicks = 0;
    responseExpected = false;
    digitalWrite(not_connected_yellow_led, LOW);
  } else {  // don't log _v commands to serial
    Serial.print("CrazyFlie Command: ");
    Serial.println(command);
  }

  memset(buffer, 0, 50);
  command.getBytes(buffer, command.length() + 1);
  //Send a packet
  Udp.beginPacket(ip_address, port);
  Udp.write(buffer, command.length() + 1);
  Udp.endPacket();
  // Serial.println("endPacket called");
  memset(buffer, 0, 50);
  for (int x = 0; x < delayTicks; x++) {
    packetSize = Udp.parsePacket();
    if (packetSize) break;
    delay(500);
    toggle_led(not_connected_yellow_led);
  }
  if (packetSize && responseExpected) {
    //    Serial.print("packet size: ");
    //    Serial.println(packetSize);
    if (Udp.read(buffer, 50) > 0) {
      String commandResponse = String((char*)buffer);
      Serial.print("Drone Command Response: ");
      Serial.println(commandResponse.substring(0, commandResponse.length() - 2));
      lastCommandOK = true;
    }
  } else if (responseExpected) {
    //   // no response received
    lastCommandOK = false;
  } else {
    lastCommandOK = true;
  }
  digitalWrite(not_connected_yellow_led, LOW);
  return lastCommandOK;
}

void firstFlight() {
  // Take off
  runCommand("takeoff");
  // up 50
  runCommand("up 50");
  // cw 90
  runCommand("cw 90");
  // forward 50
  runCommand("forward 50");
  // back 50
  runCommand("back 50");
  // ccw 90
  runCommand("ccw 90");
  // land
  runCommand("land");
}

void SecondFlight() {
  // Take off
  runCommand("takeoff");
  runCommand("up 20");
  runCommand("down 20");
  runCommand("right 50");
  runCommand("left 50");
  runCommand("land");
}

void circleFlight() {
  // Take off
  runCommand("takeoff");
  runCommand("CW_circ 30 30 180");
  runCommand("CCW_circ 30 30 180");
  runCommand("land");
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void connectToNetwork(const char* ssid, const char* pwd) {
  Serial.println("Connecting to network: " + String(ssid));
  WiFi.disconnect();  // start with clean slate
  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, password);
    // status = WiFi.begin(ssid);
    // wait .5 seconds for connection; then try again
    delay(500);
  }
  Serial.println("Connected to WiFi; Ready to send drone commands");
  if (status == WL_CONNECTED) {
    connected = true;
    Udp.begin(WiFi.localIP(), srcPort);
    delay(1000);
    digitalWrite(connected_blue_led, HIGH);
    digitalWrite(not_connected_yellow_led, LOW);
    printWifiStatus();
  }
  //  Serial.println("Waiting for WIFI connection...");
}


void setup() {
  // Initilize hardware serial:
  Serial.begin(115200);

  //set all pin modes. All buttons use the internal pullup resistors.

  pinMode(connected_blue_led, OUTPUT);
  pinMode(not_connected_yellow_led, OUTPUT);
  pinMode(First_routine_button, INPUT_PULLUP);
  pinMode(Second_routine_button, INPUT_PULLUP);
  pinMode(takeoff_button, INPUT_PULLUP);
  pinMode(land_button, INPUT_PULLUP);
  pinMode(up_button, INPUT_PULLUP);
  pinMode(down_button, INPUT_PULLUP);
  pinMode(forward_button, INPUT_PULLUP);
  pinMode(back_button, INPUT_PULLUP);
  pinMode(turn_left_button, INPUT_PULLUP);
  pinMode(turn_right_button, INPUT_PULLUP);
  pinMode(kill_button, INPUT_PULLUP);
  pinMode(batt_button, INPUT_PULLUP);
  pinMode(left_button, INPUT_PULLUP);
  pinMode(right_button, INPUT_PULLUP);

  //turn the yellow LED on and blue off because wifi not connected
  digitalWrite(not_connected_yellow_led, HIGH);
  digitalWrite(connected_blue_led, LOW);

  //Connect to the WiFi network
  connectToNetwork(ssid, password);
}

void loop() {
  //  status = WiFi.status();
  if (WiFi.status() != WL_CONNECTED) {
    //turn the yellow LED on and blue off because wifi not connected
    if (digitalRead(connected_blue_led)) {
      digitalWrite(not_connected_yellow_led, HIGH);
      digitalWrite(connected_blue_led, LOW);
      Serial.print("Lost connection to Network: ");
      Serial.println(ssid);
      Serial.println("Reconnecting...");
      status = WL_DISCONNECTED;
      connectToNetwork(ssid, password);
    }
  } else {
    //read the state of each button and store the value in a new variable
    int First_routine_button_state = digitalRead(First_routine_button);
    int Second_routine_button_state = digitalRead(Second_routine_button);
    int takeoff_button_state = digitalRead(takeoff_button);
    int land_button_state = digitalRead(land_button);
    int up_button_state = digitalRead(up_button);
    int down_button_state = digitalRead(down_button);
    int forward_button_state = digitalRead(forward_button);
    int back_button_state = digitalRead(back_button);
    int turn_left_button_state = digitalRead(turn_left_button);
    int turn_right_button_state = digitalRead(turn_right_button);
    int batt_button_state = digitalRead(batt_button);
    int kill_button_state = digitalRead(kill_button);
    int left_button_state = digitalRead(left_button);
    int right_button_state = digitalRead(right_button);

    //check each button. If the button is low (pressed) it will execute the code in the if statement

    if ((takeoff_button_state == LOW) && !in_flight) {
      in_flight = runCommand("takeoff");
    } else if ((land_button_state == LOW) && in_flight) {
      in_flight = !runCommand("land");  // successful land means not in flight
    } else if ((kill_button_state == LOW) && in_flight) {
      runCommand("emergency");
      in_flight = false;
    } else if ((kill_button_state == LOW) && !in_flight) {
      NVIC_SystemReset();  // reset controller
    } else if (batt_button_state == LOW) {
      runCommand("battery");
    } else {
      lastGoCmd = goCmd;  // save last command
      goCmd = "stop_v";   // default is hover

      if (in_flight) {
        if (up_button_state == LOW) {
          goCmd = "up_v 20";
        } else if (down_button_state == LOW) {
          goCmd = "down_v 20";
        } else if (forward_button_state == LOW) {
          goCmd = "go_v 30 0 0 0";
        } else if (back_button_state == LOW) {
          goCmd = "go_v -30 0 0 0";
        } else if (turn_left_button_state == LOW) {
          goCmd = "ccw_v 45";
        } else if (turn_right_button_state == LOW) {
          goCmd = "cw_v 45";
        } else if (left_button_state == LOW) {
          goCmd = "go_v 0 30 0 0";
        } else if (right_button_state == LOW) {
          goCmd = "go_v 0 -30 0 0";
        }
        if (lastGoCmd != goCmd) {
          runCommand(goCmd);
        }
      } else {  // not in flight
        if (First_routine_button_state == LOW) {
          firstFlight();
        } else if (Second_routine_button_state == LOW) {
          circleFlight();
        }
      }
    }
  }
}
