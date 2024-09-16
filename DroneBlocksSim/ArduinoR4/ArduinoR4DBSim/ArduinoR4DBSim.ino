#include <WiFiS3.h>

//ip address of host running python
// #define ip_address "192.168.1.199"
#define ip_address "192.168.1.14"

int status = WL_IDLE_STATUS;

String goCmd = "";

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
const int flip_button = 11;
const int left_button = 10;
const int right_button = 9;

//flag to run certain commands once when connected to drone while in the main loop
bool wfi_connected = false;
bool connected = false;

//delay between button presses
const int pause_interval = 4000;

//udp commands in an array
char* drone_commands[15] = { "command", "takeoff", "land", "up 50", "cw 90", "forward 50", "ccw 90",
                             "flip b", "flip f", "flip l", "flip r", "up 50", "up 20", "down 50", "down 20" };

//wifi network name from host WiFi
// const char* ssid = "Arlington";
// const char* password = "MontgomeryBurns!6002ear";
const char* ssid = "LindaJoon";
const char* password = "MontgomeryBurns!";

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
  bool responseReceived = false;
  digitalWrite(not_connected_yellow_led, HIGH);
  int packetSize = 0;
  Serial.print("Drone Command: ");
  Serial.println(command);
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
  if (packetSize) {
    //    Serial.print("packet size: ");
    //    Serial.println(packetSize);
    if (Udp.read(buffer, 50) > 0) {
      String commandResponse = String((char*)buffer);
      Serial.print("Drone Command Response: ");
      Serial.println(commandResponse.substring(0, commandResponse.length() - 2));
    }
    responseReceived = true;
  } else {
    // NVIC_SystemReset();  // no response received
    responseReceived = false;
  }
  digitalWrite(not_connected_yellow_led, LOW);
  return responseReceived;
}

void firstFlight() {
  runCommand("takeoff");
  runCommand("forward");
  runCommand("left");
  runCommand("back");
  runCommand("right");
  runCommand("flip");
  runCommand("land");
}

void circleFlight() {
  runCommand("takeoff");
  runCommand("curve 50 50 0 0 100 0"); // cm
  runCommand("curve -50 -50 0 0 -100 0"); // cm
  runCommand("land");
}

void secondFlight() {
  // Take off
  runCommand(drone_commands[1]);
  // up 50
  runCommand(drone_commands[11]);
  // up 20
  runCommand(drone_commands[12]);
  // down 20
  runCommand(drone_commands[13]);
  // up 20
  runCommand(drone_commands[12]);
  // down 20
  runCommand(drone_commands[14]);
  // land
  runCommand(drone_commands[2]);
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
  pinMode(flip_button, INPUT_PULLUP);
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
    int flip_button_state = digitalRead(flip_button);
    int kill_button_state = digitalRead(kill_button);
    int left_button_state = digitalRead(left_button);
    int right_button_state = digitalRead(right_button);

    //check each button. If the button is low (pressed) it will execute the code in the if statement
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
    } else {
      if (in_flight) {
        goCmd = "none";
        if (up_button_state == LOW) {
          goCmd = "up";
        } else if (down_button_state == LOW) {
          goCmd = "down";
        } else if (forward_button_state == LOW) {
          goCmd = "forward";
        } else if (back_button_state == LOW) {
          goCmd = "back";
        } else if (turn_left_button_state == LOW) {
          goCmd = "ccw";
        } else if (turn_right_button_state == LOW) {
          goCmd = "cw";
        } else if (left_button_state == LOW) {
          goCmd = "left";
        } else if (right_button_state == LOW) {
          goCmd = "right";
        } else if (flip_button_state == LOW) {  // reassign to flip backward
          goCmd = "flip";
        } 
        if (goCmd != "none") {
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
