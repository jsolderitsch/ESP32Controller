#include <Bluepad32.h>

// Define LED Pins
#define LED1_PIN 26
#define LED2_PIN 25
#define LED3_PIN 21
#define LED4_PIN 27
#define LED5_PIN 15
#define LED6_PIN 4

// Variables to hold LED states
bool led1State = false;
bool led2State = false;
bool led3State = false;
bool led4State = false;
bool led5State = false;
bool led6State = false;

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myGamepads[i] == nullptr) {
      Serial.printf("CALLBACK: Gamepad is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      GamepadProperties properties = gp->getProperties();
      Serial.printf("Gamepad model: %s, VID=0x%04x, PID=0x%04x\n",
                    gp->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myGamepads[i] = gp;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println(
        "CALLBACK: Gamepad connected, but could not found empty slot");
  }
}

void onDisconnectedGamepad(GamepadPtr gp) {
  bool foundGamepad = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myGamepads[i] == gp) {
      Serial.printf("CALLBACK: Gamepad is disconnected from index=%d\n", i);
      myGamepads[i] = nullptr;
      foundGamepad = true;
      break;
    }
  }

  if (!foundGamepad) {
    Serial.println(
        "CALLBACK: Gamepad disconnected, but not found in myGamepads");
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {
  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2],
                addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But might also fix some connection / re-connection issues.
  // BP32.forgetBluetoothKeys();
  // Set LED pins as outputs

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);
  pinMode(LED5_PIN, OUTPUT);
  pinMode(LED6_PIN, OUTPUT);

  // Print to Serial Monitor
  Serial.println("Ready.");

}

// Arduino loop function. Runs in CPU 1
void loop() {
  // This call fetches all the gamepad info from the NINA (ESP32) module.
  // Just call this function in your main loop.
  // The gamepads pointer (the ones received in the callbacks) gets updated
  // automatically.
  BP32.update();

  // It is safe to always do this before using the gamepad API.
  // This guarantees that the gamepad is valid and connected.
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    GamepadPtr myGamepad = myGamepads[i];

    if (myGamepad && myGamepad->isConnected()) {
      // There are different ways to query whether a button is pressed.
      // By query each button individually:
      //  a(), b(), x(), y(), l1(), etc...
      if (myGamepad->a()) {
        // Serial.println("A button pressed");
        // delay(250);
        vTaskDelay(300);
        led6State = !led6State;
        digitalWrite(LED6_PIN, led6State);
      }

      if (myGamepad->b()) {
        // Serial.println("Circle button pressed");
        // delay(250);
        vTaskDelay(300);
        led2State = !led2State;
        digitalWrite(LED2_PIN, led2State);
      }

      if (myGamepad->x()) {
        // Serial.println("Square button pressed");
        led3State = true;
        digitalWrite(LED3_PIN, led3State);
      }

      if (myGamepad->y()) {
        // Serial.println("Triangle button pressed");
        led3State = false;
        digitalWrite(LED3_PIN, led3State);
      }

      if (myGamepad->r1()) {
        // Serial.println("Right button pressed");
        vTaskDelay(300);
        led4State = !led4State;
        digitalWrite(LED4_PIN, led4State);
      }

      if (myGamepad->l1()) {
        // Serial.println("Left button pressed");
        vTaskDelay(300);
        led5State = !led5State;
        digitalWrite(LED5_PIN, led5State);
      }

      if (myGamepad->dpad() == 0x01) {
        if (!led1State) Serial.println("DPAD up held");
        led1State = true;
        digitalWrite(LED1_PIN, led1State);
      }
      if (myGamepad->dpad() == 0x00) {
        if (led1State) Serial.println("DPAD released");
        led1State = false;
        digitalWrite(LED1_PIN, led1State);
      }
    }
  }

  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  vTaskDelay(1);
  // delay(200);
}
