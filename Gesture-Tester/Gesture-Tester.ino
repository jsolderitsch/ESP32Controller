/* Get tilt angles on X and Y, and rotation angle on Z
 * Angles are given in degrees
 * 
 * License: MIT
 */

#include "Wire.h"
#include <MPU6050_light.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>

// LED configs
#define LED_FORWARD          26
#define LED_BACK             25
#define LED_RIGHT            27
#define LED_LEFT             15

MPU6050 mpu(Wire);
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);
unsigned long timer = 0;

int Roll;
int Pitch;
int Yaw;

void setup(void) {
  Serial.begin(115200);
  // Serial.setTimeout(0);
  Wire.begin();
  
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
  Serial.println(status);
  display.println("MPU6050 status: ");
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

  pinMode(LED_FORWARD, OUTPUT);
  pinMode(LED_BACK, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);
  pinMode(LED_LEFT, OUTPUT);
  digitalWrite(LED_FORWARD, LOW);
  digitalWrite(LED_BACK, LOW);
  digitalWrite(LED_RIGHT, LOW);
  digitalWrite(LED_LEFT, LOW);
}

void loop() {

  mpu.update();
  display.clearDisplay();
  display.setCursor(0, 0);
  Roll = mpu.getAngleX();
  Pitch = mpu.getAngleY();
  Yaw = mpu.getAngleZ();

  if (abs(Roll) <= 10) {
    digitalWrite(LED_LEFT,LOW);
    digitalWrite(LED_RIGHT,LOW);
  }

  if (abs(Pitch) <= 15) {
    digitalWrite(LED_FORWARD,LOW);
    digitalWrite(LED_BACK,LOW);
  }

  if (Pitch < -16) {
    digitalWrite(LED_FORWARD,HIGH);
  }

  if (Pitch > 16) {
    digitalWrite(LED_BACK,HIGH);
  }
  
  if (Roll < -11) {
    digitalWrite(LED_RIGHT,HIGH);
  }
  
  if (Roll > 11) {
    digitalWrite(LED_LEFT,HIGH);
  }

  if((millis()-timer)>100){ // print data every 100ms
    display.print("Roll: ");
    display.println(Roll);
    display.print("Pitch: ");
    display.println(Pitch);
    display.print("Yaw: ");
    display.println(Yaw);
    display.display();
    
    timer = millis();  
  }

}
