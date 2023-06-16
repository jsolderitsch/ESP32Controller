# GamePad Controllers

Documentation about GamePad controllers that work with the ESP32 controller. Information included here is based on preliminary testing with all three Tello types: White (original model), Tello Edu (Black) and Tello Talent (Red). The PCB and breadboard forms of the controller were used with each type of Tello and GamePad. 

This testing is in its early stages and more testing is ongoing.

## PS 4 Dualshock

![PS4](images/PS4Red.png)

Genuine PS 4 gamepads are among the most responsive when used to fly the Tello.

## Nimbus

![Nimbus](images/Nimbus.png)

Steelcase Nimbus gamepads are also very responsive when used to fly the Tello.

## PS 3 Xbox Clone

![PS3 XBox](images/PS3_Xbox.png)

The clone pictured above (no button markings) was advertised online as a PS 3 clone. But when connected using the bluepad32 demo, it self-identifies as an XBox gamepad when paired with the Y plus P3 button combination.

In general, this and other PS 3 gamepad clones do **NOT** perform as well as the PS 4 or Nimbus when used with the ESP32 controller.

A pair of these gamepads was bought on Amazon for less than $30.

## PS 3 Dualshock PS 3 clone

![PS3 Clone](images/PS3_Clone.png)

The PS 3 clone pictured above does identify itself as DualShock PS 3. As such, it must be **MANUALLY PAIRED** to match the Bluetooth MAC address of the ESP32 that you want to connect it to. How to do this is documented in the Bluepad32 project pages.

## GameSir T3s

![GameSir T3s](images/GameSir%20T3s.png)

This T3s GameSir identifies itself as a Switch Pro gamepad controller when paired using the Y plus Home button combination.

Switch Pro gamepads have not been tested extensively with the ESP32 Controller. This T3s performs on par with the PS 3 clone controllers that have been tested.

***Note:*** The GameSir T1d that was featured as an approved gamepad for use with the Tello and its Android and iOS apps is ***NOT*** supported by Bluepad32, the library used with the sketches included in this repository.
