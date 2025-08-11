//Display
#include "Controller.h"
#include "ESPNowComm.h"
#include "Display.h"

ControllerPtr myControllers[1];

int OTAswap_timer = 0;
bool OTAswap_flag = 0;

void setupController() {


  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);



  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
}


void pollController() {

  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    for (auto myController : myControllers) {
      if (myController && myController->isConnected() && myController->hasData()) {


        if (myController->isGamepad()) {

          PS4.throttle = myController->throttle();


          if (myController->throttle() >= 900) {
            myController->playDualRumble(0, 2000, 0x80, 0x80);
          } 
          else {

            if (myController->a() == 1) { //x
              myController->playDualRumble(0, 2000, 0x80, 0x40);

              if (OTAswap_flag == 0) {
                OTAswap_timer = millis();
                OTAswap_flag = 1;

                }
            } 
            else {
            OTAswap_flag = 0;
            myController->playDualRumble(0, 0, 0, 0);
            }
            if (OTAswap_flag == 1 && millis() - OTAswap_timer >= 2000) {
                OTAswap_flag = 0;

                OTA_Mode = 1;

                Serial.println("Switching to OTA Mode...");
                updateDisplay();  // Update the contents of the Display
                setupOTA();
            }

          }
          
      
          PS4.brake = myController->brake();

          //  ctl->index(),        // Controller Index
          //  ctl->dpad(),         // D-pad
          //  ctl->buttons(),      // bitmask of pressed buttons
          // ctl->axisX(),        // (-511 - 512) left X Axis
          //  ctl->axisY(),        // (-511 - 512) left Y axisS
          //  ctl->axisRX(),       // (-511 - 512) right X axis
          //  ctl->axisRY(),       // (-511 - 512) right Y axis
          //brake = ctl->brake();        // (0 - 1023): brake button
          // int throttle = ctl->throttle();     // (0 - 1023): throttle (AKA gas) button
          //  ctl->miscButtons(),  // bitmask of pressed "misc" buttons
          //  ctl->gyroX(),        // Gyro X
          //  ctl->gyroY(),        // Gyro Y
          // ctl->gyroZ(),        // Gyro Z
          //  ctl->accelX(),       // Accelerometer X
          //  ctl->accelY(),       // Accelerometer Y
          //  ctl->accelZ(),        // Accelerometer Z



          PS4.left_stick_horizontal = myController->axisX();
          PS4.left_stick_vertical = myController->axisY();


          PS4.battery = myController->battery(); 


          


          if (myController->b() == 1) {  // circle
            Serial.println("Switching to ESP-NOW...");

            //delay(1000);
            OTA_Mode = 0;
            updateDisplay();  // Update the contents of the Display
            setupWireless();
          }


          // if (myController->r1()) {
          //   OTAswap_flag = 0;

          //   OTA_Mode = 1;

          //   Serial.println("Switching to OTA Mode...");
          //   setupOTA();
          // }
        }
      }
    }
  }
}


// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
  PS4.throttle = 0;
}
