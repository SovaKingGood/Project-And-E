//Display
#include "Controller.h"
#include "Display.h"
#include "ESPNowComm.h"
#include <Arduino.h>
#include "esp_heap_caps.h"





//    Install ESP32 Board with Bluepad32 (*Bluepad32 is for connecting to PS4, Switch JoyCon, etc)
// ----------------------------------------------------------------------------------------------------
//    - Copy this link: 
//  https://raw.githubusercontent.com/ricardoquesada/esp32-arduino-lib-builder/master/bluepad32_files/package_esp32_bluepad32_index.json
//    - Go to: File -> Preferences -> Paste link in: Additional boards manager URLs 
//    - Go to: Tools -> Board -> Boards Manager.
//    - Search esp32 and install both "ESP32" by Espressif Systems and "esp32_bluepad32" by Ricardo Quesada
//    - Finally, go to: Tools -> Board: "esp32" -> and select "ESP32 Dev Module"  
static unsigned long lastTime = 0;
static unsigned long workTime = 0;

const int speakerPin = 25; // DAC1 pin (connected to 8002D IN+)

const int pwmOutput = 22; // DAC1 pin (connected to 8002D IN+)




static TaskHandle_t renderTaskHandle = nullptr;  

static void renderTask(void*) {
  setupDisplay(); // Initialize display configuration
  for(;;) {
    updateDisplay();                                
    vTaskDelay(pdMS_TO_TICKS(10));      // ~30 FPS + yield
  }
}


static void controllerTask(void*) {
  ledcSetup(0, 1000, 8);      // channel 0, 1kHz, 8-bit resolution
  ledcAttachPin(pwmOutput, 0);
  setupController(); //Listen for controllers
  for(;;) {
    pollController(); // Get data from bluepad32 controllers
    int pwmVALUE = map(PS4.throttle, 0, 1020, 0, 254);
    if(pwmVALUE <=10){
      pwmVALUE = 20;
    }
    ledcWrite(0, pwmVALUE);       // pwm to c2000                         
    vTaskDelay(pdMS_TO_TICKS(10));      
  }
}




void setup() {
  

  
  Serial.begin(115200)
  Serial.println("Booting");
  Serial.println("Opened Serial on 115200");
  
  
  
  Serial.println("Setting up wireless");
  setupWireless(); // Setup ESP-NOW to listen to And-E's electromagnetic radiation!


  


  

  Serial.println("Pinning UI Task, Priority 3, 8192 stack size, on Core 1");
  xTaskCreatePinnedToCore(renderTask, "Render", 8192, nullptr, 3, &renderTaskHandle, 1);
  

  Serial.println("Pinning Controllers Task, Priority 4, 8192 stack size, on Core 1");
  xTaskCreatePinnedToCore(controllerTask, "Controller", 8192, nullptr, 4, &renderTaskHandle, 1);

}

void loop() { //priority level = 1. (0-24, 24 is highest priority)

    // Record start of idle time
  unsigned long start = micros();




  
  if(OTA_Mode){
    if(WiFi.status() != WL_CONNECTED){
      connectedToOTA = 0;
    }
    else{
      connectedToOTA = 1;
    }
    ArduinoOTA.handle(); // **This must be constantly called**
  }
  else{
    sendData(); // Send data to And-E!
  }

  //updateDisplay();  // Update the contents of the Display

  



  // ---- End of work ----
  workTime += micros() - start;

  // Once per second, calculate CPU load
  unsigned long now = millis();
  if (now - lastTime >= 1000) {
    // Total time per second = 1,000,000 us
    float cpuLoad = (workTime / 10000.0);  // scale to %
    if (cpuLoad > 100) cpuLoad = 100;      // clamp at 100%

    Serial.printf("CPU Load: %.2f%%\n", cpuLoad);
    Serial.printf("Free heap: %d bytes\n",
                  heap_caps_get_free_size(MALLOC_CAP_8BIT));

    // Reset counters
    workTime = 0;
    lastTime = now;
  }


  delay(1);  
  // Must have a delay to run ESP32 backround tasks (priority 0) and reset the watchdog timer (ESP resets if timer exceeds 5 seconds)

}
