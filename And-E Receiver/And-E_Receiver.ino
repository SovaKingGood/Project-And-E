#include "FFT.h"
#include "ESPNowComm.h"

// 1️⃣ Install ESP32 Board
//    - Go to: Tools → Board → Boards Manager.
//    - Search esp32 and install "esp32" by Espressif Systems and install it.
//    - Go to: Tools -> Board: "esp32" -> "ESP32 Dev Module"







// Arduino setup function. Runs in CPU 1
void setup() {

  Serial.begin(115200);

  setupWireless();
  setupFFT();



   xTaskCreatePinnedToCore(
     FFT_Task, "FFT_Task", 11000, NULL, 1, NULL, 1  // Run FFT on Core 1 ESP-NOW is Core 0
   );
  

}



// Arduino loop function. Runs in CPU 1.
void loop() {
   

    sendData();
    //Serial.print("test");
    //esc.writeMicroseconds(pwmValue);

    delay(100);
}


    
