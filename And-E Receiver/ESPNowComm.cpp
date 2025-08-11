#include "ESPNowComm.h"




uint8_t Display_MAC[6] = {0xCC, 0xDB, 0xA7, 0x9E, 0x20, 0xDC}; 

ControllerMessage PS4 = {};
SensorMessage AndE = {};

// Define the PWM output pin for the ESC (adjust pin number as needed)
const int escPin = 4; 
const int steerPin = 5;
const int wingPin = 18;
const int espchannel = 6;

// Create a Servo object to control the ESC
Servo esc;
Servo steer;
Servo wing;

int pwmValue;
int steerValue;
int wingValue;




 void setupWireless(){

  esc.attach(escPin, 1000, 2000);
  Serial.println("ESC pin attached");
  esc.writeMicroseconds(1000);
  delay(2000); // Wait for the ESC to arm


  steer.attach(steerPin, 1000, 2000);
  Serial.println("Steering pin attached");
  steer.writeMicroseconds(1500);
  delay(2000); // Wait for the ESC to arm

  wing.attach(wingPin, 1000, 2000);
  Serial.println("Wing pin attached");
  wing.writeMicroseconds(1500);
  delay(2000); // Wait for the ESC to arm




  // ledcSetup(0, 50, 16);
  // ledcAttachPin(escPin, 0);
  



  WiFi.mode(WIFI_STA);
  delay(200);
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
  WiFi.disconnect();
   //   esp_wifi_set_ps(WIFI_PS_MIN_MODEM);  // Enables Wi-Fi modem sleep to allow Bluetooth
  esp_wifi_set_channel(espchannel, WIFI_SECOND_CHAN_NONE);  // ✅ Set the same channel on both devices

  //esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
   // Set max Wi-Fi speed for ESP-NOW
   // esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_MAX); 
  

  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  esp_now_register_recv_cb(onReceive);
  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, Display_MAC, 6);
  peerInfo.channel = espchannel;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }


 }


 void onReceive(const esp_now_recv_info* info , const uint8_t* incomingDataBytes, int len){

  //Serial.println("📡 Receiving data...");
    
    // Print sender MAC address
    // Serial.print("From MAC: ");
    // for (int i = 0; i < 6; i++) {
    //     Serial.print(senderMAC[i], HEX);
    //     if (i < 5) Serial.print(":");
    // }
    // Serial.println();

   if (len == sizeof(PS4)) {  
    memcpy(&PS4, incomingDataBytes, sizeof(PS4));
    //unsigned long receiveTime = millis();
    //Serial.print("ESP-NOW Packet Received at: ");
    //Serial.println(receiveTime);
    
    if(PS4.brake >= 10){
      //pwmValue = map(PS4.brake, 0, 1023, 4915, 3277); 
      pwmValue = map(PS4.brake, 0, 1023, 1500, 1000); // 1000 = 3277 1500 = 4915 2000 = 6553
    }
    else{
      pwmValue = map(PS4.throttle, 0, 1023, 1500, 2000);
      //pwmValue = map(PS4.throttle, 0, 1023, 4915, 6553);
    }
    }
    else {
      Serial.println("wtf");
    }
    esc.writeMicroseconds(pwmValue);

    Serial.print("Stick: ");
    Serial.println(PS4.left_stick_horizontal);
    steerValue = map(PS4.left_stick_horizontal, -511, 512, 1000, 2000);
    steer.writeMicroseconds(steerValue);
    Serial.print("steerValue:");
    Serial.println(steerValue);

    Serial.print("Wing: ");
    Serial.println(PS4.left_stick_vertical);
    wingValue = map(PS4.left_stick_vertical, -511, 512, 1000, 2000);
    wing.writeMicroseconds(wingValue);
    Serial.print("wingValue:");
    Serial.println(wingValue);
    

    //ledcWrite(0, pwmValue);
    //Serial.print("pwmValue: "); Serial.println(pwmValue);

    // Print received values
         //Serial.print("Throttle: "); Serial.println(PS4.throttle);
        // Serial.print("Brake: "); Serial.println(PS4.brake);
        // Serial.print("Battery: "); Serial.println(PS4.battery);
     
}


 void sendData() {
  
  esp_err_t result = esp_now_send(Display_MAC, (uint8_t*)&AndE, sizeof(AndE));


}


void onSent(const uint8_t *Display_MAC, esp_now_send_status_t status) {
    Serial.print("Message sent to: ");
    for (int i = 0; i < 6; i++) {
        Serial.print(Display_MAC[i], HEX);
        if (i < 5) Serial.print(":");
    }

    if (status == ESP_NOW_SEND_SUCCESS) {
        Serial.println(" - Delivery Success!");
    } else {
        Serial.println(" - Delivery Failed!");
    }
}