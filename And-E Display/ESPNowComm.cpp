//Display
#include "ESPNowComm.h"

uint8_t AndE_MAC[6] = {0x3C, 0x8A, 0x1F, 0xA4, 0x15, 0xF8};
//24:EC:4A:01:41:4C

const char* ssid = "EUPHENDORE"; // Replace with your WiFi SSID
const char* password = "esp32ANDE"; // Replace with your WiFi Password

bool OTA_Mode = 0;
bool espnow_status = 0;

bool connectedToOTA = 0;


ControllerMessage PS4 = {};
SensorMessage AndE = {};
const int espchannel = 6;
int failedSends = 0;

void setupWireless(){

  WiFi.mode(WIFI_STA);
  Serial.print("ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());
  delay(2000);
  WiFi.disconnect();
  esp_wifi_set_channel(espchannel, WIFI_SECOND_CHAN_NONE);  // ✅ Set the same channel on both devices

  //esp_wifi_set_ps(WIFI_PS_MIN_MODEM);  // Enables Wi-Fi modem sleep to allow Bluetooth
    
  //esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
   // Set max Wi-Fi speed for ESP-NOW
    //esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_MAX); 

  //pinMode(LED_PIN, OUTPUT);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }
  esp_now_register_recv_cb(onReceive);  
  esp_now_register_send_cb(onSent);
  esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, AndE_MAC, 6);
    peerInfo.channel = espchannel;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer");
      return;
    }

  Serial.println("Receiver is ready to receive messages!");

  
}

void setupOTA() {
      // 🔥 Disable ESP-NOW
    esp_now_deinit();  // Turns off ESP-NOW before entering OTA

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);  // Replace with your network details

      

    delay(2000);
    Serial.println("Scanning for Wi-Fi networks...");
    int numNetworks = WiFi.scanNetworks();
    if (numNetworks == 0) {
        Serial.println("❌ No networks found!");
    } else {
        Serial.println("✅ Networks found:");
        for (int i = 0; i < numNetworks; i++) {
            Serial.printf("%d: %s (Channel: %d, RSSI: %d dBm)\n",
                          i + 1, WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i));
        }
    }
    Serial.println();

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    connectedToOTA = 1;
    delay(30);
    
    Serial.println("\nConnected to Wi-Fi.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    ArduinoOTA.setHostname(ssid);
    ArduinoOTA.setPassword(password);

    ArduinoOTA.onStart([]() { Serial.println("OTA Update Start"); });
    ArduinoOTA.onEnd([]() { Serial.println("\nOTA Update End"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress * 100) / total);
    });
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();
    Serial.println("OTA Ready.");
}

void onSent(const uint8_t *AndE_MAC, esp_now_send_status_t status) {
    // Serial.print("Message sent to: ");
    // for (int i = 0; i < 6; i++) {
    //     Serial.print(AndE_MAC[i], HEX);
    //     if (i < 5) Serial.print(":");
    // }

    // if (status == ESP_NOW_SEND_SUCCESS) {
    //     Serial.println(" - Delivery Success!");
    // } else {
    //     Serial.println(" - Delivery Failed!");
    // }
        if (status == ESP_NOW_SEND_SUCCESS) {
        Serial.println("ESP-NOW Send Success!");
        espnow_status = 1;
    } else {
        failedSends++;
        espnow_status = 0;
        //Serial.print("ESP-NOW Send Failed! Total Failures: ");
        //Serial.println(failedSends);
      }
}


void sendData(){

  esp_err_t result = esp_now_send(AndE_MAC, (uint8_t*)&PS4, sizeof(PS4));


}
void onReceive(const uint8_t* senderMAC, const uint8_t* incomingDataBytes, int len){

  //MessageType messageType = *(MessageType*)incomingDataBytes;  // Read first byte to determine messageType

   if (len == sizeof(AndE)) {  
    memcpy(&AndE, incomingDataBytes, sizeof(AndE));
    Serial.print("Received Data:");
    Serial.println(AndE.RPM);

  }
}