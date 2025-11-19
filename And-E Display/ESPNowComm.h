//Display
#include <esp_now.h>
#include <esp_wifi.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>

#include <WiFi.h>

extern uint8_t AndE_MAC[6];

void setupWireless();
void sendData();
void setupOTA();

#define LED_PIN 2  // Assuming the blue LED is connected to GPIO 2


//This is to upload to the board wirelessly
// Wi-Fi credentials
extern const char* ssid;
extern const char* password;
extern bool OTA_Mode;
extern bool espnow_status;
extern int failedSends;
extern bool connectedToOTA;





enum MessageType {
  MISC_DATA,
  CONTROLLERRR_DATA,
  ERRORS
};

extern int fillWidth;

//What we send
typedef struct __attribute__((packed)) {
  int throttle;
  int brake;
  float battery;
  int left_stick_horizontal;
  int left_stick_vertical;

} ControllerMessage;


//What we receive

typedef struct __attribute__((packed)) {
  int RPM;
  float Temps;
  float Voltage;
  float Accel_x;

} SensorMessage;

extern ControllerMessage PS4;
extern SensorMessage AndE;




// ESP-NOW receive callback
void onReceive(const uint8_t* senderMAC, const uint8_t* incomingDataBytes, int len);
void onSent(const uint8_t *AndE_MAC, esp_now_send_status_t status);