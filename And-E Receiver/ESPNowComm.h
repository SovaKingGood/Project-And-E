#include <WiFi.h>
#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <driver/ledc.h>  
#include <ESP32Servo.h>

//What we receive
typedef struct __attribute__((packed)){
  int throttle;
  int brake;
  float battery;
  int left_stick_horizontal;
  int left_stick_vertical;

} ControllerMessage;


//What we send

typedef struct __attribute__((packed)){
  int RPM;
  float Temps;
  float Voltage;
  float Accel_x;

} SensorMessage;

extern ControllerMessage PS4;
extern SensorMessage AndE;

extern int pwmValue;

extern Servo esc;
extern Servo steer;
extern Servo wing;



extern uint8_t Display_MAC[6]; 

void onSent(const uint8_t *Display_MAC, esp_now_send_status_t status);
void onReceive(const esp_now_recv_info* info  , const uint8_t* incomingDataBytes, int len);
void sendData();
void setupWireless();
void sendControlData();

