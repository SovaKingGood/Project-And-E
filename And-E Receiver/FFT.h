#include "driver/i2s.h"
#include <arduinoFFT.h>


#define PULSE_PIN 26 // GPIO for RPM signal
#define N 1024  // FFT Size (Use 256 or 512 for real-time performance) 1024 2048 4096
#define SAMPLE_RATE 20000   // Sampling rate in Hz
// For non-ESP32-S3 boards, use ADC1 channel 6 (GPIO34)
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  #define ADC_PIN 1         // Use analogRead on GPIO34 for ESP32-S3
#else
  #define ADC_CHANNEL ADC1_CHANNEL_6  // For boards that support I2S ADC mode
#endif

#define DMA_BUF_LEN 1024  // smaller DMA buffer length

void setupFFT();
float getPeakFrequency(float *vReal);
void FFT_Task(void *parameter);
void removeUnwantedFrequencies(float *vReal, int lowCutoffHz, int highCutoffHz);
void setupADC();