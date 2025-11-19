#include "FFT.h"
#include "ESPNowComm.h"


float *vReal = nullptr;
float *vImag = nullptr;
int16_t *tempBuffer = nullptr;
//float vReal[N];
//float vImag[N];
ArduinoFFT<float>* FFT_ptr = nullptr; 
//ArduinoFFT<float> FFT(vReal, vImag, N, SAMPLE_RATE);
//int16_t tempBuffer[N];
int sampleCount = 0;

// ESP-NOW Data Structure
typedef struct {
    float peakFreq;
} FFT_Data;
FFT_Data fftData;



void setupFFT(){
  pinMode(PULSE_PIN, INPUT_PULLUP);

  
  vReal = (float *)malloc(N * sizeof(float));
  vImag = (float *)malloc(N * sizeof(float));
  tempBuffer = (int16_t *)malloc(N * sizeof(int16_t));

  if (!vReal || !vImag || !tempBuffer) {
    Serial.println("Memory allocation failed!");
    // Handle the error, e.g., by entering an error loop.
  } else {
    Serial.println("Memory allocation successful.");
  }
    
  FFT_ptr = new ArduinoFFT<float>(vReal, vImag, N, SAMPLE_RATE);

  setupADC();
  


}

// Set up ADC based on board type
void setupADC() {
  #if defined(CONFIG_IDF_TARGET_ESP32S3)
    Serial.println("ESP32-S3 detected: Using analogRead for ADC sampling.");
    // No I2S initialization here – we will use analogRead in the task.
  #else
    // I2S ADC initialization for legacy ESP32 boards
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = DMA_BUF_LEN
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_adc_mode(ADC_UNIT_1, ADC_CHANNEL);
    i2s_adc_enable(I2S_NUM_0);
  #endif
}

void removeUnwantedFrequencies(float *vReal, int lowCutoffHz, int highCutoffHz) {
    int lowCutoffBin = (lowCutoffHz * N) / SAMPLE_RATE;  // Convert Hz to bin index
    int highCutoffBin = (highCutoffHz * N) / SAMPLE_RATE; // Convert Hz to bin index

    for (int i = 0; i <= lowCutoffBin; i++) {
        vReal[i] = 0;  // Remove DC and low frequencies
    }
    for (int i = highCutoffBin; i < N / 2; i++) {  // N/2 because Nyquist limit
        vReal[i] = 0;  // Remove high frequencies
    }
}

volatile float peakFreq;

// FFT Task: Samples ADC and performs FFT processing
void FFT_Task(void *parameter) {
  #if defined(CONFIG_IDF_TARGET_ESP32S3)
    // ESP32-S3: Use analogRead in a loop to sample the ADC
    for (;;) {
      if (sampleCount < N) {
         // Read ADC using analogRead; note that this is blocking and may not hit exactly 20kHz
         tempBuffer[sampleCount++] = analogRead(ADC_PIN);
         //delayMicroseconds(50);  // ~50us delay approximates 20kHz sampling
      } else {
         // Prepare data for FFT: scale 12-bit ADC value to voltage
         for (int i = 0; i < N; i++) {
             vReal[i] = (tempBuffer[i] & 0x0FFF) * 3.3 / 4096.0;
             vImag[i] = 0;
         }
         FFT_ptr->windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
         FFT_ptr->compute(FFT_FORWARD);
         FFT_ptr->complexToMagnitude();
         removeUnwantedFrequencies(vReal, 100, 1500);
         peakFreq = getPeakFrequency(vReal);

         // Example processing: convert frequency to RPM (assumes one pulse per revolution)
         AndE.RPM = peakFreq;  // AndE should be defined elsewhere

         sampleCount = 0;
         vTaskDelay(pdMS_TO_TICKS(300)); 
      }
    }
  #else
    // Non-ESP32-S3: Use I2S DMA for ADC sampling
    int16_t dmaBuffer[DMA_BUF_LEN];
    size_t bytes_read;
    for (;;) {
      if (i2s_read(I2S_NUM_0, dmaBuffer, sizeof(dmaBuffer), &bytes_read, portMAX_DELAY) == ESP_OK) {
         int samples = bytes_read / sizeof(int16_t);
         for (int i = 0; i < samples && sampleCount < N; i++) {
           tempBuffer[sampleCount++] = dmaBuffer[i];
         }
         if (sampleCount >= N) {
            for (int i = 0; i < N; i++) {
               vReal[i] = (tempBuffer[i] & 0x0FFF) * 3.3 / 4096.0;
               vImag[i] = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(1)); 
            FFT_ptr->windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
            vTaskDelay(pdMS_TO_TICKS(1)); 
            FFT_ptr->compute(FFT_FORWARD);
            vTaskDelay(pdMS_TO_TICKS(1)); 
            FFT_ptr->complexToMagnitude();
            vTaskDelay(pdMS_TO_TICKS(1)); 
            removeUnwantedFrequencies(vReal, 100, 1100);
            peakFreq = getPeakFrequency(vReal);
            AndE.RPM = peakFreq;
            sampleCount = 0;
            vTaskDelay(pdMS_TO_TICKS(300)); 
         }
      }
    }
  #endif
}

// Find Peak Frequency
float getPeakFrequency(float *vReal) {
    int peakIndex = 0;
    float peakValue = 0;

    for (int i = 1; i < N / 2; i++) {  // Only check first half (Nyquist limit)
        if (vReal[i] > peakValue && vReal[i] > 10) {
            peakValue = vReal[i];
            peakIndex = i;
        }
    }
    //Serial.print("Peak Frequency:");
    //Serial.print((peakIndex * ((float)SAMPLE_RATE /N)));
    //Serial.print("Value:");
    //Serial.println(peakValue);
    //Serial.print("Peak Freq Amp:");
    //Serial.println(peakValue);
    return peakIndex * ((float)SAMPLE_RATE /(N));  // Convert index to frequency
}

