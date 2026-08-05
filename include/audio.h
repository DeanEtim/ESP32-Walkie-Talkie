#ifndef AUDIO_H
#define AUDIO_H

#include <Arduino.h>

// I2S Microphone Pins
constexpr uint8_t MIC_BCLK_PIN = 13;
constexpr uint8_t MIC_WS_PIN = 14;
constexpr uint8_t MIC_SD_PIN = 12;

// I2S Speaker Pins
constexpr uint8_t SPK_BCLK_PIN = 27;
constexpr uint8_t SPK_WS_PIN = 26;
constexpr uint8_t SPK_SD_PIN = 25;

// Audio Configuration
constexpr uint32_t SAMPLE_RATE = 16000;
constexpr size_t AUDIO_CHUNK_SIZE = 240;

// Initialize I2S microphone and speaker
void audioInit();

// Read one audio chunk from the microphone
size_t audioRead(uint8_t *buffer);

// Play one audio chunk through the speaker
void audioWrite(const uint8_t *buffer, size_t length);

#endif