#ifndef ESPNOW_H
#define ESPNOW_H

#include <Arduino.h>

// Control Packet Types
enum ControlPacketType
{
    PTT_PRESSED,
    PTT_RELEASED
};

// Initialize ESP-NOW
void espNowInit();

// Send one chunk of audio
bool espNowSendAudio(const uint8_t *buffer, size_t length);

// Send a control packet
bool espNowSendControl(ControlPacketType type);

// Returns true if a new audio packet has been received
bool espNowAudioAvailable();

// Copy the received audio into the supplied buffer
size_t espNowReadAudio(uint8_t *buffer);

// Returns true if another device is currently transmitting
bool channelBusy();

#endif