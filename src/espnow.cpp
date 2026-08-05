#include "espnow.h"
#include "audio.h"

#include <WiFi.h>
#include <esp_now.h>

// Broadcast Address
static const uint8_t BROADCAST_ADDRESS[6] =
    {
        0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF};

// Packet Types
enum PacketType
{
    AUDIO_PACKET = 0x01,
    CONTROL_PACKET = 0x02
};

// Audio Packet
struct AudioPacket
{
    uint8_t packetType;
    uint8_t length;
    uint8_t data[AUDIO_CHUNK_SIZE];
};

// Control Packet
struct ControlPacket
{
    uint8_t packetType;
    uint8_t controlType;
};

// Receive Buffer
static uint8_t receivedAudio[AUDIO_CHUNK_SIZE];
static size_t receivedLength = 0;
static volatile bool newAudio = false;

// Channel Status
static volatile bool busy = false;
static uint32_t unlockTime = 0;

// Called whenever an ESP-NOW packet has been transmitted
static void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    (void)mac_addr;
    (void)status;
}

// Called whenever an ESP-NOW packet is received
static void onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int len)
{
    (void)mac_addr;

    if (len == sizeof(AudioPacket))
    {
        const AudioPacket *packet = (const AudioPacket *)data;

        memcpy(receivedAudio, packet->data, packet->length);
        receivedLength = packet->length;
        newAudio = true;
    }
    else if (len == sizeof(ControlPacket))
    {
        const ControlPacket *packet = (const ControlPacket *)data;

        switch (packet->controlType)
        {
        case PTT_PRESSED:
            busy = true;
            break;

        case PTT_RELEASED:
            unlockTime = millis() + 1000;
            break;
        }
    }
}

void espNowInit()
{
    // Put the ESP32 into Station Mode
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        for (;;)
        {
            ;
        }
    }

    // Register Callbacks
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataReceived);

    // Add the broadcast peer
    esp_now_peer_info_t peerInfo = {};

    memcpy(peerInfo.peer_addr, BROADCAST_ADDRESS, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    esp_now_add_peer(&peerInfo);
}

bool espNowSendAudio(const uint8_t *buffer, size_t length)
{
    AudioPacket packet = {};

    packet.packetType = AUDIO_PACKET;
    packet.length = length;

    memcpy(packet.data, buffer, length);

    return (esp_now_send(BROADCAST_ADDRESS, (uint8_t *)&packet, sizeof(packet)) == ESP_OK);
}

bool espNowSendControl(ControlPacketType type)
{
    ControlPacket packet = {};

    packet.packetType = CONTROL_PACKET;
    packet.controlType = type;

    return (esp_now_send(BROADCAST_ADDRESS,
                         (uint8_t *)&packet,
                         sizeof(packet)) == ESP_OK);
}

bool espNowAudioAvailable()
{
    // Release the channel one second after the remote user releases PTT
    if (busy && unlockTime != 0 && millis() >= unlockTime)
    {
        busy = false;
        unlockTime = 0;
    }

    return newAudio;
}

size_t espNowReadAudio(uint8_t *buffer)
{
    memcpy(buffer, receivedAudio, receivedLength);
    newAudio = false;
    return receivedLength;
}

bool channelBusy()
{
    // Release the channel one second after the remote user releases PTT
    if (busy && unlockTime != 0 && millis() >= unlockTime)
    {
        busy = false;
        unlockTime = 0;
    }

    return busy;
}