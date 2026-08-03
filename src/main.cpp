/**
 * ============================================================================
 *  ESP32 Digital Walkie-Talkie (Push-To-Talk over ESP-NOW)
 *
 *  Hardware:
 *    - ESP32-WROOM-32D
 *    - INMP441 I2S MEMS microphone  (L/R pin wired to GND -> LEFT channel)
 *    - MAX98357A I2S amplifier + speaker
 *    - Push-to-talk button on GPIO 2
 *    - Blue / Red / Yellow status LEDs
 * ============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <driver/i2s.h>

// --- INMP441 microphone ---
#define I2S_SCK_PIN 13 // Serial clock (BCLK)
#define I2S_WS_PIN 14  // Word select (LRCLK)
#define I2S_SD_PIN 12  // Serial data
#define I2S_PORT I2S_NUM_0

// --- MAX98357A amplifier (I2S port 1, TX) ---
#define SPK_BCLK_PIN 27
#define SPK_LRC_PIN 26
#define SPK_DIN_PIN 25
#define SPK_I2S_PORT I2S_NUM_1

// --- Push-to-talk button ---
#define PTT_BUTTON_PIN 2

// --- Status LEDs ---
#define LED_BLUE_PIN 4    // ON = idle/ready, slow blink = busy
#define LED_RED_PIN 16    // ON = channel locked (someone else recording)
#define LED_YELLOW_PIN 17 // ON = playing received audio

// AUDIO CONFIGURATION
#define SAMPLE_RATE 16000 // Hz, mono
#define BUFFER_COUNT 10   // DMA buffer count (starting point from spec)
#define BUFFER_SIZE 1024  // Chunk size (bytes) used when reading/writing audio

// Maximum recording length. 4s
#define MAX_RECORD_SECONDS 2
#define RECORD_BUFFER_BYTES (SAMPLE_RATE * 2 * MAX_RECORD_SECONDS)

// This shift controls the microphone gain: smaller number = louder. 14 is a good starting point.
#define MIC_GAIN_SHIFT 14

#define AUDIO_PAYLOAD_BYTES 240
#define MAX_AUDIO_PACKETS ((RECORD_BUFFER_BYTES + AUDIO_PAYLOAD_BYTES - 1) / AUDIO_PAYLOAD_BYTES)

// Audio data packet: 8 bytes of header + 240 bytes of PCM = 248 bytes total,
struct AudioPacket
{
  uint16_t messageID;    // Increments after every completed recording
  uint16_t packetNumber; // 0 .. totalPackets-1
  uint16_t totalPackets; // How many packets make up this message
  uint16_t payloadSize;  // Valid bytes in data[] (only the last packet is short)
  uint8_t data[AUDIO_PAYLOAD_BYTES];

  // ESPNOW 250-byte limit
};

// Small control packet used for channel locking. It is deliberately a
// different size from AudioPacket, so the receive callback can tell the two
// packet types apart just by looking at the received length.
enum ControlType : uint8_t
{
  CTRL_RECORDING_STARTED = 1,
  CTRL_RECORDING_FINISHED = 2
};

struct ControlPacket
{
  uint8_t type; // One of ControlType
};

// ESP-NOW broadcast address = every device on the channel receives it
static const uint8_t BROADCAST_ADDR[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// DEVICE STATE
enum DeviceState
{
  STATE_IDLE,
  STATE_RECORDING,
  STATE_PLAYING
};

static volatile DeviceState deviceState = STATE_IDLE;
static volatile bool channelLocked = false; // True while ANOTHER device records
static volatile bool transmitting = false;  // True while we stream packets out

// RECORDING BUFFER (sender side)
static uint8_t *recordBuffer = nullptr; // Allocated once in setup()
static uint32_t recordLength = 0;       // Bytes captured in the current recording
static uint16_t nextMessageID = 0;      // Increments after every finished recording

// State for the message currently being rebuilt from incoming packets.
struct AssemblyState
{
  bool active;
  uint16_t messageID;
  uint16_t totalPackets;
  uint16_t receivedCount;
  uint32_t totalBytes;
  uint8_t *buffer;                                  // malloc'd per message
  uint8_t receivedMap[(MAX_AUDIO_PACKETS + 7) / 8]; // Bitmap of packets seen
};

static AssemblyState assembly = {};

// PLAYBACK QUEUE (FIFO of complete messages)
#define MAX_QUEUED_MESSAGES 4

struct QueuedMessage
{
  uint8_t *data;
  uint32_t length;
};

static QueuedMessage playQueue[MAX_QUEUED_MESSAGES];
static volatile int queueHead = 0;  // Index of the next message to play
static volatile int queueCount = 0; // Number of messages waiting

// The message currently being played
static uint8_t *playData = nullptr;
static uint32_t playLength = 0;
static uint32_t playPos = 0;

// Protects the queue, because packets arrive on the Wi-Fi task while playback
// runs on the main loop.
static portMUX_TYPE queueMux = portMUX_INITIALIZER_UNLOCKED;

// MISC STATE
static volatile bool sendDone = true; // Set by the ESP-NOW send callback

// Button debouncing
static bool buttonPressed = false;
static bool lastButtonReading = false;
static uint32_t lastButtonChange = 0;
#define DEBOUNCE_MS 30

/*
 * Drives all three LEDs from the current state. Non-blocking; call often.
 *   Blue:   solid ON when idle & ready, slow blink (every 2 s) when busy
 *   Red:    ON while another device holds the channel
 *   Yellow: ON while playing received audio
 */
void updateLEDs()
{
  const bool busy = (deviceState != STATE_IDLE) || transmitting;

  if (busy)
  {
    // Slow blink: one short flash every 2 seconds
    digitalWrite(LED_BLUE_PIN, (millis() % 2000) < 200 ? HIGH : LOW);
  }
  else
  {
    digitalWrite(LED_BLUE_PIN, HIGH); // Idle and ready
  }

  digitalWrite(LED_RED_PIN, channelLocked ? HIGH : LOW);
  digitalWrite(LED_YELLOW_PIN, deviceState == STATE_PLAYING ? HIGH : LOW);
} // end updateLEDs()

/*
 * Configures both I2S ports:
 *   - Port 0 as RX for the INMP441 microphone (32-bit frames, LEFT channel,
 *     because the mic's L/R pin is tied to GND).
 *   - Port 1 as TX for the MAX98357A amplifier (16-bit mono).
 */
void setupI2S()
{
  // ---------- Microphone (RX) ----------
  i2s_config_t micConfig = {};
  micConfig.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  micConfig.sample_rate = SAMPLE_RATE;
  micConfig.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT; // INMP441 is a 24-bit device
  micConfig.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;  // L/R pin = GND: left slot
  micConfig.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  micConfig.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  // Small DMA buffers keep enough RAM free for the 5-second record buffer.
  micConfig.dma_buf_count = BUFFER_COUNT;
  micConfig.dma_buf_len = 256;
  micConfig.use_apll = false;

  i2s_pin_config_t micPins = {};
  micPins.bck_io_num = I2S_SCK_PIN;
  micPins.ws_io_num = I2S_WS_PIN;
  micPins.data_out_num = I2S_PIN_NO_CHANGE;
  micPins.data_in_num = I2S_SD_PIN;

  i2s_driver_install(I2S_PORT, &micConfig, 0, nullptr);
  i2s_set_pin(I2S_PORT, &micPins);
  i2s_zero_dma_buffer(I2S_PORT);

  // ---------- Speaker (TX) ----------
  i2s_config_t spkConfig = {};
  spkConfig.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  spkConfig.sample_rate = SAMPLE_RATE;
  spkConfig.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  spkConfig.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT; // Mono
  spkConfig.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  spkConfig.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  spkConfig.dma_buf_count = 6;
  spkConfig.dma_buf_len = 256;
  spkConfig.use_apll = false;

  i2s_pin_config_t spkPins = {};
  spkPins.bck_io_num = SPK_BCLK_PIN;
  spkPins.ws_io_num = SPK_LRC_PIN;
  spkPins.data_out_num = SPK_DIN_PIN;
  spkPins.data_in_num = I2S_PIN_NO_CHANGE;

  i2s_driver_install(SPK_I2S_PORT, &spkConfig, 0, nullptr);
  i2s_set_pin(SPK_I2S_PORT, &spkPins);
  i2s_zero_dma_buffer(SPK_I2S_PORT);
} // end setupI2S()

void receivePacket(const uint8_t *mac, const uint8_t *data, int len); // fwd decl

/** ESP-NOW send callback: lets transmitAudio() pace itself packet by packet. */
void onEspNowSent(const uint8_t *mac, esp_now_send_status_t status)
{
  (void)mac;
  (void)status;
  sendDone = true;
} // end onEspNowSent()

/**
 * Initialises ESP-NOW in station mode (no router) and registers the broadcast
 * peer so a single send reaches every device on the channel.
 */
void setupEspNow()
{
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // Make sure we are not associated with any AP

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("[ERROR] ESP-NOW init failed - rebooting");
    delay(2000);
    ESP.restart();
  }

  esp_now_register_send_cb(onEspNowSent);
  esp_now_register_recv_cb(receivePacket);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_ADDR, 6);
  peer.channel = 0;     // Current Wi-Fi channel
  peer.encrypt = false; // Broadcast frames cannot be encrypted
  esp_now_add_peer(&peer);
} // end setupEspNow()

/* Broadcasts a one-byte control packet (channel lock / unlock). */
void sendControlPacket(ControlType type)
{
  ControlPacket pkt = {(uint8_t)type};
  esp_now_send(BROADCAST_ADDR, (const uint8_t *)&pkt, sizeof(pkt));
} // end sendControlPacket()

/*
 * Called when the PTT button is pressed while the channel is free.
 * Locks the channel for everyone else and starts capturing microphone audio.
 */
void startRecording()
{
  sendControlPacket(CTRL_RECORDING_STARTED);

  recordLength = 0;
  i2s_zero_dma_buffer(I2S_PORT);
  deviceState = STATE_RECORDING;

  Serial.println("[REC] Recording started");
} // end startRecording()

/*
 * Reads one chunk of microphone audio, converts the 32-bit I2S frames down to
 * 16-bit PCM and appends them to the record buffer. This is called repeatedly from loop() while in the RECORDING state.
 * It Returns false when the buffer is full (which means the max duration has been reached).
 */
bool captureMicChunk()
{
  static int32_t rawSamples[BUFFER_SIZE / 4]; // 32-bit frames from the mic
  size_t bytesRead = 0;

  i2s_read(I2S_PORT, rawSamples, sizeof(rawSamples), &bytesRead, 20 / portTICK_PERIOD_MS);

  const int frameCount = bytesRead / 4;
  int16_t *out = (int16_t *)(recordBuffer + recordLength);
  uint32_t freeBytes = RECORD_BUFFER_BYTES - recordLength;
  int toStore = min((uint32_t)frameCount, freeBytes / 2);

  for (int i = 0; i < toStore; i++)
  {
    // 24-bit sample sits in the top of the 32-bit frame; shift down to
    // 16 bits. MIC_GAIN_SHIFT controls how loud the result is.
    out[i] = (int16_t)(rawSamples[i] >> MIC_GAIN_SHIFT);
  }
  recordLength += toStore * 2;

  return recordLength < RECORD_BUFFER_BYTES; // false = buffer full
} // end captureMicChunk()

/*
 * Splits the recording into 240-byte packets and broadcasts them one by one,
 * waiting for each send callback so we never overrun the radio.
 */
void transmitAudio()
{
  if (recordLength == 0)
    return;

  transmitting = true;

  const uint16_t totalPackets =
      (recordLength + AUDIO_PAYLOAD_BYTES - 1) / AUDIO_PAYLOAD_BYTES;

  Serial.printf("[TX] Sending message %u: %u bytes in %u packets\n",
                nextMessageID, recordLength, totalPackets);

  AudioPacket pkt;
  for (uint16_t p = 0; p < totalPackets; p++)
  {
    const uint32_t offset = (uint32_t)p * AUDIO_PAYLOAD_BYTES;
    const uint16_t payload =
        (uint16_t)min((uint32_t)AUDIO_PAYLOAD_BYTES, recordLength - offset);

    pkt.messageID = nextMessageID;
    pkt.packetNumber = p;
    pkt.totalPackets = totalPackets;
    pkt.payloadSize = payload;
    memcpy(pkt.data, recordBuffer + offset, payload);

    sendDone = false;
    esp_now_send(BROADCAST_ADDR, (const uint8_t *)&pkt, sizeof(pkt));

    // Wait (max 20 ms) for the send callback before queuing the next packet
    const uint32_t start = millis();
    while (!sendDone && millis() - start < 20)
      delayMicroseconds(200);

    delay(2);     // Small gap so receivers can keep up
    updateLEDs(); // Keep the blue "busy" blink alive during transmit
  }

  nextMessageID++; // Next recording gets a fresh ID
  transmitting = false;
  Serial.println("[TX] Done");
} // end transmitAudio()

/*
 * Called when the PTT button is released (or the 5-second limit is hit).
 * Frees the channel for everyone, then streams the recording out.
 */
void stopRecording()
{
  deviceState = STATE_IDLE;
  sendControlPacket(CTRL_RECORDING_FINISHED);

  Serial.printf("[REC] Recording stopped (%u bytes)\n", recordLength);

  transmitAudio();
} // end stopRecording()

/*Throws away a half-built message (e.g. when a newer one interrupts it). */
void resetAssembly()
{
  if (assembly.buffer != nullptr)
    free(assembly.buffer);
  memset(&assembly, 0, sizeof(assembly));
}

/*Adds a fully rebuilt message to the FIFO playback queue. */
void enqueueMessage(uint8_t *data, uint32_t length)
{
  portENTER_CRITICAL(&queueMux);
  if (queueCount < MAX_QUEUED_MESSAGES)
  {
    const int tail = (queueHead + queueCount) % MAX_QUEUED_MESSAGES;
    playQueue[tail].data = data;
    playQueue[tail].length = length;
    queueCount++;
    portEXIT_CRITICAL(&queueMux);
  }
  else
  {
    portEXIT_CRITICAL(&queueMux);
    Serial.println("[RX] Queue full - dropping message");
    free(data);
  }
} // end enqueueMessage()

/* Handles RECORDING_STARTED / RECORDING_FINISHED (the channel lock). */
void processControlPacket(const ControlPacket *pkt)
{
  // Ignore lock changes while we ourselves are recording (collision safety)
  if (deviceState == STATE_RECORDING)
    return;

  if (pkt->type == CTRL_RECORDING_STARTED)
  {
    channelLocked = true; // Red LED on, our PTT is disabled
  }
  else if (pkt->type == CTRL_RECORDING_FINISHED)
  {
    channelLocked = false; // Channel free again
  }
} // end processControlPacket()

/* Adds one audio packet into the message being reassembled. */
void processAudioPacket(const AudioPacket *pkt)
{
  if (pkt->totalPackets == 0 || pkt->totalPackets > MAX_AUDIO_PACKETS ||
      pkt->packetNumber >= pkt->totalPackets ||
      pkt->payloadSize > AUDIO_PAYLOAD_BYTES)
    return; // Malformed - ignore

  // A packet from a new message interrupts any unfinished assembly
  if (assembly.active && assembly.messageID != pkt->messageID)
  {
    Serial.println("[RX] Incomplete message discarded");
    resetAssembly();
  }

  // First packet of a new message: allocate a buffer for the whole thing
  if (!assembly.active)
  {
    const uint32_t maxBytes = (uint32_t)pkt->totalPackets * AUDIO_PAYLOAD_BYTES;
    uint8_t *buf = (uint8_t *)malloc(maxBytes);
    if (buf == nullptr)
    {
      Serial.println("[RX] Out of memory - message dropped "
                     "(consider lowering MAX_RECORD_SECONDS)");
      return;
    }
    assembly.active = true;
    assembly.messageID = pkt->messageID;
    assembly.totalPackets = pkt->totalPackets;
    assembly.receivedCount = 0;
    assembly.totalBytes = 0;
    assembly.buffer = buf;
    memset(assembly.receivedMap, 0, sizeof(assembly.receivedMap));
  }

  // Ignore duplicates
  const uint16_t p = pkt->packetNumber;
  if (assembly.receivedMap[p / 8] & (1 << (p % 8)))
    return;
  assembly.receivedMap[p / 8] |= (1 << (p % 8));

  memcpy(assembly.buffer + (uint32_t)p * AUDIO_PAYLOAD_BYTES,
         pkt->data, pkt->payloadSize);
  assembly.receivedCount++;
  assembly.totalBytes += pkt->payloadSize;

  // Message complete -> hand it to the playback queue
  if (assembly.receivedCount == assembly.totalPackets)
  {
    Serial.printf("[RX] Message %u complete (%u bytes)\n",
                  assembly.messageID, assembly.totalBytes);
    enqueueMessage(assembly.buffer, assembly.totalBytes);
    assembly.buffer = nullptr; // Ownership moved to the queue
    memset(&assembly, 0, sizeof(assembly));
  }
} // end processAudioPacket()

/*
 * ESP-NOW receive callback. Control packets and audio packets are told apart
 * by their length (1 byte vs 248 bytes).
 */
void receivePacket(const uint8_t *mac, const uint8_t *data, int len)
{
  (void)mac;

  if (len == sizeof(ControlPacket))
  {
    processControlPacket((const ControlPacket *)data);
  }
  else if (len == sizeof(AudioPacket))
  {
    processAudioPacket((const AudioPacket *)data);
  }
  // Anything else is not ours - ignore it
} // end receivePacket()

/*
 * Pops the next message off the queue and starts playing it.
 * Does nothing if the queue is empty.
 */
void playNextMessage()
{
  portENTER_CRITICAL(&queueMux);
  if (queueCount == 0)
  {
    portEXIT_CRITICAL(&queueMux);
    return;
  }
  playData = playQueue[queueHead].data;
  playLength = playQueue[queueHead].length;
  queueHead = (queueHead + 1) % MAX_QUEUED_MESSAGES;
  queueCount--;
  portEXIT_CRITICAL(&queueMux);

  playPos = 0;
  deviceState = STATE_PLAYING;
  Serial.printf("[PLAY] Playing message (%u bytes)\n", playLength);
}

/*
 * Feeds the speaker one chunk per loop() pass so the button and LEDs stay
 * responsive. When the message ends it automatically starts the next queued
 * one, or returns to IDLE.
 */
void servicePlayback()
{
  if (deviceState != STATE_PLAYING)
    return;

  const uint32_t remaining = playLength - playPos;
  const uint32_t chunk = min((uint32_t)BUFFER_SIZE, remaining);
  size_t written = 0;

  i2s_write(SPK_I2S_PORT, playData + playPos, chunk, &written,
            100 / portTICK_PERIOD_MS);
  playPos += written;

  if (playPos >= playLength)
  {
    free(playData);
    playData = nullptr;
    i2s_zero_dma_buffer(SPK_I2S_PORT);
    Serial.println("[PLAY] Finished");

    deviceState = STATE_IDLE;
    playNextMessage(); // Chain straight into the next queued message
  }
} // end servicePlayback()

/*
 * Debounces the PTT button and triggers start/stop of recording.
 * The button is ignored while the channel is locked by another device.
 */
void serviceButton()
{
  const bool reading = (digitalRead(PTT_BUTTON_PIN) == LOW);

  if (reading != lastButtonReading)
  {
    lastButtonReading = reading;
    lastButtonChange = millis();
    return;
  }

  if (millis() - lastButtonChange < DEBOUNCE_MS || reading == buttonPressed)
    return;

  buttonPressed = reading;

  if (buttonPressed)
  {
    // Only start if the channel is free and we are not doing anything else
    if (!channelLocked && deviceState == STATE_IDLE && !transmitting)
      startRecording();
  }
  else
  {
    if (deviceState == STATE_RECORDING)
      stopRecording();
  }
} // end serviceButton()

void setup()
{
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== ESP32 Walkie-Talkie booting ===");

  pinMode(PTT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_BLUE_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);

  // Grab the big record buffer FIRST, while the heap is still unfragmented.
  recordBuffer = (uint8_t *)malloc(RECORD_BUFFER_BYTES);
  if (recordBuffer == nullptr)
  {
    Serial.println("[FATAL] Could not allocate the record buffer. "
                   "Lower MAX_RECORD_SECONDS and re-flash.");
    while (true) // Fast red blink = fatal error
    {
      digitalWrite(LED_RED_PIN, !digitalRead(LED_RED_PIN));
      delay(150);
    }
  }

  setupI2S();
  setupEspNow();

  Serial.printf("Ready. Free heap: %u bytes\n", ESP.getFreeHeap());
  Serial.println("Hold the button to talk.");
} // end setup()

void loop()
{
  serviceButton();

  if (deviceState == STATE_RECORDING)
  {
    // Keep pulling audio from the mic; auto-stop at the 5-second limit
    if (!captureMicChunk())
    {
      Serial.println("[REC] Max duration reached");
      stopRecording();
    }
  }
  else
  {
    // Start playback whenever something is waiting and we are free
    if (deviceState == STATE_IDLE)
      playNextMessage();

    servicePlayback();
  }

  updateLEDs();
} // end loop()