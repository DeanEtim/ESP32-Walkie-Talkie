#include "audio.h"

#include <driver/i2s.h>

// I2S Ports
constexpr i2s_port_t MIC_I2S = I2S_NUM_0;
constexpr i2s_port_t SPK_I2S = I2S_NUM_1;

void audioInit()
{
    // ---------------- Microphone ----------------
    i2s_config_t micConfig = {};
    micConfig.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
    micConfig.sample_rate = SAMPLE_RATE;
    micConfig.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
    micConfig.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    micConfig.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    micConfig.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    micConfig.dma_buf_count = 8;
    micConfig.dma_buf_len = 256;
    micConfig.use_apll = false;

    i2s_pin_config_t micPins = {};
    micPins.bck_io_num = MIC_BCLK_PIN;
    micPins.ws_io_num = MIC_WS_PIN;
    micPins.data_out_num = I2S_PIN_NO_CHANGE;
    micPins.data_in_num = MIC_SD_PIN;

    i2s_driver_install(MIC_I2S, &micConfig, 0, nullptr);
    i2s_set_pin(MIC_I2S, &micPins);

    // ---------------- Speaker ----------------
    i2s_config_t spkConfig = {};
    spkConfig.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    spkConfig.sample_rate = SAMPLE_RATE;
    spkConfig.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    spkConfig.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    spkConfig.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    spkConfig.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    spkConfig.dma_buf_count = 8;
    spkConfig.dma_buf_len = 256;
    spkConfig.use_apll = false;

    i2s_pin_config_t spkPins = {};
    spkPins.bck_io_num = SPK_BCLK_PIN;
    spkPins.ws_io_num = SPK_WS_PIN;
    spkPins.data_out_num = SPK_SD_PIN;
    spkPins.data_in_num = I2S_PIN_NO_CHANGE;

    i2s_driver_install(SPK_I2S, &spkConfig, 0, nullptr);
    i2s_set_pin(SPK_I2S, &spkPins);
}

size_t audioRead(uint8_t *buffer)
{
    static int32_t samples[AUDIO_CHUNK_SIZE / 2];

    size_t bytesRead = 0;

    i2s_read(
        MIC_I2S,
        samples,
        sizeof(samples),
        &bytesRead,
        portMAX_DELAY);

    int16_t *out = (int16_t *)buffer;

    size_t sampleCount = bytesRead / sizeof(int32_t);

    for (size_t i = 0; i < sampleCount; i++)
    {
        out[i] = samples[i] >> 14;
    }

    return sampleCount * sizeof(int16_t);
}

void audioWrite(const uint8_t *buffer, size_t length)
{
    size_t written;

    i2s_write(
        SPK_I2S,
        buffer,
        length,
        &written,
        portMAX_DELAY);
}