#include "walkie.h"
#include "button.h"
#include "espnow.h"
#include "leds.h"
#include "audio.h"

static bool previousButtonState = false;

// Initializes the walkie-talkie system
void walkieInit()
{
    buttonInit();
    ledsInit();
    audioInit();
    espNowInit();
} // end walkieInit()

// Main loop for the walkie-talkie system.
// Call this repeatedly from loop().
void walkieUpdate()
{
    // Read the current button state
    bool currentButtonState = buttonPressed();

    // Update LEDs
    if (channelBusy())
    {
        redLedOn();
        blueLedOff();
    }
    else
    {
        redLedOff();
        blueLedOn();
    }

    // Detect button press
    if (currentButtonState && !previousButtonState)
    {
        espNowSendControl(PTT_PRESSED);
    }

    // Detect button release
    if (!currentButtonState && previousButtonState)
    {
        espNowSendControl(PTT_RELEASED);
    }

    // Stream microphone audio while the button is held
    if (currentButtonState && !channelBusy())
    {
        uint8_t buffer[AUDIO_CHUNK_SIZE];
        size_t bytesRead = audioRead(buffer);
        espNowSendAudio(buffer, bytesRead);
    }

    // Play received audio
    if (espNowAudioAvailable())
    {
        uint8_t buffer[AUDIO_CHUNK_SIZE];
        size_t bytesReceived = espNowReadAudio(buffer);
        audioWrite(buffer, bytesReceived);
    }

    // Save the current button state for the next loop iteration
    previousButtonState = currentButtonState;

} // end walkieUpdate()