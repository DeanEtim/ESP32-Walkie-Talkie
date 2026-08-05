#include "leds.h"

void ledsInit()
{
    pinMode(RED_LED_PIN, OUTPUT);
    pinMode(BLUE_LED_PIN, OUTPUT);

    redLedOff();
    blueLedOff();
} // end ledsInit()

void redLedOn()
{
    digitalWrite(RED_LED_PIN, HIGH);
} // end redLedOn()

void redLedOff()
{
    digitalWrite(RED_LED_PIN, LOW);
} // end redLedOff()

void blueLedOn()
{
    digitalWrite(BLUE_LED_PIN, HIGH);
} // end blueLedOn()

void blueLedOff()
{
    digitalWrite(BLUE_LED_PIN, LOW);
} // end blueLedOff()