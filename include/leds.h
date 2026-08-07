#ifndef LEDS_H
#define LEDS_H

#include <Arduino.h>

// LED Pins
constexpr uint8_t RED_LED_PIN = 5;
constexpr uint8_t BLUE_LED_PIN = 4;

// Functions
void ledsInit();

void blueLedOn();
void blueLedOff();

void redLedOn();
void redLedOff();

#endif