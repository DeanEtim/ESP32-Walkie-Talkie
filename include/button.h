#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

// Push-To-Talk Button
constexpr uint8_t PTT_BUTTON_PIN = 2;

// Initialize the button
void buttonInit();

// Returns true while the button is being held down
bool buttonPressed();

#endif