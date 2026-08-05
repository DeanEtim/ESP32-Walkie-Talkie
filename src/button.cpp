#include "button.h"

void buttonInit()
{
    pinMode(PTT_BUTTON_PIN, INPUT_PULLUP);
} // end buttonInit()

bool buttonPressed()
{
    return digitalRead(PTT_BUTTON_PIN) == LOW;
} // end buttonPressed()