#include "LED.h"

void InitializeLEDPortPins(void)
{
	// Initialize the value and port pin direction of the LEDs.
    TURN_OFF_GREEN_LED;
    SET_GREEN_LED_AS_AN_OUTPUT;
    TURN_OFF_RED_LED;
    SET_RED_LED_AS_AN_OUTPUT;
}

