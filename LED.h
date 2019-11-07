#ifndef LED_H
#define LED_H

#include <msp430.h>

//  Green LED
#define GREEN_LED 1
#define GREEN_LED_BIT				BIT0
#define GREEN_LED_PORT				P1OUT
#define GREEN_LED_DDR				P1DIR
#define SET_GREEN_LED_AS_AN_OUTPUT	GREEN_LED_DDR |= GREEN_LED_BIT
#define TURN_ON_GREEN_LED		    GREEN_LED_PORT |= GREEN_LED_BIT
#define TURN_OFF_GREEN_LED		    GREEN_LED_PORT &= ~GREEN_LED_BIT
#define TOGGLE_GREEN_LED		    GREEN_LED_PORT ^= GREEN_LED_BIT

//  Red LED
#define RED_LED 2
#define RED_LED_BIT				    BIT6
#define RED_LED_PORT				P1OUT
#define RED_LED_DDR				    P1DIR
#define SET_RED_LED_AS_AN_OUTPUT	RED_LED_DDR |= RED_LED_BIT
#define TURN_ON_RED_LED			    RED_LED_PORT |= RED_LED_BIT
#define TURN_OFF_RED_LED			RED_LED_PORT &= ~RED_LED_BIT
#define TOGGLE_RED_LED				RED_LED_PORT ^= RED_LED_BIT

// Prototypes
void InitializeLEDPortPins(void);

#endif
