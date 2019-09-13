// Definitions for GPIO
#ifndef GPIO_DEFS_H
#define GPIO_DEFS_H
#include <stdbool.h>

#define MASK(x) (1UL << (x))

// LEDs Port and Pins
#define LED_PORT (PORTE)
#define SPEED0 (30)	// PTE30
#define SPEED1 (29) // PTE29
#define SPEED2 (23) // PTE23
#define SPEED3 (22)	// PTE22
#define SPEED4 (21)	// PTE21
#define SPEED5 (20) // PTE20

// LED States
#define LED_OFF (0)
#define LED_ON	(1)

// ADC Pin 
#define ADC_POS (0) // PTB0

// Motor Direction Button
#define DIR_BUTTON_POS 		(6)	// PTD6
#define DIR_BUTTONUP 			(0)
#define DIR_BUTTONDOWN 		(1)
#define DIR_BUTTONBOUNCE 	(2)
#define DIR_BOUNCEDELAY 	(3)

// Start Button
#define START_BUTTON_POS 		(7)	//PTD7
#define START_BUTTONUP 			(0)
#define START_BUTTONDOWN 		(1)
#define START_BUTTONBOUNCE 	(2)
#define START_BOUNCEDELAY		(3)
#define START_EVT_ID 				(0)	// flag event

// Function prototypes
void configureGPIOinput(void);  			// Initialise button
void configureGPIOoutput(void); 			// Initialise output
void ledOnOff(int ledPin, int onOff);	// Turn led on/off
bool isDirPressed(void);							// check direction switch
bool isStartPressed(void); 						// check start switch
#endif
