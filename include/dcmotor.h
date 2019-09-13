#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include <MKL25Z4.H>

#define MASK(x) (1UL << (x))

// PORT & GPIO Pins
#define PWM_PORT (PORTA)
#define PWM_PIN_IP1 (1)	// PTA1
#define PWM_PIN_IP2 (2) // PTA2

// Motor States
#define START (0)
#define MOVING (1)
#define STOPPED (2)

// Motor Direction 
#define FORWARD (0)
#define REVERSE	(1)

// Stop Modes
#define STOPPED_FAST (0)
#define STOPPED_SLOW (1)
// These values were measured with a multimeter, potentiometer was about 10K
#define MIN_THRESHOLD (0x009F) 					// values below, set 0% duty cycle / 0 RPM (no load)
#define MAX_THRESHOLD (0xE68D) 					// values equal/greater, set 100% duty cycle / 200 RPM (no load)
// These values were calculated using the formula described in the report
#define SPEED1_MAX_THRESHOLD	(0x70CB) 	// values less than, set 50% duty cycle / 100 RPM (no load)
#define SPEED2_MAX_THRESHOLD	(0xB42F) 	// values less than, set 67% duty cycle / 134 RPM (no load)
#define SPEED3_MAX_THRESHOLD	(0xCD75) 	// values less than, set 78% duty cycle / 156 RPM (no load)
#define SPEED4_MAX_THRESHOLD	(0xE0F7)	// values less than, set 89% duty cycle / 178 RPM (no load)

// Function Prototypes
void configureMotorGPIOoutput(void);
void stopMotor(int fastSlow);
void setMotorDirection(int direction);
int getSpeedLevel(const volatile uint16_t* adc_value, int* current_level);
void setSpeed(const int* speed_level, int *current_led_on);
#endif
