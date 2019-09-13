#include "../include/dcmotor.h"
#include "../include/tpmPwm.h"
#include "../include/gpio.h"
#include <cmsis_os2.h>

/*----------------------------------------------------------------------------
	Configure GPIO Output Pins:
		PWM_PIN_IP1: the pin on PortA connected to IP1 on Dual H-Bridge
		PWM_PIN_IP2: the pin on PortA connected to IP2 on Dual H-Bridge
 *----------------------------------------------------------------------------*/
void configureMotorGPIOoutput(void){
	// Enable clock to port A
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
	
	// Set Pins on PWM_PORT as GPIO
    PWM_PORT->PCR[PWM_PIN_IP1] &= ~PORT_PCR_MUX_MASK;
    PWM_PORT->PCR[PWM_PIN_IP1] |= PORT_PCR_MUX(1);
    PWM_PORT->PCR[PWM_PIN_IP2] &= ~PORT_PCR_MUX_MASK;
    PWM_PORT->PCR[PWM_PIN_IP2] |= PORT_PCR_MUX(1);
	
	// Set Pins as Outputs
	PTA->PDDR |= MASK(PWM_PIN_IP1) | MASK(PWM_PIN_IP2);
	
	// Set start motor directoin: motor is stopped
	PTA->PSOR = MASK(PWM_PIN_IP1);	// Set to High
	PTA->PSOR = MASK(PWM_PIN_IP2);	// Set to High
}
/*----------------------------------------------------------------------------
  Stop Motor 

		Set the direction by:
		PWM_PIN_IP1 | PWM_PIN_IP2	| Output
		HIGH		|	HIGH		| Motor stops (fast)
		LOW			|	LOW			| Motor stops (free wheel)
 *----------------------------------------------------------------------------*/
void stopMotor(int fastSlow){
	switch (fastSlow){
		case STOPPED_FAST:
			PTA->PSOR = MASK(PWM_PIN_IP1);	// Set to High
			PTA->PSOR = MASK(PWM_PIN_IP2);	// Set to High
			break;
		case STOPPED_SLOW:
			PTA->PCOR = MASK(PWM_PIN_IP1);	// Set to Low
			PTA->PCOR = MASK(PWM_PIN_IP2);	// Set to Low
			break;
	}
}
/*----------------------------------------------------------------------------
  Set Motor Direction

		Set the direction by:
		PWM_PIN_IP1 | PWM_PIN_IP2	| Output
				HIGH		|			LOW			| Motor forward
				LOW			|			HIGH		| Motor reverse
 *----------------------------------------------------------------------------*/
void setMotorDirection(int direction){
	switch (direction)
	{
		case 0:
			PTA->PSOR = MASK(PWM_PIN_IP1);	// Set to High
			PTA->PCOR = MASK(PWM_PIN_IP2);	// Set to Low
			break;
		case 1:
			PTA->PCOR = MASK(PWM_PIN_IP1);	// Set to Low
			PTA->PSOR = MASK(PWM_PIN_IP2);	// Set to High
			break;
	}
}
/*----------------------------------------------------------------------------
  Get Speed Level
	
	According to the following comparisons:
	adc_value < MIN_THRESHOLD = 0
	adc_value >= MAX_THRESHOLD = 5 
	adc_value [MIN_THRESHOLD, MAX_THRESHOLD] non-inclusive = 1, 2, 3, 4	
	
	- update variable pointed to by current_level
	
 *----------------------------------------------------------------------------*/
int getSpeedLevel(const volatile uint16_t* adc_value, int* current_level){
	if (*adc_value < MIN_THRESHOLD){
		*current_level = 0;		// update current speed level
		return 0;
	}
	else if (*adc_value >= MAX_THRESHOLD){
		*current_level = 5;		// update current speed level
		return 5;
	}
	else if (*adc_value < SPEED1_MAX_THRESHOLD){
		*current_level = 5;		// update current speed level
			return 1;
	}
	else if (*adc_value < SPEED2_MAX_THRESHOLD){
		*current_level = 5;		// update current speed level
		return 2;
	}
	else if (*adc_value < SPEED3_MAX_THRESHOLD){
		*current_level = 5;		// update current speed level
		return 3;
	}
	else if (*adc_value < SPEED4_MAX_THRESHOLD){
		*current_level = 5;		// update current speed level
		return 4;
	}
	
	return 0xFFFF;	// return an error
}
/*----------------------------------------------------------------------------
  Set Speed
	
	According the value of speed_level:
	- set a duty cycle
	- turn off current led alight
	- turn on new led
	- update variable pointed to by current_led_on
-------------------------------------------------------------------------------*/
void setSpeed(const int* speed_level, int *current_led_on){
	
	switch(*speed_level){
		case 0:
			setPWMDuty(PWM_DUTY_MIN);			    // set duty cycle
			ledOnOff(*current_led_on, LED_OFF);	    // previous led off
			ledOnOff(SPEED0, LED_ON);				// new led on
			*current_led_on = SPEED0;				// update current led
			break;
		case 1:
			setPWMDuty(525u);										// set duty cycle - 50%
			ledOnOff(*current_led_on, LED_OFF);	    // previous led off
			ledOnOff(SPEED1, LED_ON); 			    // new led on
			*current_led_on = SPEED1;				// update current led
			break;
		case 2:
			setPWMDuty(702u);					    // set duty cycle - 67% 
			ledOnOff(*current_led_on, LED_OFF);	    // previous led off
			ledOnOff(SPEED2, LED_ON);				// new led on
			*current_led_on = SPEED2;				// update current led
			break;
		case 3:
			setPWMDuty(817u); 						// set duty cycle - 78&
			ledOnOff(*current_led_on, LED_OFF);	    // previous led off
			ledOnOff(SPEED3, LED_ON);			    // new led on
			*current_led_on = SPEED3;			    // update current led
			break;
		case 4:
			setPWMDuty(932u);						// set duty cycle 89%
			ledOnOff(*current_led_on, LED_OFF);	    // current led off
			ledOnOff(SPEED4, LED_ON);				// new led on
			*current_led_on = SPEED4;				// update current led 
			break;
		case 5:
			setPWMDuty(PWM_DUTY_MAX); 				// set duty cycle
			ledOnOff(*current_led_on, LED_OFF);	    // current led off
			ledOnOff(SPEED5, LED_ON);				// new led on
			*current_led_on = SPEED5;				// update current led
			break;
	}
}
