#include <MKL25Z4.H>
#include <stdbool.h>
#include "../include/gpio.h"

/*----------------------------------------------------------------------------
  GPIO Input Configuration
 *----------------------------------------------------------------------------*/
void configureGPIOinput(void) {
   SIM->SCGC5 |=  SIM_SCGC5_PORTD_MASK; /* enable clock for port D */
	
	/* 
	Configuration of direction button 
   - Select GPIO and enable pull-up resistors and no interrupts
	- Set port D switch bit to input 
   */
   PORTD->PCR[DIR_BUTTON_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK |
                             PORT_PCR_PE_MASK | PORT_PCR_IRQC(0x0);
   PTD->PDDR &= ~MASK(DIR_BUTTON_POS);
	/* 
	Configuration of start button
	- Select GPIO and enable pull-up resistors and no interrupts
	- Set port D switch bit to input
	*/
	PORTD->PCR[START_BUTTON_POS] |= PORT_PCR_MUX(1) | PORT_PCR_PS_MASK |
																	PORT_PCR_PE_MASK | PORT_PCR_IRQC(0x0);
	PTD->PDDR &= ~MASK(START_BUTTON_POS);
}

/* ----------------------------------------
    Configure 5 GPIO outputs on port 
       1. Enable clock to GPIO port
       2. Enable GPIO port
       3. Set GPIO direction to output
       4. Ensure output low
		Pin numbers given by:
			- SPEED0 	 (30)
			- SPEED1 (29)
			- SPEED2  (23)
			- SPEED3   (22)
			- SPEED4 (21)
			- SPEED5		(20) 
 * ---------------------------------------- */
void configureGPIOoutput(void) {
    // Enable clock to port E
    SIM->SCGC5 |=  SIM_SCGC5_PORTE_MASK; 

    // Make pins GPIO
	LED_PORT->PCR[SPEED0] &= ~PORT_PCR_MUX_MASK;
	LED_PORT->PCR[SPEED0] |= PORT_PCR_MUX(1);
	LED_PORT->PCR[SPEED1] &= ~PORT_PCR_MUX_MASK;
	LED_PORT->PCR[SPEED1] |= PORT_PCR_MUX(1);
	LED_PORT->PCR[SPEED2] &= ~PORT_PCR_MUX_MASK;
	LED_PORT->PCR[SPEED2] |= PORT_PCR_MUX(1);
	LED_PORT->PCR[SPEED3] &= ~PORT_PCR_MUX_MASK;
	LED_PORT->PCR[SPEED3] |= PORT_PCR_MUX(1);
	LED_PORT->PCR[SPEED4] &= ~PORT_PCR_MUX_MASK;
	LED_PORT->PCR[SPEED4] |= PORT_PCR_MUX(1);
	LED_PORT->PCR[SPEED5] &= ~PORT_PCR_MUX_MASK;
	LED_PORT->PCR[SPEED5] |= PORT_PCR_MUX(1);
	
    // Set pins to output
    PTE->PDDR |= MASK(SPEED0);
	PTE->PDDR |= MASK(SPEED1);
	PTE->PDDR |= MASK(SPEED2);
	PTE->PDDR |= MASK(SPEED3);
	PTE->PDDR |= MASK(SPEED4);
	PTE->PDDR |= MASK(SPEED5);	
		
    // Turn off output
	PTE->PCOR = MASK(SPEED0);
    PTE->PCOR = MASK(SPEED1);
    PTE->PCOR = MASK(SPEED2);
    PTE->PCOR = MASK(SPEED3);
	PTE->PCOR = MASK(SPEED4);
    PTE->PCOR = MASK(SPEED5);
}
/*-----------------------------------------------
		Turn LED on and off
-----------------------------------------------*/
void ledOnOff (int ledPin, int onOff){
	if (onOff == LED_ON)
		PTE->PSOR = MASK(ledPin);
	else
		PTE->PCOR = MASK(ledPin);
}
/*----------------------------------------------------------------------------
  isDirPressed: test the switch

  Operating the switch connects the input to ground. A non-zero value
  shows the switch is not pressed.
 *----------------------------------------------------------------------------*/
bool isDirPressed(void) {
    if (PTD->PDIR & MASK(DIR_BUTTON_POS)) {
            return false ;
    }
    return true ;
}
/*----------------------------------------------------------------------------
  isStartPressed: test the switch

  Operating the switch connects the input to ground. A non-zero value
  shows the switch is not pressed.
 *----------------------------------------------------------------------------*/
bool isStartPressed(void) {
    if (PTD->PDIR & MASK(START_BUTTON_POS)) {
            return false ;
    }
    return true ;
}
