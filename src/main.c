/*----------------------------------------------------------------------------
	ec18446 - PROJECT D:
	
	This project implements the following features:
	
	1) Vary motor speed with PWM
	2) Set 6 different speeds, including no speed at all
	3) Calculate and display RPM speed
	4) Change and display the direction of rotation
	
	Non-implemented requirements:
	1) Detect the motor is stopped
	
	There are 5 threads that implemented the requirements:
	1)t_control: it implements the motor control state-machine
	2)t_rpm: it calculates the rpm speed
	3)t_rotation: it determines the direction of rotation
	4)t_dirButton: it polls a button event used to change the direction of rotation
	5)t_startButton: it polls a button event used to start up the system
  Messages Queues:
	1)interval_q_id: it sends a sum of channel values of channel 0
	2)rotation_q_id: it sends a sum difference of two different channel values at
		two different channel interrupt (inside flag checks)
	Event Flags:
	1) start_evt_flags: it sends a signal to the t_control thread to start up
	
 *---------------------------------------------------------------------------*/

#include <cmsis_os2.h>
#include <MKL25Z4.H>
#include <stdint.h>
#include <stdbool.h>

#include "../include/gpio.h"
#include "../include/tpmPwm.h"
#include "../include/dcmotor.h"
#include "../include/adc_defs.h"

/* -------------------------------------
	Threads
   ------------------------------------- */
osThreadId_t t_control;			/* id of task to control motor*/
osThreadId_t t_rpm;					/* id of task to calculate RPM speed */
osThreadId_t t_rotation;		/* id of task to sense motor rotation*/
osThreadId_t t_dirButton;		/* id of task to change direction of motor*/
osThreadId_t t_startButton;	/* id of task to start system*/
/*--------------------------------------
	Event Flags
---------------------------------------*/
osEventFlagsId_t start_evt_flags;					// declare an id for event flags object
/*--------------------------------------
  Message queue
------------------------------------- */
osMessageQueueId_t interval_q_id;   			/* id of a message queue to cummicate with t_rpm*/
osMessageQueueId_t rotation_q_id;   			/* id of a message queues to communicate with t_rotation*/
/* -------------------------------------
    TPM1 interrupt handler

    Check each channel to see if caused interrupt
      - Channel 0 and 1 in use
    Also check overflow flag
      - Both may be set
    Reset by writing 1 to the flag register
   ------------------------------------- */

// Data updated by ISR
uint32_t count = 0 ; 						        // last count (or zero)
uint32_t cumul = 0 ; 						        // cumul count since message
uint32_t interval[8] = {0, 0, 0, 0, 0, 0, 0, 0} ;   // array of intervals
int nextIdx = 0 ;   // index to next spaces
int newCount ;      // store value of a new count

uint32_t interrupt_count = 0;	// count of channel interrupts
uint32_t count_a = 0;		    	// ch value at interrupt triggered by rotation sensor A
uint32_t count_b = 0; 		    // ch value at interrupt triggered by rotation sensor B

/*  Update Interval

    Called to update the interval and, periodically,
    send a message to rpm task
*/
void updateInterval() {
    // if no previous count has been recorded no interval
    // can be calculated
    if (!count) {
        count = TPM1->CONTROLS[0].CnV ;
        return ;
    }
    // calculate new interval
    newCount = TPM1->CONTROLS[0].CnV ;
	
    // handle case where TPM counter has wrapped
    if (newCount < count) {
        interval[nextIdx] = 0x10000 + newCount - count ;
    } else {
        interval[nextIdx] = newCount - count ;
    }
    count = newCount ;

    // update the interval since last message
    cumul = cumul + interval[nextIdx] ;
    
    // next index
    nextIdx = (nextIdx + 1) % 8 ;

    // send message if cumul time greater than threashold and
    // array is full
    if (cumul > MESSINTERVAL) {
        // check 8 non zero values
        int c = 0 ;        // count of non-zero intervals
        uint32_t sum = 0 ; // no need to divide by 8 here
        for (int i = 0; i < 8; i++) {
            if (interval[i] > 0) c++ ;
            sum += interval[i] ;
        }
        if (c == 8) {
            // send interval message
            osMessageQueuePut(interval_q_id, &sum, 0, NULL);  // Send Message	
            // may fail and should be checked
        }
    }
	
}
/*  Get Channel Value 
		
		Called to store channel value at each channel interrupt
		and keep track of interrupt count
*/
void getChannelValue(uint32_t* count, int channel){
    // only store channel value once in each cycle
    // so that interrupt count can be exactly 2 and
    // updateRotation is called in timer-overflow flag check
    if(!*count){
        *count = TPM1->CONTROLS[channel].CnV; // store channel value
        interrupt_count++;	                  // increment interrupt count
    }
}
/*  Update Rotation
		
		Called to determine direction of rotation
*/
void updateRotation() {
	
	// Compare count_a and count_b and send message to rotation task
  if(interrupt_count == 2)	
  {
    // Difference may be negative, use an integer type variable
  	int difference = count_a - count_b;
  	osMessageQueuePut(rotation_q_id, &difference, 0, NULL);
    count_a = 0;            // reset for next cycle
    count_b = 0;            // reset for next cycle
  	interrupt_count = 0;    // reset count for next cycle
  }
}

// Handler for TPM1 interrupts
//   - channel 0 and channel 1 interrupts used
void TPM1_IRQHandler(void) {
	
	// clear pending interrupts
	NVIC_ClearPendingIRQ(TPM1_IRQn);

	// Channel 0 flag
	if (TPM_CnSC_REG(TPM1,0) & TPM_CnSC_CHF_MASK) {
		// clear it
		TPM_CnSC_REG(TPM1,0) |= TPM_CnSC_CHF(1) ;
				
		updateInterval();               // Update interval
		getChannelValue(&count_a, 0);   // get channel value
	}

	// Channel 1 flag
	if (TPM_CnSC_REG(TPM1,1) & TPM_CnSC_CHF_MASK) {
        // clear it
        TPM_CnSC_REG(TPM1,1) |= TPM_CnSC_CHF(1) ;
        // code goes here - not used in demo
        getChannelValue(&count_b, 1); // get channel value 
	}

	// Overflow flag
	if (TPM1->SC & TPM_SC_TOF_MASK) {
        // clear it
        TPM1->SC |= TPM_SC_TOF(1) ;
        // code goes here: 
		updateRotation();   // update rotation
	}
}
/*----------------------------------------------------------------------------
  Polling the Direction Button

  The button is polled using a task with a delay.
  The polling rate is 25Hz
 *----------------------------------------------------------------------------*/
void directionButtonTask(void *arg) {
    int buttonState = DIR_BUTTONUP ; 			// current state of the button
    int bounceCounter = DIR_BOUNCEDELAY ; // counter for debounce
		int curDirection = FORWARD; 					// default at runtime
		
    while (1) {
        if (bounceCounter > 0) bounceCounter-- ;
        switch (buttonState) {
            case DIR_BUTTONUP:
                if (isDirPressed()) {
                    buttonState = DIR_BUTTONDOWN ;
                    // Set motor direction
                    switch(curDirection){
											case FORWARD:
                        curDirection = REVERSE;
                        setMotorDirection(STOPPED_FAST); // stop motor first
                        setMotorDirection(REVERSE);			 // set new direction
											break;
                      case REVERSE:
												curDirection = FORWARD;
                        setMotorDirection(STOPPED_FAST); // stop motor first
                        setMotorDirection(FORWARD);			 // set new direction
											break;
                    }
                }
                break ;
            case DIR_BUTTONDOWN:
                if (!isDirPressed()) {
                    buttonState = DIR_BUTTONBOUNCE ;
                    bounceCounter = DIR_BOUNCEDELAY ;
                }
                break ;
            case DIR_BUTTONBOUNCE:
                if (isDirPressed()) {
                    buttonState = DIR_BUTTONDOWN ;
                }
                else if (bounceCounter == 0) {
                    buttonState = DIR_BUTTONUP ;
                }
                break ;
        }
        osDelay(40) ; // delay
    }
}
/*----------------------------------------------------------------------------
  Polling the start Button
	
  The button is polled using a task with a delay
  The polling rate is 25Hz
 *----------------------------------------------------------------------------*/
void startButtonTask(void *arg) {
    int buttonState = START_BUTTONUP; 			// current state of the button
    int bounceCounter = START_BOUNCEDELAY ; // counter for debounce
		
    while (1) {
        if (bounceCounter > 0) bounceCounter--;
        switch (buttonState) {
            case START_BUTTONUP:
                if (isStartPressed()) {
									buttonState = START_BUTTONDOWN;
									// set flag to start up system
									osEventFlagsSet(start_evt_flags, MASK(START_EVT_ID));	
                }
                break ;
            case START_BUTTONDOWN:
                if (!isStartPressed()){
                    buttonState = START_BUTTONBOUNCE;
                    bounceCounter = START_BOUNCEDELAY;
                }
                break ;
            case START_BUTTONBOUNCE:
                if (isStartPressed()) {
                    buttonState = START_BUTTONDOWN;
                }
                else if (bounceCounter == 0) {
                    buttonState = START_BUTTONUP;
                }
                break ;
        }
        osDelay(40) ; // delay
    }
}
/* ------------------------------------------------------------------------------------
		Task t_rpm - calculate rpm speed 
----------------------------------------------------------------------------------- */
uint32_t rpm_speed;	// calculated rpm speed (see in debugger)
uint32_t count_sum; // value retrived from message queue (see in debugger)

// ---------------------------****IMPORTANT****--------------------------------------
// How the rpm speed calculation is determined by experiment, please read:
// With a set 50% duty cycle, count_sum varies between 36600-37800 as seen in debugger 
// Then manually counting the number of rotations of the wheel in one minute,
// 51 RPM were measured so we can deduce that: rpm_speed = "big constant" / count_sum	
// so "big constant" = rpm_speed * count_sum. Note, "big constant" will change
// proportionally with count_sum, so it is required to choose a value of count_sum within
// the range observed in the debugger to compute the value of "big constant".
// Choosing the upper-limit of the observed range of count_sum, we get: 	

#define BIG_CONSTANT (1887000u)	// 37000 * 51 = 1887000

void senseRPM (void *arg){
	
	while (1) {
		// Check if there is a message in the queue - handle errors
		osStatus_t status = osMessageQueueGet(interval_q_id, &count_sum, NULL, 1000u);
		if (status == osOK) {
			rpm_speed = BIG_CONSTANT / count_sum ;
		}
		else {
			// No message in the queue, no channel interrupts
			// therefore, motor is stoppped and rpm is zero
			rpm_speed = 0;	
		}
	}
}
/*---------------------------------------------------------------------------------
		Task t_rotation - determine motor rotation
-----------------------------------------------------------------------------------*/
bool isForward = false;	// current direction of rotation (see in debugger)
int input;						    // value retrived from message queue (see in debugger)

void senseRotation(void *arg) {
	
	while(1){
		osStatus_t status = osMessageQueueGet(rotation_q_id, &input, NULL, osWaitForever);
		if(status == osOK) 
		{
			if(input > 0)
				isForward = true; // If a - b = positive, forward rotation
			else
				isForward = false; // If a - b = negative, reverse rotation
		}
	}
}
/*----------------------------------------------------------------------------------------
	Task - Control Motor 
----------------------------------------------------------------------------------------*/
void controlMotor(void *arg){
  int state = START;		    	// current system state
  int currentLED;		          // pin id of current led on
  int currLevel;		          // current speed level
	
	while(1){
		MeasureVoltage();         // continuously update sres variable
		switch(state){
			case START:
				// Wait for button press event
				osEventFlagsWait(start_evt_flags, MASK(START_EVT_ID), osFlagsWaitAny, osWaitForever);
				MeasureVoltage();         											// update sres variable
				currLevel = getSpeedLevel(&sres, &currLevel);		// update current level
				if (currLevel == 0)
					// no need to set speed, motor is initially stopped at runtime
					// only need to change state
					state = STOPPED;															// update state
				else{ 					
					setMotorDirection(FORWARD);										// set forward direction
					setSpeed(&currLevel, &currentLED);						// set new speed
					state = MOVING;																// update state
				}
				break;
			case MOVING:
				currLevel = getSpeedLevel(&sres, &currLevel);		// update current level
				if (currLevel == 0){
					stopMotor(STOPPED_FAST);				    					// stop motor
					setSpeed(&currLevel, &currentLED);	        	// set new speed
					state = STOPPED;						    							// update state
				}
				else
					setSpeed(&currLevel, &currentLED);						// set new speed
				break;                                        	
			case STOPPED:                                   	
				currLevel = getSpeedLevel(&sres, &currLevel);		// update current level
				if (currLevel > 0){                           	
					setMotorDirection(FORWARD);										// set forward direction
					setSpeed(&currLevel, &currentLED);	        	// set new speed
					state = MOVING;
				}
				break;
		}
	}
}
/*----------------------------------------------------------------------------
 *        Main: Initialize and start RTX Kernel
 *---------------------------------------------------------------------------*/
volatile uint8_t calibrationFailed;	// zero expected

int main (void) {
	// Enable clock to ports B and E (ADC pins are on these ports)
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
		
	/* GPIO Setup*/
  configureGPIOinput() ;     	// Initialise button
  configureGPIOoutput() ;     // Initialise LEDs 
	configureMotorGPIOoutput();	// Initialise motor outputs
	
	/* ADC Setup*/
	Init_ADC(); 					    					// Initialise ADC
  calibrationFailed = ADC_Cal(ADC0);  // Calibrate the ADC 
  while (calibrationFailed); 					// Block progress if calibration failed
  Init_ADC(); 												// Reinitialise ADC
	
	/* TPM Module Setup: Input Capture, PWM*/
	configureTPMClock();				// Enable clock to TPM0
	configureTPM0forPWM();	    // Initialise TPM0 PWM	
	configureTPM1forCapture();	// Initialise TPM1 Input Capture
	
	SystemCoreClockUpdate();
	osKernelInitialize();
	
	/* Create event flags*/
	start_evt_flags = osEventFlagsNew(NULL);
	
	/* Create message queues*/
	interval_q_id = osMessageQueueNew(2, sizeof(uint32_t), NULL);
	rotation_q_id = osMessageQueueNew(2, sizeof(int), NULL);
	
	/* Create tasks*/
	t_control = osThreadNew(controlMotor, NULL, NULL);
	t_rpm = osThreadNew(senseRPM, NULL, NULL);
	t_rotation = osThreadNew(senseRotation, NULL, NULL);
	t_dirButton = osThreadNew(directionButtonTask, NULL, NULL);
	t_startButton = osThreadNew(startButtonTask, NULL, NULL);	
	
  osKernelStart();

  // end of initialisation
  for (;;) ;
}
