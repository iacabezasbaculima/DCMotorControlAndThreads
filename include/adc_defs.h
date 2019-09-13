#ifndef ADC_DEFS_H
#define ADC_DEFS_H

#include <MKL25Z4.H>
#include <stdint.h>

// Freedom KL25Z ADC Channel
#define ADC_CHANNEL (8)				// on port B: pin B0
#define ADC_DIFF_CHANNEL (0)	// on port E: pin E20 (plus) and E21 (minus)

#define VREF (3.3) // reference voltage
#define ADCRANGE (0xffff) // maximum for a 16 bit conversion

// The following values measured
// Potentiometer used is almost 10K Ohms
#define MIN_ADC (0x9F)		//159	- 8mV
#define MAX_ADC (0xEA0E)	//59180 - 2980mV

// External variables
extern volatile uint16_t sres ;	// raw value - single 
extern volatile int16_t dres ;	// raw value - differential
 
// Function prototypes 
void MeasureVoltage(void) ;
void MeasureVoltageDiff(void) ;
void Init_ADC(void) ;
uint8_t ADC_Cal(ADC_MemMapPtr) ;

#endif
