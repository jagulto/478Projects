#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"

#define 	BLUE_MASK 		0x04
//*****************************************************************************
//
//!
//! A very simple example that uses a general purpose timer generated periodic 
//! interrupt to toggle the on-board LED.
//
//*****************************************************************************


void
PortFunctionInit(void)
{
//
		volatile uint32_t ui32Loop;   
	// Enable the GPIO port that is used for the on-board LED.
    //
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;

    //
    // Do a dummy read to insert a few cycles after enabling the peripheral.
    //
    ui32Loop = SYSCTL_RCGC2_R;

    //
    // Enable the GPIO pin for the blue LED (PF2).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    //
    GPIO_PORTF_DIR_R |= 0x04;
    GPIO_PORTF_DEN_R |= 0x04;

}

//Globally enable interrupts 
void IntGlobalEnable(void)
{
    __asm("    cpsie   i\n");
}

void Timer0A_Init(unsigned long period)
{   
	volatile uint32_t ui32Loop; 
	
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_TIMER0; // activate timer0
  ui32Loop = SYSCTL_RCGC1_R;				// Do a dummy read to insert a few cycles after enabling the peripheral.
  TIMER0_CTL_R &= ~0x00000001;     // disable timer0A during setup
  TIMER0_CFG_R = 0x00000000;       // configure for 32-bit timer mode
  TIMER0_TAMR_R = 0x00000002;      // configure for periodic mode, default down-count settings
  TIMER0_TAILR_R = period-1;       // reload value
	NVIC_PRI4_R &= ~0xE0000000; 	 // configure Timer0A interrupt priority as 0
  NVIC_EN0_R |= 0x00080000;     // enable interrupt 19 in NVIC (Timer0A)
	TIMER0_IMR_R |= 0x00000001;      // arm timeout interrupt
  TIMER0_CTL_R |= 0x00000001;      // enable timer0A
}

//interrupt handler for Timer0A
void Timer0A_Handler(void)
{
		// acknowledge flag for Timer0A
		TIMER0_ICR_R |= 0x00000001; 
	
		// Toggle the blue LED.
    GPIO_PORTF_DATA_R ^=BLUE_MASK;

}

int main(void)
{	
		unsigned long period = 8000000; //reload value to Timer0A to generate half second delay
	
		//initialize the GPIO ports	
		PortFunctionInit();
	
    // Turn on the LED.
    GPIO_PORTF_DATA_R |= 0x04;

    //initialize Timer0A and configure the interrupt
		Timer0A_Init(period);
	
		IntGlobalEnable();        		// globally enable interrupt
	
    //
    // Loop forever.
    //
    while(1)
    {

    }
}
