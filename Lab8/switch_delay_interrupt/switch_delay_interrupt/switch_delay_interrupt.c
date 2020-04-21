#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"


#define 	BLUE_MASK 		0x04
//*****************************************************************************
//
//!
//! A very simple example that interfaces with the blud LED (PF2) and SW2 (PF0) 
//! using direct register access. When SW2 is pressed, the LED is turned on. When 
//! SW2 is released, the LED is turned off. Interrupt on PF0 is configured as 
//! edge-triggered on both edges
//
//*****************************************************************************


void
PortFunctionInit(void)
{

		volatile uint32_t ui32Loop;   
	
		// Enable the clock of the GPIO port that is used for the on-board LED and switch.
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;

    //
    // Do a dummy read to insert a few cycles after enabling the peripheral.
    //
    ui32Loop = SYSCTL_RCGC2_R;

		// Unlock GPIO Port F
		GPIO_PORTF_LOCK_R = 0x4C4F434B;   
		GPIO_PORTF_CR_R |= 0x01;           // allow changes to PF0

    // Set the direction of PF2 (blue LED) as output
    GPIO_PORTF_DIR_R |= 0x04;
	
		// Set the direction of PF0 (SW2) as input by clearing the bit
    GPIO_PORTF_DIR_R &= ~0x01;
	
    // Enable both PF2 and PF0 for digital function.
    GPIO_PORTF_DEN_R |= 0x05;
	
		//Enable pull-up on PF0
		GPIO_PORTF_PUR_R |= 0x01; 

}

//Globally enable interrupts 
void IntGlobalEnable(void)
{
    __asm("    cpsie   i\n");
}

void
Interrupt_Init(void)
{
  NVIC_EN0_R |= 0x40000000;  		// enable interrupt 30 in NVIC (GPIOF)
	NVIC_PRI7_R &= 0x00E00000; 		// configure GPIOF interrupt priority as 0
	GPIO_PORTF_IM_R |= 0x01;   		// arm interrupt on PF0
	GPIO_PORTF_IS_R &= ~0x01;     // PF0 is edge-sensitive
  GPIO_PORTF_IBE_R |= 0x01;   	// PF0 both edges trigger 
  //GPIO_PORTF_IEV_R &= ~0x01;  // PF0 falling edge event
	IntGlobalEnable();        		// globally enable interrupt
}

//interrupt handler
void GPIOPortF_Handler(void){
	
	// acknowledge flag for PF0
  GPIO_PORTF_ICR_R |= 0x01;      
	
	//SW2 is pressed
  if((GPIO_PORTF_DATA_R&0x01)==0x00) 
	{
			// Turn on the LED.
			GPIO_PORTF_DATA_R |= 0x04;
	}
	else
	{
			// Turn off the LED.
			GPIO_PORTF_DATA_R &= ~0x04;
	}
}

int main(void)
{
	
		//initialize the GPIO ports	
		PortFunctionInit();
		
	//configure the GPIOF interrupt
		Interrupt_Init();
	
    //
    // Loop forever.
    //
    while(1)
    {

      // Delay for a bit.
				SysCtlDelay(6000000);	
    }
}
