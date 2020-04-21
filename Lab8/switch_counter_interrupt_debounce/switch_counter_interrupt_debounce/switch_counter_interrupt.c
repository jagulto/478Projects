#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"


//*****************************************************************************
//
//!
//! Design a counter. The counter is incremented by 1 when SW1 (PF4) or SW2 (PF0) 
//! is pressed.
//
//*****************************************************************************

// global variable visible in Watch window of debugger
// increments at least once per button press
volatile unsigned long count = 0;

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

	
		// Set the direction of PF4 (SW1) and PF0 (SW2) as input by clearing the bit
    GPIO_PORTF_DIR_R &= ~0x11;
	
    // Enable PF4, and PF0 for digital function.
    GPIO_PORTF_DEN_R |= 0x11;
	
		//Enable pull-up on PF4 and PF0
		GPIO_PORTF_PUR_R |= 0x11; 

}

//Globally enable interrupts 
void IntGlobalEnable(void)
{
    __asm("    cpsie   i\n");
}

//Globally disable interrupts 
void IntGlobalDisable(void)
{
    __asm("    cpsid   i\n");
}

void
Interrupt_Init(void)
{
  NVIC_EN0_R |= 0x40000000;  		// enable interrupt 30 in NVIC (GPIOF)
	NVIC_PRI7_R &= 0x00E00000; 		// configure GPIOF interrupt priority as 0
	GPIO_PORTF_IM_R |= 0x11;   		// arm interrupt on PF0 and PF4
	GPIO_PORTF_IS_R &= ~0x11;     // PF0 and PF4 are edge-sensitive
  GPIO_PORTF_IBE_R |= 0x11;   	// PF0 and PF4 both edges trigger 
  //GPIO_PORTF_IEV_R &= ~0x11;  	// PF0 and PF4 falling edge event
	IntGlobalEnable();        		// globally enable interrupt
}

//interrupt handler
void GPIOPortF_Handler(void)
{
	//switch debounce
	NVIC_EN0_R &= ~0x40000000; 
	SysCtlDelay(53333);	// Delay for a while
	NVIC_EN0_R |= 0x40000000; 
	
	//SW1 has action
	if(GPIO_PORTF_RIS_R&0x10)
	{
		// acknowledge flag for PF4
		GPIO_PORTF_ICR_R |= 0x10; 
		
		//SW1 is pressed
		if((GPIO_PORTF_DATA_R&0x10)==0x00) 
		{
			//counter imcremented by 1
			count++;
		}
	}
	
	//SW2 has action
  if(GPIO_PORTF_RIS_R&0x01)
	{
		// acknowledge flag for PF0
		GPIO_PORTF_ICR_R |= 0x01; 
		
		if((GPIO_PORTF_DATA_R&0x01)==0x00) 
		{
			//counter imcremented by 1
			count++;
		}
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

    }
}
