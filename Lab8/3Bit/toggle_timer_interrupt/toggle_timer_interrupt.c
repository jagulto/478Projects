#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "driverlib/sysctl.h"

//*****************************************************************************
//
//!
//! A very simple example that uses a general purpose timer generated periodic 
//! interrupt to toggle the on-board LED.
//
//*****************************************************************************
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
	
		GPIO_PORTF_DIR_R |= 0x04 | 0x02 | 0x08;
	
    // Enable PF4, and PF0 for digital function.
    GPIO_PORTF_DEN_R |= 0x1F;
	
		//Enable pull-up on PF4 and PF0
		GPIO_PORTF_PUR_R |= 0x11; 
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
		if (count < 7) count++;
    //GPIO_PORTF_DATA_R ^=BLUE_MASK;
}

void
Interrupt_Init(void)
{
  NVIC_EN0_R |= 0x40000000;  		// enable interrupt 30 in NVIC (GPIOF)
	NVIC_PRI7_R &= 0x00500000; 		// configure GPIOF interrupt priority as 0
	GPIO_PORTF_IM_R |= 0x11;   		// arm interrupt on PF0 and PF4
	GPIO_PORTF_IS_R &= ~0x11;     // PF0 and PF4 are edge-sensitive
  GPIO_PORTF_IBE_R |= 0x11;   	// PF0 and PF4 both edges trigger 
  //GPIO_PORTF_IEV_R &= ~0x11;  	// PF0 and PF4 falling edge event
}

//interrupt handler
void GPIOPortF_Handler(void)
{
	//switch debounce
	NVIC_EN0_R &= ~0x40000000; 
	//for (int i = 0; i < 10000; i++) 
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
			//counter decremented by 1
			if (count > 0) count--;
			//else count = 7;
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
			if (count < 7) count++;
			//else count = 0;
			if (count > 7) count = 7;
		}
	}
}

int main(void)
{	
		unsigned long period = SysCtlClockGet(); //reload value to Timer0A to generate half second delay
	
		//initialize the GPIO ports	
		PortFunctionInit();
	
    //initialize Timer0A and configure the interrupt
		Timer0A_Init(period);
		Interrupt_Init();
	
		IntGlobalEnable();        		// globally enable interrupt
	
    //
    // Loop forever.
    //
    while(1)
    {
			if (count == 1 || count == 3 || count == 5 || count == 7) GPIO_PORTF_DATA_R |= 0x02;	
			else GPIO_PORTF_DATA_R &= ~0x02;																	

			if (count == 2 || count == 3 || count == 6 || count == 7) GPIO_PORTF_DATA_R |= 0x04;															
			else GPIO_PORTF_DATA_R &= ~0x04;	
			
			if (count >= 4) GPIO_PORTF_DATA_R |= 0x08;
			else GPIO_PORTF_DATA_R &= ~0x08;
    }
}
