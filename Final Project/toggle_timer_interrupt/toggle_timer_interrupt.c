#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/uart.h"
#include "driverlib/uartstdio.h"

int seconds = 30;
int minutes = 59;
int hours = 12;

bool changeTime = false;
bool changeMinute = false;
bool changeHour = false;
bool ledState = true;

int newMinute = 0;
int newHour = 0;

int counter = 0;

void PortFunctionInit(void) {
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

void ConfigureUART(void) {
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

//Globally enable interrupts 
void IntGlobalEnable(void) {
    __asm("    cpsie   i\n");
}

void Timer0A_Init(unsigned long period) {   
	volatile uint32_t ui32Loop; 
	
	SYSCTL_RCGC1_R |= SYSCTL_RCGC1_TIMER0; // activate timer0
  ui32Loop = SYSCTL_RCGC1_R;				// Do a dummy read to insert a few cycles after enabling the peripheral.
  TIMER0_CTL_R &= ~0x00000001;     // disable timer0A during setup
  TIMER0_CFG_R = 0x00000000;       // configure for 32-bit timer mode
  TIMER0_TAMR_R = 0x00000002;      // configure for periodic mode, default down-seconds settings
  TIMER0_TAILR_R = period-1;       // reload value
	NVIC_PRI4_R &= ~0xE0000000; 	 // configure Timer0A interrupt priority as 0
  NVIC_EN0_R |= 0x00080000;     // enable interrupt 19 in NVIC (Timer0A)
	TIMER0_IMR_R |= 0x00000001;      // arm timeout interrupt
  TIMER0_CTL_R |= 0x00000001;      // enable timer0A
}

//interrupt handler for Timer0A
void Timer0A_Handler(void) {
		// acknowledge flag for Timer0A
		TIMER0_ICR_R |= 0x00000001; 
	
		seconds++;
		counter++;
	
		if (counter > 7) counter = 0;
			
		if (seconds > 59) {
			minutes++;
			seconds = 0;
		}
		
		if (minutes > 59) {
			minutes = 0;
			hours++;
		}
		
		if (hours > 12) hours = 1;
		
		if (!changeTime) UARTprintf("%02d:%02d:%02d\n", hours, minutes, seconds);
}



void
Interrupt_Init(void)
{
  NVIC_EN0_R |= 0x40000000;  		// enable interrupt 30 in NVIC (GPIOF)
	NVIC_PRI7_R &= 0x00D00000; 		// configure GPIOF interrupt priority as 0
	GPIO_PORTF_IM_R |= 0x11;   		// arm interrupt on PF0 and PF4
	GPIO_PORTF_IS_R &= ~0x11;     // PF0 and PF4 are edge-sensitive
  //GPIO_PORTF_IBE_R |= 0x11;   	// PF0 and PF4 both edges trigger 
  GPIO_PORTF_IEV_R &= ~0x11;  	// PF0 and PF4 falling edge event
}

void GPIOPortF_Handler(void) {
	NVIC_EN0_R &= ~0x40000000; 
	SysCtlDelay(53333);
	NVIC_EN0_R |= 0x40000000; 
	
	if (GPIO_PORTF_RIS_R&0x11) {
			GPIO_PORTF_ICR_R |= 0x11; 
			
			if((GPIO_PORTF_DATA_R&0x11)==0x00) {
					if (!changeTime) {
						changeTime = true;
						newMinute = minutes;
						
						UARTprintf("Enter minutes: (left - set | right - increment)\n");
						UARTprintf("%02d\n", newMinute);
						changeMinute = true;
					} else {
						changeTime = false;
						changeMinute = false;
						changeHour = false;
					}
			} else if((GPIO_PORTF_DATA_R&0x10)==0x00 && (GPIO_PORTF_DATA_R&0x01)!=0x00) {
					if (changeMinute) {
						minutes = newMinute;
						changeMinute = false;
						changeHour = true;
						
						newHour = hours;
						UARTprintf("Enter hours: (left - set | right - increment)\n");
						UARTprintf("%02d\n", newHour);
				} else if (changeHour) {
						hours = newHour;
						changeHour = false;
						changeTime = false;
				}
			} else if((GPIO_PORTF_DATA_R&0x01)==0x00 && (GPIO_PORTF_DATA_R&0x10)!=0x00) {
						if (changeMinute) {
							if (newMinute < 59) newMinute++;
							else newMinute = 0;
							UARTprintf("%02d\n", newMinute);
						} else if (changeHour) {
							if (newHour < 12) newHour++;
							else newHour = 1;
							UARTprintf("%02d\n", newHour);
						}
						
						else ledState = !ledState;
				}
		}
}

int main(void) {	
		unsigned long period = SysCtlClockGet(); //reload value to Timer0A to generate half second delay
	

	
		//initialize the GPIO ports	
		PortFunctionInit();
	
    //initialize Timer0A and configure the interrupt
		Timer0A_Init(period);
		Interrupt_Init();
	
		IntGlobalEnable();        		// globally enable interrupt
	
		ConfigureUART();

    while(1) {
			if (ledState) {
					if (counter == 1 || counter == 3 || counter == 7 || counter == 9) GPIO_PORTF_DATA_R |= 0x02;	
					else GPIO_PORTF_DATA_R &= ~0x02;																	

					if (counter == 2 || counter == 3 || counter == 6 || counter == 7) GPIO_PORTF_DATA_R |= 0x04;															
					else GPIO_PORTF_DATA_R &= ~0x04;	
					
					if (counter >= 4) GPIO_PORTF_DATA_R |= 0x08;
					else GPIO_PORTF_DATA_R &= ~0x08;
			} else {
					GPIO_PORTF_DATA_R &= ~0x02;		
					GPIO_PORTF_DATA_R &= ~0x04;	
					GPIO_PORTF_DATA_R &= ~0x08;
					counter = 0;
			}
    }
}
