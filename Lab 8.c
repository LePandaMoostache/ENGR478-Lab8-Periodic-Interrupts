#include <stdint.h>
#include <stdbool.h>
#include "Lab 8.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom_map.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "inc/tm4c123gh6pm.h"

#define 	RED_MASK 			0x02
#define 	BLUE_MASK 		0x04
#define 	GREEN_MASK		0x08
#define 	OFF						0x00
//*****************************************************************************

// global variable visible in Watch window of debugger
// increments at least once per button press
volatile unsigned long count = 0;
volatile unsigned long clock;
volatile unsigned long timer0AGet;
volatile unsigned long gPIOFGet;

void
PortFunctionInit(void) {

    volatile uint32_t ui32Loop;

    // Enable the clock of the GPIO port that is used for the on-board LED and switch.
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;

    //
    // Do a dummy read to insert a few cycles after enabling the peripheral.
    //
    ui32Loop = SYSCTL_RCGC2_R;

    // Unlock GPIO Port F
    GPIO_PORTF_LOCK_R = 0x4C4F434B;
    GPIO_PORTF_CR_R |= 0x01; // allow changes to PF0

    //
    // Enable pin PF2 for GPIOOutput
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
	
	  //
    // Enable pin PF3 for GPIOOutput
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

    //
    // Enable pin PF0 for GPIOInput
    //

    //
    //First open the lock and select the bits we want to modify in the GPIO commit register.
    //
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) = 0x1;

    //
    //Now modify the configuration of the pins that we unlocked.
    //
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0);

    //
    // Enable pin PF4 for GPIOInput
    //
    GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);

    //
    // Enable pin PF1 for GPIOOutput
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);

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

void Timer0A_Init(unsigned long period)
{   
	//
  // Enable Peripheral Clocks 
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC); 		// configure for 32-bit timer mode
  TimerLoadSet(TIMER0_BASE, TIMER_A, period -1);      //reload value
  IntEnable(INT_TIMER0A);    				// enable interrupt 19 in NVIC (Timer0A)
	IntPrioritySet(INT_TIMER0A, 0x00);  	 // configure Timer0A interrupt priority as 0
	timer0AGet = IntPriorityGet(INT_TIMER0A);
	IntPriorityMaskSet(0x4);				// 0100.0000 = 40 in hex
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);      // arm timeout interrupt
  TimerEnable(TIMER0_BASE, TIMER_A);      // enable timer0A
}

//interrupt handler for Timer0A
void Timer0A_Handler (void)
{
    // acknowledge flag for Timer0A timeout
		TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
		// Increments counter by 1
    count ++;
		count = count&7;
}  


/** GPIOF INTERRUPT CONFIGURATION***/
void
Interrupt_Init(void)
{

  NVIC_EN0_R |= 0x40000000;  		// enable interrupt 30 in NVIC (GPIOF)
	IntPrioritySet(INT_GPIOF, 0x2);         // configure GPIOF interrupt priority as 2
	gPIOFGet = IntPriorityGet(INT_GPIOF);
	GPIO_PORTF_IM_R |= 0x11;   		// arm interrupt on PF0 and PF4
	GPIO_PORTF_IS_R &= ~0x11;     // PF0 and PF4 are edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;   	// PF0 and PF4 not both edges trigger 
	GPIO_PORTF_IEV_R &= ~0x11;  	// PF0 and PF4 falling edge event
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
		if(((GPIO_PORTF_DATA_R&0x10)==0x00)) 
		{
			//counter imcremented by 1
			count++;
			count = count & 7;
		}
	}
	
	//SW2 has action
  if(GPIO_PORTF_RIS_R&0x01)
	{
		// acknowledge flag for PF0
		GPIO_PORTF_ICR_R |= 0x01; 
		
		if(((GPIO_PORTF_DATA_R&0x01)==0x00)) 
		{
			//counter imcremented by 1
			count--;
			count = count & 7;
		}
	}

}
	
	int main(void)
{
    //initialize the GPIO ports    
    PortFunctionInit();
        
    //configure the GPIOF interrupt
    Interrupt_Init();

   unsigned long period = 16000000;
   SysCtlClockSet(SYSCTL_SYSDIV_10|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
   clock = SysCtlClockGet();

	
    //initialize Timer0A and configure the interrupt
    Timer0A_Init(period);
	
    //
    // Loop forever.
    //
    while(1)
    {
        switch (count) {

            case 0:
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, OFF);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, OFF);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, OFF);
                break;
            case 1:
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, RED_MASK);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, OFF);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, OFF);
                break;
            case 2:
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, OFF);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, BLUE_MASK);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, OFF);
                break;
            case 3:
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, RED_MASK);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, BLUE_MASK);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, OFF);
                break;
            case 4:
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, OFF);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, OFF);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GREEN_MASK);
                break;
            case 5:
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, RED_MASK);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, OFF);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GREEN_MASK);
                break;
            case 6:
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, OFF);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, BLUE_MASK);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GREEN_MASK);
                break;
            case 7:
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, RED_MASK);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, BLUE_MASK);
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GREEN_MASK);
                break;

        }
}
	
	
}
