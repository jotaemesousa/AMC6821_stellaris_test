#include <inc/lm3s5749.h>
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <driverlib/debug.h>
#include <driverlib/sysctl.h>
#include <driverlib/systick.h>
#include <driverlib/gpio.h>
#include <driverlib/timer.h>
#include <driverlib/interrupt.h>
#include "utils/uartstdio.h"
#include "driverlib/timer.h"
#include "soft_pwm.h"
#include "stdbool.h"

static unsigned long ulClockMS = 0;


void ConfigureGPIO(void);


int main(void) {
	//SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_12MHZ);
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_12MHZ);


	ulClockMS = SysCtlClockGet() / (3 * 1000);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTStdioInit(0);

	UARTprintf("Configuring GPIO...");
	ConfigureGPIO();
	UARTprintf("done\n");


	UARTprintf("Configuring pwm...");
    UARTprintf("%u", SysCtlClockGet());
	initSoftPWM(500,50);
	setPWMGenFreq(1,500);
	setPWMGenFreq(2,250);
	setPWMGenFreq(3,100);
	setPWMGenFreq(4,50);
	setSoftPWMDuty(0,1);
	setSoftPWMDuty(1,2);
	setSoftPWMDuty(2,3);
	setSoftPWMDuty(3,4);
	setSoftPWMDuty(4,5);
	setSoftPWMDuty(5,6);
	setSoftPWMDuty(6,7);
	setSoftPWMDuty(7,8);
//	setPWMGenFreq(2,500);
//	setPWMGenFreq(3,500);
//	setPWMGenFreq(4,500);
//	UARTprintf("done\n");
//	updateSoftPWM(0);
//	updateSoftPWM(1);
//	updateSoftPWM(2);
//	updateSoftPWM(3);
//	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_PIN_7);


	//
	// Loop forever.
	//
	while (1)
	{

	}
}

void ConfigureGPIO(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_7);
	//GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_PIN_7);
}

