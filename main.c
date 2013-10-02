#include <inc/lm3s5749.h>
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <inc/hw_gpio.h>
#include <driverlib/rom.h>
#include <driverlib/rom_map.h>
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
#include "servo.h"

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
	initSoftPWM(500,40);
	setPWMGenFreq(1,400);
	//servo_init();
//	setPWMGenFreq(2,50);
//	setPWMGenFreq(3,100);
//	setPWMGenFreq(4,50);
	//setSoftPWMDuty(0,1);
	//setSoftPWMDuty(1,2);
	//setSoftPWMDuty(2,(47 + 14)/2);	//min 14, max 47
	//setSoftPWMDuty(3,4);
//	setSoftPWMDuty(4,5);
//	setSoftPWMDuty(5,6);
//	setSoftPWMDuty(6,7);
//	setSoftPWMDuty(7,8);
//	setPWMGenFreq(2,500);
//	setPWMGenFreq(3,500);
//	setPWMGenFreq(4,500);
//	UARTprintf("done\n");
//	updateSoftPWM(0);
//	updateSoftPWM(1);
//	updateSoftPWM(2);
//	updateSoftPWM(3);
//	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_PIN_7);

	enablePWM();
//	servo_setPosition(150);
	//
	// Loop forever.
	//
	while (1)
	{
//		servo_setPosition(60);
////		setSoftPWMDuty(2, 23);
//		SysCtlDelay(1000*ulClockMS);
//////		setSoftPWMDuty(2, 23);
//////		SysCtlDelay(500*ulClockMS);
////		setSoftPWMDuty(2, 27);
//		servo_setPosition(30);
//		SysCtlDelay(1000*ulClockMS);
	}
}

void ConfigureGPIO(void)
{
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // enable port
	MAP_SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOB); //enable ahb

	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_AHB_BASE,GPIO_PIN_7); // portb as output
	HWREG(GPIO_PORTB_AHB_BASE + GPIO_O_DATA + ((GPIO_PIN_7 ) << 2)) = (GPIO_PIN_7 );  //portb high


}

