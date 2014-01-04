extern "C"
{
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
}

#include "rf24/RF24.h"
#include "libraries/AMC6821/AMC6821.h"

extern "C"
{
void SysTickHandler();
uint32_t millis();
}

static unsigned long milliSec = 0;


static unsigned long ulClockMS=0;

void ConfigureGPIO(void);

typedef struct ROSpberryRemote
{
	int16_t linear;
	int16_t steer;
	uint8_t buttons;

}RC_remote;

int main(void) {
	//SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_12MHZ);
	SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_12MHZ);


	ulClockMS = SysCtlClockGet() / (3 * 1000);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTStdioInit(0);

//	UARTprintf("Configuring GPIO...");
//	//ConfigureGPIO();
//	UARTprintf("done\n");

	UARTprintf("Setting up I2C\n");

	//I2C
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeI2C(GPIO_PORTB_BASE,GPIO_PIN_2 | GPIO_PIN_3);
	I2CMasterInitExpClk(I2C0_MASTER_BASE,SysCtlClockGet(),false);  //false = 100khz , true = 400khz
	I2CMasterTimeoutSet(I2C0_MASTER_BASE, 1000);

	UARTprintf("done\n");

	UARTprintf("Fan controller...");

	AMC6821 fan_ctl(0b00011000);

//	fan_ctl.setPWMINV();
	fan_ctl.reset();
	SysCtlDelay(1000*ulClockMS);
	fan_ctl.setPWMINV();
	fan_ctl.setSTART(true);
	int f;
	f = fan_ctl.getPWM();
	//fan_ctl.setDCY(0);
	fan_ctl.setFDRC(amc6821_fdrc_software_rpm);
	fan_ctl.setPSV(20);

	UARTprintf("done\n");

	// Loop forever.
	//
	uint16_t lbyte = 0;
	uint16_t local_temp = 0;
	uint16_t remote_temp = 0;
	uint16_t x = 0;
	while (1)
	{
		fan_ctl.readTemp11bits(local_temp, remote_temp);

		UARTprintf("local tem = %d.%d, remote temp = %d.%d\n", local_temp>>3, (local_temp & 0x0007)*1000/8, remote_temp>>3, (remote_temp & 0x0007)*1000/8);

		x = fan_ctl.readRPM();
		UARTprintf("rpm = %d\n", x);
		SysCtlDelay(1000*ulClockMS);

//		fan_ctl.setPWM((amc6821_pwm_frequency)1);
//		x = fan_ctl.getPWM();
//		SysCtlDelay(1000*ulClockMS);
//		UARTprintf("PWM = %u\n", x);

	}
}

void ConfigureGPIO(void)
{
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // enable port
	MAP_SysCtlGPIOAHBEnable(SYSCTL_PERIPH_GPIOB); //enable ahb

	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_AHB_BASE,GPIO_PIN_7); // portb as output
	HWREG(GPIO_PORTB_AHB_BASE + GPIO_O_DATA + ((GPIO_PIN_7 ) << 2)) = (GPIO_PIN_7 );  //portb high


}

void SysTickHandler()
{
	milliSec++;

}

uint32_t millis()
{
	return milliSec;
}
