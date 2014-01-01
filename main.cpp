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

	UARTprintf("Configuring GPIO...");
	ConfigureGPIO();
	UARTprintf("done\n");

	UARTprintf("Configuring NRF24L01...");

	RF24 radio = RF24();

	// Radio pipe addresses for the 2 nodes to communicate.
	const uint64_t pipes[3] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL, 0xF0F0F0F0C3LL};

	// Setup and configure rf radio
	radio.begin();

	// optionally, increase the delay between retries & # of retries
	radio.setRetries(15,15);

	// optionally, reduce the payload size.  seems to
	// improve reliability
	radio.setPayloadSize(sizeof(RC_remote));

	radio.setDataRate(RF24_250KBPS);

	// Open pipes to other nodes for communication
	radio.openWritingPipe(pipes[1]);
	radio.openReadingPipe(1,pipes[0]);

	// Start listening
	radio.startListening();

	// Dump the configuration of the rf unit for debugging
	radio.printDetails();

	radio.stopListening();
	radio.startListening();

	UARTprintf("done\n");

	UARTprintf("Configuring pwm...");
	UARTprintf("%u", SysCtlClockGet());

	initSoftPWM(500,40);
	servo_init();
	enablePWM();
	RC_remote ferrari;
	//
	// Loop forever.
	//
	while (1)
	{

		// if there is data ready
		if ( radio.available() )
		{
			bool done = false;
			while (!done)
			{

				// Fetch the payload, and see if this was the last one.
				done = radio.read( &ferrari, sizeof(RC_remote));

				if(done)
				{
					//UARTprintf("rec %d\n", ferrari.steer);
					servo_setPosition((ferrari.steer + 127) / 2);


				}
			}
		}
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
