/*
 * soft_pwm.c
 *
 *  Created on: Sep 21, 2013
 *      Author: joao
 */
#include <inc/lm3s5749.h>
#include <inc/hw_types.h>
#include <inc/hw_memmap.h>
#include <inc/hw_ints.h>
#include <driverlib/debug.h>
#include <driverlib/sysctl.h>
#include <driverlib/systick.h>
#include <driverlib/gpio.h>
#include <driverlib/pwm.h>
#include <driverlib/interrupt.h>
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "driverlib/timer.h"
#include "soft_pwm.h"
#include "stdint.h"
#include "stdlib.h"

//#define UART_DEBUG

static const uint32_t pin_table [8] = {0, 1, 0, 1, 6, 7, 4, 6};
static const uint32_t port_table [4] = {GPIO_PORTD_BASE, GPIO_PORTB_BASE, GPIO_PORTA_BASE, GPIO_PORTC_BASE};
static const uint32_t periph_table [4] = {SYSCTL_PERIPH_GPIOD, SYSCTL_PERIPH_GPIOB, SYSCTL_PERIPH_GPIOA, SYSCTL_PERIPH_GPIOC};

uint8_t lookUp[50];


void initSoftPWM(unsigned int max_freq, unsigned int res_min)
{
	if(max_freq < 10000 && max_freq > 0 &&
			res_min > 0 && max_freq * res_min <= 500000)
	{
		max_pwm_freq = max_freq;
		min_pwm_res = res_min;
	}
	else
	{
		max_pwm_freq = 500;
		min_pwm_res = 100;
	}
#ifdef UART_DEBUG
	UARTprintf("Freq = %u, min_res = %u\n", max_pwm_freq, min_pwm_res);
#endif


	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);
	TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet() / (max_pwm_freq * min_pwm_res));

	int i;
	for(i = 0; i < MAX_PWM_GENERATORS; i++)
	{
		pwm_counters[i] = 0;
		max_count[i] = 0;
		config_done[i] = 0;
		compare_value[i] = 0;
	}
}

void enablePWM(void)
{
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	TimerEnable(TIMER0_BASE, TIMER_A);
}

uint8_t setPWMGenFreq(uint8_t generator, unsigned int freq)
{
	config_done[generator - 1] = 0;
	if(generator <= MAX_PWM_GENERATORS && generator > 0)
	{
		if(freq > 0 && freq <= max_pwm_freq)
		{
			freq_pwm[generator - 1] = freq;
			max_count[generator -1] = max_pwm_freq * min_pwm_res / freq_pwm[generator - 1];
			config_done[generator - 1] = 1;

#ifdef UART_DEBUG
			UARTprintf(" max count  %u", max_count[generator - 1] );
			UARTprintf("Freq = %u, min_res = %u\n", max_pwm_freq, min_pwm_res);
			UARTprintf("port = %x, pin = %x\n", port_table[generator - 1], (1 << pin_table[(generator - 1) * 2]) | (1 << pin_table[(generator - 1) * 2 + 1]));
#endif
			SysCtlPeripheralEnable(periph_table[generator - 1]);
			GPIOPinTypeGPIOOutput(port_table[generator - 1], (1 << pin_table[(generator - 1) * 2]) | (1 << pin_table[(generator - 1) * 2 + 1]));

			lookUp_pwm[(generator - 1) * 2] = (uint8_t *)malloc(max_count[generator - 1]);
			lookUp_pwm[(generator - 1) * 2 + 1] = (uint8_t *)malloc(max_count[generator - 1]);

			int i;
			for(i = 0; i < max_count[generator - 1]; i++)
			{
				lookUp_pwm[(generator - 1) * 2][i]= 0;
				lookUp_pwm[(generator - 1) * 2 + 1][i] = 0;
			}

			lookUp_pwm[(generator - 1) * 2 ][10] = 1;
			lookUp_pwm[(generator - 1) * 2 + 1][11] = 1;

#ifdef UART_DEBUG
			UARTprintf("printing tab...");
			for(i = 0; i < max_count[generator - 1]; i++)
			{
				UARTprintf("%d ", lookUp_pwm[(generator - 1) * 2 ][i]);
			}
			UARTprintf("pr \n");
			for(i = 0; i < max_count[generator - 1]; i++)
						{
							UARTprintf("%d ", lookUp_pwm[(generator - 1) * 2 +1][i]);
						}
			UARTprintf("done\n");
#endif
			return 0;
		}
		else
		{
			return 1;
		}
	}
	else
	{
		return 2;
	}
}

void updateSoftPWM(unsigned char index)
{
//	GPIOPinWrite(GPIO_PORTD_BASE, 1 << pin_table[index], lookUp[pwm_counters[index]]);
	GPIOPinWrite(port_table[index], 1 << pin_table[index<<1], (lookUp_pwm[index<<1][pwm_counters[index]])<< pin_table[index<<1]);
	GPIOPinWrite(port_table[index], 1 << pin_table[(index<<1) + 1], (lookUp_pwm[(index << 1) + 1][pwm_counters[index]])<< pin_table[(index<<1)+1]);
	pwm_counters[index] = (pwm_counters[index] + 1) % max_count[index];

#ifdef UART_DEBUG
	//UARTprintf(" max count  %u, port %x, pin = %x , pin = %x\n", max_count[index], port_table[index], pin_table[index * 2], pin_table[index * 2 + 1]);
#endif
}

uint8_t setSoftPWMDuty(uint8_t pwm, unsigned long int dcycle)
{
	if(pwm >= 0 && pwm <= (MAX_PWM_GENERATORS * 2))
	{
		if(dcycle < getSoftPWMPeriod(pwm/2 + 1) && dcycle >= 0)
		{
			compare_value[pwm] = dcycle;
			return 0;
		}
		else if(dcycle >= getSoftPWMPeriod(pwm/2 + 1))
		{
			compare_value[pwm] = 0;
			return 1;
		}
		else
		{
			compare_value[pwm] = 0;
			return 2;
		}
	}
	else
	{
		return 3;
	}
}

int32_t getSoftPWMPeriod(uint8_t generator)
{
	if(config_done[generator - 1])
	{
		return max_count[generator - 1];
	}
	else
	{
		return -1;
	}
}

void Timer0IntHandler(void)
{
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_PIN_7);
	unsigned char i;
	for(i = 0; i < MAX_PWM_GENERATORS; i++)
	{
		if(config_done[i])
		{
			updateSoftPWM(i);
		}
	}
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_7, 0);

}

