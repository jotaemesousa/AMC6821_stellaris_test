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

static const uint32_t pin_table [8] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_4, GPIO_PIN_6};
static const uint32_t port_table [4] = {GPIO_PORTD_BASE, GPIO_PORTB_BASE, GPIO_PORTA_BASE, GPIO_PORTC_BASE};
static const uint32_t periph_table [4] = {SYSCTL_PERIPH_GPIOD, SYSCTL_PERIPH_GPIOB, SYSCTL_PERIPH_GPIOA, SYSCTL_PERIPH_GPIOC};

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

	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	TimerEnable(TIMER0_BASE, TIMER_A);

	int i;
	for(i = 0; i < MAX_PWM_GENERATORS; i++)
	{
		pwm_counters[i] = 0;
		max_count[i] = 0;
		config_done[i] = 0;
		compare_value[i] = 0;
	}
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
			UARTprintf("port = %x, pin = %x\n", port_table[generator - 1], pin_table[(generator - 1) * 2] | pin_table[(generator - 1) * 2 + 1]);
#endif
			SysCtlPeripheralEnable(periph_table[generator - 1]);
			GPIOPinTypeGPIOOutput(port_table[generator - 1], pin_table[((generator - 1) * 2)] | pin_table[((generator - 1) * 2 + 1)]);

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
	if(pwm_counters[index] < compare_value[index*2])
	{
		GPIOPinWrite(port_table[index], pin_table[index * 2], pin_table[index * 2]);
	}
	else
	{
		GPIOPinWrite(port_table[index], pin_table[index * 2], 0);
	}
	if(pwm_counters[index] < compare_value[index*2+1])
	{
		GPIOPinWrite(port_table[index], pin_table[index * 2 + 1], pin_table[index * 2 + 1]);
	}
	else
	{
		GPIOPinWrite(port_table[index], pin_table[index * 2 + 1], 0);
	}

	pwm_counters[index]++;

	if(pwm_counters[index] >= max_count[index])
	{
		pwm_counters[index] = 0;
	}
#ifdef UART_DEBUG
	UARTprintf(" max count  %u, port %x, pin = %x , pin = %x\n", max_count[index], port_table[index], pin_table[index * 2], pin_table[index * 2 + 1]);
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

