/* **********************************************************************
 * AVR-GCC source code for UWC-Open-MPP-Solar-Tracker
 * Original code by Corinna 'Elektra' Aichele
 * Code changes/Updates/Added features by DPW
 *
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This source code is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this source file. If not, see http://www.gnu.org/licenses/.
 *
 * This is currently test code, with some changes needed to be made in
 * the final version.
 *
 * For example:
 * 1. Opening and closing of the relay is done by shorting a DIP switch.
 * Once a battery tech has been decided, this needs to be automated.
 * 2. The MPPT algorythm is basic at the moment, but a full hill-climbing
 * and sweep algorythm has been implemented.
 *
 *
 *************************************************************************/

#define DEBUG TRUE
#define CALIBRATION_BATTERY 27
#define CALIBRATION_SOLAR  51// 50.73
#define CALIBRATION_TEMPERATURE 5
#define BATTERY_EMPTY 9000
#define BATTERY_FULL 11000
#define BATTERY_MAX 13800
#define BATTERY_PARALLEL_UPPER 10300
#define BATTERY_PARALLEL_LOWER 9700
#define SOLAR_VOC 16000 // should be 18000


//defines needed for Atmega328:
#define    UCSRA    UCSR0A
#define    UCSRB    UCSR0B
#define    UCSRC    UCSR0C
#define    UBRRH    UBRR0H
#define    UBRRL    UBRR0L
#define    UDRE    UDRE0
#define    UDR    UDR0
#define    RXC    RXC0
#define    RXEN    RXEN0
#define    TXEN    TXEN0
#define    UCSZ1    UCSZ01
#define    UCSZ0    UCSZ00

#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

/* UART init */
/*
#ifndef F_CPU
#warning "F_CPU not defined in Makefile. Using 3.686400 MHz"
#define F_CPU 3686400UL
#endif
*/

#define F_CPU 16000000UL
/* Define baud rate for serial port */
#define USART_BAUDRATE 28800UL

/* Calculate UART BAUD register setting */
//#define UBRR_SETTING ((F_CPU+BAUD*8)/(BAUD*16)-1)
#define UBRR_SETTING ((F_CPU /16UL) / USART_BAUDRATE) - 1

uint8_t led_heartbeat_counter;
uint8_t led_battery_counter;
uint16_t battery_voltage_adc;
uint16_t solar_voltage_adc;
uint16_t solar_voltage;
uint16_t current_out_adcval;
uint16_t battery_voltage;
uint16_t u_zero_current;
char vo[] = "";
uint8_t duty;
uint16_t upper_mpp_current_value;
uint16_t lower_mpp_current_value;
uint16_t medium_mpp_current_value;
uint16_t v_mpp_estimate = 0;
uint8_t step = 0x1; // Three point measurement step size
uint16_t temperature = 0;
uint16_t adc_temperature = 0;
uint16_t mppt_pwm = 0;
volatile uint16_t ticks = 0;

/* ADC init */
void ADC_Init(void)
{
  // Reference: Use Vcc as AVcc
  ADMUX = (1<<REFS0);

  /* Bit ADFR ("free running") in ADCSRA is zero
   * by default which means single conversion */

  // Enable frequency divider
  ADCSRA = (1<<ADPS1) | (1<<ADPS0);
  // set ADC enable
  ADCSRA |= (1<<ADEN);

  /* After activating the ADC, a warm-up readout is
   * recommended to increase accuracy */

  // run ADC readout
  ADCSRA |= (1<<ADSC);
  // wait until finished
  while (ADCSRA & (1<<ADSC) ) {
  }
}



/* ADC read, single conversion */
uint16_t ADC_Read( uint8_t channel )
{
  /* Select ADC channel */
  ADMUX = (ADMUX & ~(0x1F)) | (channel & 0x1F);
  // single conversion
  ADCSRA |= (1<<ADSC);
  // wait for conversion until finished
  while (ADCSRA & (1<<ADSC) ) {
  }
  return ADCW;
}



/* Multiple ADC readouts, calculate average	 */
uint16_t ADC_Read_Avg( uint8_t channel, uint8_t nsamples )
	{
	uint32_t sum = 0;

	for (uint8_t i = 0; i < nsamples; ++i ) {
		sum += ADC_Read( channel );
	}

	return (uint16_t)( sum / nsamples );
	}


/* UART init */
void uart_tx_enable(void)
{
	UBRRH = (UBRR_SETTING >> 8);
	UBRRL = (UBRR_SETTING & 0xFF);

	// enable UART TX
	UCSRB |= (1<<TXEN);
	// select asynchronous mode 8N1
	//UCSRC = (1<<URSEL)|(1<<UCSZ1)|(1<<UCSZ0);
	// asynchronous mode 8N1
	UCSRC = (1 << UCSZ1) | (1 << UCSZ0);
}


/* UART disable */
void uart_tx_disable(void)
	{
		UCSRB |= (0<<TXEN);	 // disable UART TX
		}

/* UART send single character */
int uart_putc(unsigned char c) {
	// wait until transmit is possible
	while (!(UCSRA & (1<<UDRE))) {
	}
// send character
	UDR = c;
	return 0;
}


/* UART send string */
void uart_puts(char *s)
	{
		while (*s)
		{	//transmit as long as  *s != '\0'
			uart_putc(*s);
			s++;
		}
	}

/* Low voltage disconnect */
void low_voltage_disconnect (uint16_t voltage) {/*
	// Battery voltage level that enables load in mV
	if (voltage > 12300)
	{
		PORTD |= (1 << PD7); // Set load on
		uart_puts("Load enabled\r\n");
	}
	 // Battery voltage level that enables load in mV
	if (voltage < 11800)
	{
		PORTD |= (1 << PD7);
		uart_puts("Load disabled\r\n");
	}*/
}

/* Power saving sleep routine based on counter/timer2
 * Sleep (idle) time with 3.684 MHz clock is 70.6ms
void interrupt_based_sleep (void)
	{
	TIFR |= (1<<TOV2);
	// enable counter2 overflow interrupt
	TIMSK |= (1<<TOIE2);
	// Set counter2 to zero
	TCNT2 = 0x00;
	sei();
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();
	sleep_mode();
	cli();
	}*/

/* ISR TIMER2 overflow routine */
ISR(TIMER2_OVF_vect) {
	led_heartbeat_counter++;
	if (led_heartbeat_counter > 1) {
		led_heartbeat_counter = 0;
		PORTD ^= (1 << PD4); // Toggle LED
	}
}

void readDIP (void) {
	uart_puts("DIP SWITCH STATUS:\r\n");
	uart_puts("DIP 1: ");

	if (PINB & (1<<PB2)) {
		uart_puts("ON: Router On");
		PORTD |= (1 << PD7); // Set router on
	}

	else {
		uart_puts("OFF: Router Off");
		PORTD &= ~(1 << PD7); // Set router off
	}
	uart_puts ("\r\n");

	uart_puts("DIP 2: ");
	if (PINB & (1<<PB3)) {
		uart_puts("ON: USB On");
		PORTD |= (1 << PD2);
	}
	else {
		uart_puts("OFF: USB Off");
		PORTD &= ~(1 << PD2);
	}
	uart_puts ("\r\n");

	uart_puts("DIP 3: ");
	if (PINC & (1<<PC5)) {
		uart_puts("ON: Relay Closed");
		PORTD |= (1 << PD3);
	}
	else {
		uart_puts("OFF: Relay Open");
		PORTD &= ~(1 << PD3);
	}
	uart_puts ("\r\n");

	uart_puts("DIP 4: ");
	if (PINC & (1<<PC4))
		uart_puts("ON");
	else
		uart_puts("OFF");
	uart_puts ("\r\n");

	uart_puts("DIP 5: ");
	if (PINC & (1<<PC3))
		uart_puts("ON");
	else
		uart_puts("OFF");
	uart_puts ("\r\n");

	uart_puts("DIP 6: ");
	if (PINC & (1<<PD2))
		uart_puts("ON");
	else
		uart_puts("OFF");
	uart_puts ("\r\n");
}

void printtemperature(void) {
	adc_temperature = ADC_Read_Avg(7, 32);
	temperature = (CALIBRATION_TEMPERATURE * adc_temperature);
	uart_puts("TEMPERATURE: ");
	itoa( temperature, vo, 10 );
	uart_puts( vo );
	uart_puts(" deg C/100""\r\n");
}


void setLEDs (void) {
	// Set the LEDs according to the new states
	// Update the Battery LED
	if (battery_voltage > BATTERY_FULL) {
		PORTD &= ~(1 << PD5); // Set led off
		uart_puts ("BATTERY: FULL\r\n");
	}
	else if (battery_voltage < BATTERY_EMPTY) {
		PORTD |= (1 << PD5); // Set led on
		uart_puts ("BATTERY: EMPTY\r\n");
	}
	else {
		uart_puts ("BATTERY: MEDIUM\r\n");
		led_battery_counter++;
		if (led_battery_counter > 1)
			led_battery_counter = 0;
		if (led_battery_counter < 1)
			PORTD |= (1 << PD5); // Set led on
		else
			PORTD &= ~(1 << PD5); // Set led off
	}
	// Update the heartbeat
	led_heartbeat_counter++;
	if (led_heartbeat_counter > 1) {
		led_heartbeat_counter = 0;
		PORTD ^= (1 << PD4); // Toggle LED
	}

	// The parallelling LED
	if ((battery_voltage < BATTERY_PARALLEL_UPPER) && (battery_voltage > BATTERY_PARALLEL_LOWER))
		PORTD |= (1 << PD6);
	else
		PORTD &= ~(1 << PD6);
}

void serialdatareport (void)
{
	// Experiment: Prevent serial data from getting garbled.
	_delay_ms(200);
	// Prepare UART for sending data
	uart_tx_enable();
	uart_tx_disable();
}

void readallADC(void) {
	solar_voltage_adc = ADC_Read_Avg(6, 32);
	solar_voltage = (CALIBRATION_SOLAR * solar_voltage_adc);
	uart_puts("Solar Voltage: ");
	itoa( solar_voltage, vo, 10 );
	uart_puts( vo );
	uart_puts(" mV""\r\n");

	battery_voltage_adc = ADC_Read_Avg(0, 32);
	battery_voltage = (CALIBRATION_BATTERY * battery_voltage_adc);
	uart_puts("Battery Voltage: ");
	itoa( battery_voltage, vo, 10 );
	uart_puts( vo );
	uart_puts(" mV""\r\n");
}

void mppt_algorythm(void) {
	v_mpp_estimate = SOLAR_VOC;
	// Condition one. Above battery maximum. Immediatly stop.
	if (battery_voltage >= BATTERY_MAX) {
			uart_puts("PWM MODE: Battery full\r\n");
			OCR1A = 0x00;
			mppt_pwm = 0;
	}

	// Condition two. Approaching limit
	else if (battery_voltage >= (BATTERY_MAX-100)) {
		uart_puts("PWM MODE: Battery approaching full\r\n");
		mppt_pwm = mppt_pwm - 0x10;
	}

	else if ((solar_voltage < v_mpp_estimate) && (battery_voltage < BATTERY_MAX-50))  {
		uart_puts("PWM MODE: Tracking, increasing pwm\r\n");
		mppt_pwm++;
	}

	else if ((solar_voltage > v_mpp_estimate) && (battery_voltage < BATTERY_MAX-50))  {
		uart_puts("PWM MODE: Tracking, decreasing pwm\r\n");
		mppt_pwm--;
	}

	else if (battery_voltage > solar_voltage) {
		uart_puts("PWM MODE: Solar voltage too low\r\n");
	}

	else uart_puts("PWM MODE: Nothing done..\r\n");


	// Do what was decided above
	if (mppt_pwm > 0x350) {
		mppt_pwm = 0x350;
	}

	OCR1A = mppt_pwm;

	uart_puts ("\r\nPWM value: 0x");
	itoa(mppt_pwm, vo, 16 );
	uart_puts (vo);
	uart_puts ("\r\n");
}

int main(void)
{
	uart_puts("Starting MPPT Code ...");
	// Set up GPIOs.
	// Use &= ~ for inputs and |= for outputs.
	DDRD |= (1<<PD2); // PD2: 5V+ Enable
	DDRD |= (1<<PD3); // PD3: Battery Parallel Output
	DDRD |= (1<<PD4); // PD4: HeartBeat LED
	DDRD |= (1<<PD5); // PB5: Battery LED
	DDRD |= (1<<PD6); // PD6: Parallel LED
	DDRD |= (1<<PD7); // PD7: Router Output Enable
	DDRB |= (1<<PB0); // PB0: Load Disable
	DDRB |= (1<<PB1); // PB1: Solar PWM Output
	DDRB &= ~(1<<PB2); // PB2: DIP1
	DDRB &= ~(1<<PB3); // PB3: DIP2
	DDRC &= ~(1<<PC2); // PC2: DIP6
	DDRC &= ~(1<<PC3); // PC3: DIP5
	DDRC &= ~(1<<PC4); // PC4: DIP4
	DDRC &= ~(1<<PC5); // PC5: DIP3

	// Enable ADC
	ADC_Init();

	// Set up PWM1 to control refence voltage of OP-AMP
	TCCR1A|=(1<<COM1A1)|(0<<COM1A0)|(0<<WGM13)|(1<<WGM12)|(1<<WGM11)|(1<<WGM10);
	TCCR1B|=(1<<CS10);
	ICR1=0x3ff;
	OCR1A = 0xf0;

	// Enable Timer/Counter2 to generate interrups for timer based sleep
	//TCCR2 |= ( 1<<CS02 )| ( 1<<CS01)| ( 1<<CS00 );	// Use counter2, set prescaler to 1024

    OCR2A = 62;
    TCCR2A |= (1 << WGM21);    // Set to CTC Mode
    TIMSK2 |= (1 << OCIE2A);   //Set interrupt on compare match
    TCCR2B |= (1 << CS21);    // set prescaler to 64 and starts PWM

    //sei();
    // enable interrupts

	_delay_ms(200);

while (1) {
	uart_puts("MPPT CODE V0.9\r\n----------\r\n");
	readDIP();
	setLEDs();
	printtemperature();
	readallADC();

	mppt_algorythm();

	 //low_voltage_disconnect(battery_voltage);
	 //uart_puts("Parsed low voltage disconnect routine");
	 //uart_puts ("\r\n");

	/* Check if there is actually power from the solar panel.
	 * If not, sleep for a while */
	/* First, measure solar input voltage */

/*
	if ((battery_voltage < solar_voltage) && (battery_voltage < (BATTERY_MAX - 100))) {

		OCR1A = 0x354;
		_delay_ms(400);
		solar_voltage_adc = ADC_Read_Avg(6, 32);
		solar_voltage = (CALIBRATION_SOLAR * solar_voltage_adc);
		uart_puts("V_in_idle ");
		itoa( solar_voltage, vo, 10 );
		uart_puts( vo );
		uart_puts(" mV""\r\n");
		v_mpp_estimate = solar_voltage / 1.24;

		OCR1A = 0x0;
		_delay_ms(200);
		solar_voltage_adc = ADC_Read_Avg(6, 32);
		solar_voltage = (CALIBRATION_SOLAR * solar_voltage_adc);
		uart_puts("Minimum V_mpp ");
		itoa( solar_voltage, vo, 10 );
		uart_puts( vo );
		uart_puts(" mV""\r\n");

		uart_puts("Calculated V_mpp ");
		itoa( v_mpp_estimate, vo, 10 );
		uart_puts( vo );
		uart_puts(" mV""\r\n");

		while ((solar_voltage < v_mpp_estimate) && (battery_voltage < BATTERY_MAX))  {
			OCR1A += step;
			_delay_ms(1);
			solar_voltage_adc = ADC_Read_Avg(6, 32);
			solar_voltage = (CALIBRATION_SOLAR * solar_voltage_adc);

			// Measure battery voltage
			battery_voltage_adc = ADC_Read_Avg(0, 32);
			battery_voltage = (CALIBRATION_BATTERY * battery_voltage_adc);
		}

		ticks = 0;
		uart_puts("Going to sleep. MPP should be set");
		uart_puts ("\r\n");

		while (ticks != 200) {
			//interrupt_based_sleep();
			charge_end_limit();
			ticks ++;
		}
	}*/


/*
		while (ticks != 200) {
			//interrupt_based_sleep();
			ticks ++;
		}*/
	//}

	//charge_end_limit();
	serialdatareport();
	_delay_ms(5000);
}

	return 0; // never reached
}
