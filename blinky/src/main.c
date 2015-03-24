//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"

#include "Timer.h"
#include "BlinkLed.h"
#include "serial.h"
#include "nrf24.h"
#include "adc.h"
#include "tim.h"
#include "ws2812.h"

#define RF_CHANNEL      0x3C      // Stock TX fixed frequency
#define PAYLOADSIZE       9          // Protocol packet size
const char rf_addr_bind[5] = { 0x65, 0x65, 0x65, 0x65, 0x65 };

// ----------------------------------------------------------------------------
//
// STM32F0 led blink sample (trace via $(trace)).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// To demonstrate POSIX retargetting, reroute the STDOUT and STDERR to the
// trace device and display messages on both of them.
//
// Then demonstrates how to blink a led with 1Hz, using a
// continuous loop and SysTick delays.
//
// On DEBUG, the uptime in seconds is also displayed on the trace device.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// The external clock frequency is specified as a preprocessor definition
// passed to the compiler via a command line option (see the 'C/C++ General' ->
// 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
// The value selected during project creation was HSE_VALUE=8000000.
//
// Note: The default clock settings take the user defined HSE_VALUE and try
// to reach the maximum possible system clock. For the default 8MHz input
// the result is guaranteed, but for other values it might not be possible,
// so please adjust the PLL settings in system/src/cmsis/system_stm32f0xx.c
//

// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.
#define BLINK_ON_TICKS  (TIMER_FREQUENCY_HZ * 1 / 10)
#define BLINK_OFF_TICKS (TIMER_FREQUENCY_HZ * 1/10)

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

//const uint64_t pipes[2] = { 0x0102030405LL, 0x0102030405LL };
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

int main(int argc, char* argv[]) {
	// By customising __initialize_args() it is possible to pass arguments,
	// for example when running tests with semihosting you can pass various
	// options to the test.
	// trace_dump_args(argc, argv);

	// Send a greeting to the trace device (skipped on Release).
	trace_puts("Hello ARM World!");

	// At this stage the system clock should have already been configured
	// at high speed.
	trace_printf("System clock: %uHz\n", SystemCoreClock);

	timer_start();

	blink_led_init();

	uint32_t seconds = 0;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(
	RCC_APB1Periph_I2C1 | RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(
			RCC_APB2Periph_SPI1 | RCC_APB2Periph_USART1 | RCC_APB2Periph_ADC1
					| RCC_APB2Periph_TIM16 | RCC_APB2Periph_TIM15
					| RCC_APB2Periph_TIM1 | RCC_APB2Periph_SYSCFG, ENABLE);

	//init_UART(115200);
	//uint8_t buf[] = "hello world\n\r";
	//serial_send_bytes(buf, 10);

	/*
	 nrf24_init();

	 nrf24_setRetries(15, 15);
	 nrf24_setPayloadSize(8);

	 nrf24_openWritingPipe(pipes[1]);
	 nrf24_openReadingPipe(1, pipes[0]);

	 nrf24_startListening();

	 nrf24_printDetails();
	 */

	/*
	 while (1) {
	 // First, stop listening so we can talk.
	 nrf24_stopListening();

	 // Take the time, and send it.  This will block until complete
	 unsigned long time = system_ticks;
	 trace_printf("Now sending %lu...", time);
	 uint8_t ok = nrf24_write(&time, sizeof(unsigned long));

	 if (ok)
	 trace_printf("ok...");
	 else
	 trace_printf("failed.\n\r");

	 // Now, continue listening
	 nrf24_startListening();

	 // Wait here until we get a response, or timeout (250ms)
	 unsigned long started_waiting_at = system_ticks;
	 uint8_t timeout = 0;
	 while (!nrf24_available(NULL)  && !timeout)
	 if (system_ticks - started_waiting_at > 500)
	 timeout = 1;

	 // Describe the results
	 if (timeout) {
	 printf("Failed, response timed out.\n\r");
	 } else {
	 // Grab the response, compare, and send to debugging spew
	 unsigned long got_time;
	 nrf24_read(&got_time, sizeof(unsigned long));

	 // Spew it
	 trace_printf("Got response %lu, round-trip delay: %lu\n\r", got_time,
	 system_ticks - got_time);
	 }

	 // Try again 1s later
	 timer_sleep(1000);

	 }
	 //*/

	/*
	 while (1) {

	 if (nrf24_available(NULL)) {
	 // Dump the payloads until we've gotten everything
	 unsigned long got_time;
	 uint8_t done = 0;
	 while (!done) {
	 // Fetch the payload, and see if this was the last one.
	 done = nrf24_read(&got_time, sizeof(unsigned long));

	 // Spew it
	 trace_printf("Got payload %lu...", got_time);

	 // Delay just a little bit to let the other unit
	 // make the transition to receiver
	 timer_sleep(20);
	 }

	 // First, stop listening so we can talk
	 nrf24_stopListening();

	 // Send the final one back.
	 nrf24_write(&got_time, sizeof(unsigned long));
	 printf("Sent response.\n\r");

	 // Now, resume listening so we catch the next packets.
	 nrf24_startListening();
	 }
	 }
	 //*/
	// Infinite loop
	adc_init();
	tim_init();

	#define BLACK {0x00, 0x00, 0x00}
	#define BLUE {0x



	ws2812_init(8);
	//ws2812_set_pixel(0, 128,128,0xAA);
	//ws2812_set_pixel(1, 129,129,0xAA);

	ws2812_send();

	while (1) {

		//while((DMA_GetFlagStatus(DMA1_FLAG_TC1)) == RESET );

		/* Clear DMA TC flag */
		//DMA_ClearFlag(DMA1_FLAG_TC1);
		trace_printf("analog value ");
		int i;
		for (i = 0; i < 4; i++)
			trace_printf(" %d", analog_val[i]);
		trace_printf("\n");
		TIM3->CCR1 = analog_val[0];
		TIM3->CCR2 = analog_val[1];
		TIM3->CCR3 = analog_val[2];
		TIM3->CCR4 = analog_val[3];


		ws2812_set_pixel(1, 128,0,0);
		ws2812_set_pixel(5, 0,analog_val[3]>>5,0);
		ws2812_send();


		timer_sleep(100);
	}

	while (1) {

		blink_led_on();
		timer_sleep(BLINK_ON_TICKS);

		//serial_send_bytes("*",1);

		blink_led_off();
		timer_sleep(BLINK_OFF_TICKS);

		++seconds;

		// Count seconds on the trace device.
		//trace_printf("Second %u\n", seconds);
	}
	// Infinite loop, never return.
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
