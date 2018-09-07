#include <atmel_start.h>
#include <Config/peripheral_clk_config.h>

#include "RepRapFirmware.h"
#include "Tasks.h"
#include <HAL/DmacManager.h>
#include <HAL/AnalogIn.h>
#include "AdcAveragingFilter.h"

#define USE_RTOS	1

const size_t NumReadingsAveraged = 32;

// Diagnostic LED
#define DIAG_LED GPIO(GPIO_PORTC, 10)

// SERCOM3 Rx is on PB21 (OUT_8_TACHO), Tx is on PB20 (OUT_7_TACHO)

static void tx_cb_USART_0(const struct usart_async_descriptor *const io_descr)
{
  /* Transfer completed */
}

static struct io_descriptor *io;

static AdcAveragingFilter<NumReadingsAveraged> vinFilter;

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	SystemCoreClock = CONF_CPU_FREQUENCY;			// FreeRTOS needs this to be set correctly because it uses it to set the systick reload value
	DmacInit();
	AnalogIn::Init();

	AppMain();
}

void Init()
{
	// Set pin direction to output
	gpio_set_pin_direction(DIAG_LED, GPIO_DIRECTION_OUT);

	gpio_set_pin_level(DIAG_LED, true);

	gpio_set_pin_function(DIAG_LED, GPIO_PIN_FUNCTION_OFF);

	const uint32_t baudDiv = 65536 - ((65536 * 16.0f * 57600) / CONF_GCLK_SERCOM3_CORE_FREQUENCY);
	usart_async_set_baud_rate(&USART_0, baudDiv);

    usart_async_register_callback(&USART_0, USART_ASYNC_TXC_CB, tx_cb_USART_0);
    //usart_async_register_callback(&USART_0, USART_ASYNC_RXC_CB, rx_cb);
    //usart_async_register_callback(&USART_0, USART_ASYNC_ERROR_CB, err_cb);
    usart_async_get_io_descriptor(&USART_0, &io);
    usart_async_enable(&USART_0);

    // Set up ADC to read VIN
    AnalogIn::EnableChannel(VinMonitorPin, vinFilter.CallbackFeedIntoFilter, &vinFilter);
}

static float count = 0.0;

void Spin()
{
	delay(500);
	gpio_toggle_pin_level(DIAG_LED);
	String<100> str;
	float volts = (vinFilter.GetSum() * (3.3 * 11))/(4096 * NumReadingsAveraged);
	str.printf("{\"message\":\"Hello World! %.1f, %.1fV\"}\n", (double)count, (double)volts);
	io_write(io, (const unsigned char *)str.c_str(), str.strlen());
	count += 1.0;
}

// End
