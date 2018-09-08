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

// Temperature sense stuff
#define NVM_TEMP_CAL_TLI_POS 0
#define NVM_TEMP_CAL_TLI_SIZE 8
#define NVM_TEMP_CAL_TLD_POS 8
#define NVM_TEMP_CAL_TLD_SIZE 4
#define NVM_TEMP_CAL_THI_POS 12
#define NVM_TEMP_CAL_THI_SIZE 8
#define NVM_TEMP_CAL_THD_POS 20
#define NVM_TEMP_CAL_THD_SIZE 4
#define NVM_TEMP_CAL_VPL_POS 40
#define NVM_TEMP_CAL_VPL_SIZE 12
#define NVM_TEMP_CAL_VPH_POS 52
#define NVM_TEMP_CAL_VPH_SIZE 12
#define NVM_TEMP_CAL_VCL_POS 64
#define NVM_TEMP_CAL_VCL_SIZE 12
#define NVM_TEMP_CAL_VCH_POS 76
#define NVM_TEMP_CAL_VCH_SIZE 12

int16_t temp_cal_tl, temp_cal_th;
int16_t temp_cal_vpl, temp_cal_vph, temp_cal_vcl, temp_cal_vch;

// SERCOM3 Rx is on PB21 (OUT_8_TACHO), Tx is on PB20 (OUT_7_TACHO)

static void tx_cb_USART_0(const struct usart_async_descriptor *const io_descr)
{
  /* Transfer completed */
}

static struct io_descriptor *io;

static AdcAveragingFilter<NumReadingsAveraged> vinFilter;
static AdcAveragingFilter<NumReadingsAveraged> tpFilter;
static AdcAveragingFilter<NumReadingsAveraged> tcFilter;

void ADC_temperature_init(void)
{
	temp_cal_vpl = (*((uint32_t *)(NVMCTRL_TEMP_LOG_W0) + (NVM_TEMP_CAL_VPL_POS / 32)) >> (NVM_TEMP_CAL_VPL_POS % 32))
	               & ((1 << NVM_TEMP_CAL_VPL_SIZE) - 1);
	temp_cal_vph = (*((uint32_t *)(NVMCTRL_TEMP_LOG_W0) + (NVM_TEMP_CAL_VPH_POS / 32)) >> (NVM_TEMP_CAL_VPH_POS % 32))
	               & ((1 << NVM_TEMP_CAL_VPH_SIZE) - 1);
	temp_cal_vcl = (*((uint32_t *)(NVMCTRL_TEMP_LOG_W0) + (NVM_TEMP_CAL_VCL_POS / 32)) >> (NVM_TEMP_CAL_VCL_POS % 32))
	               & ((1 << NVM_TEMP_CAL_VCL_SIZE) - 1);
	temp_cal_vch = (*((uint32_t *)(NVMCTRL_TEMP_LOG_W0) + (NVM_TEMP_CAL_VCH_POS / 32)) >> (NVM_TEMP_CAL_VCH_POS % 32))
	               & ((1 << NVM_TEMP_CAL_VCH_SIZE) - 1);

	const uint8_t temp_cal_tli = (*((uint32_t *)(NVMCTRL_TEMP_LOG_W0) + (NVM_TEMP_CAL_TLI_POS / 32)) >> (NVM_TEMP_CAL_TLI_POS % 32))
	               & ((1 << NVM_TEMP_CAL_TLI_SIZE) - 1);
	const uint8_t temp_cal_tld = (*((uint32_t *)(NVMCTRL_TEMP_LOG_W0) + (NVM_TEMP_CAL_TLD_POS / 32)) >> (NVM_TEMP_CAL_TLD_POS % 32))
	               & ((1 << NVM_TEMP_CAL_TLD_SIZE) - 1);
	temp_cal_tl = ((uint16_t)temp_cal_tli) << 4 | ((uint16_t)temp_cal_tld);

	const uint8_t temp_cal_thi = (*((uint32_t *)(NVMCTRL_TEMP_LOG_W0) + (NVM_TEMP_CAL_THI_POS / 32)) >> (NVM_TEMP_CAL_THI_POS % 32))
	               & ((1 << NVM_TEMP_CAL_THI_SIZE) - 1);
	const uint8_t temp_cal_thd = (*((uint32_t *)(NVMCTRL_TEMP_LOG_W0) + (NVM_TEMP_CAL_THD_POS / 32)) >> (NVM_TEMP_CAL_THD_POS % 32))
	               & ((1 << NVM_TEMP_CAL_THD_SIZE) - 1);
	temp_cal_th = ((uint16_t)temp_cal_thi) << 4 | ((uint16_t)temp_cal_thd);
}

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	SystemCoreClock = CONF_CPU_FREQUENCY;			// FreeRTOS needs this to be set correctly because it uses it to set the systick reload value
	DmacInit();
	AnalogIn::Init();
	ADC_temperature_init();

	AppMain();
}

void Init()
{
	// Set up the DIAG LED pin
	gpio_set_pin_direction(DIAG_LED, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(DIAG_LED, true);

	// Set up the UART to send to PanelDue
	const uint32_t baudDiv = 65536 - ((65536 * 16.0f * 57600) / CONF_GCLK_SERCOM3_CORE_FREQUENCY);
	usart_async_set_baud_rate(&USART_0, baudDiv);

	usart_async_register_callback(&USART_0, USART_ASYNC_TXC_CB, tx_cb_USART_0);
	//usart_async_register_callback(&USART_0, USART_ASYNC_RXC_CB, rx_cb);
	//usart_async_register_callback(&USART_0, USART_ASYNC_ERROR_CB, err_cb);
	usart_async_get_io_descriptor(&USART_0, &io);
	usart_async_enable(&USART_0);

	// Set up ADC to read VIN and the temperature sensors
	vinFilter.Init(0);
	tpFilter.Init(0);
	tcFilter.Init(0);
	AnalogIn::EnableChannel(VinMonitorPin, vinFilter.CallbackFeedIntoFilter, &vinFilter);
	AnalogIn::EnableTemperatureSensor(0, tpFilter.CallbackFeedIntoFilter, &tpFilter);
	AnalogIn::EnableTemperatureSensor(1, tcFilter.CallbackFeedIntoFilter, &tcFilter);
}

void Spin()
{
	delay(500);
	gpio_toggle_pin_level(DIAG_LED);

	// Get the VIN voltage
	const float volts = (vinFilter.GetSum() * (3.3 * 11))/(4096 * NumReadingsAveraged);

	// Get the chip temperature
	const int16_t tc_result = tcFilter.GetSum()/NumReadingsAveraged;
	const int16_t tp_result = tpFilter.GetSum()/NumReadingsAveraged;

	int32_t result = (int64_t)(temp_cal_tl * temp_cal_vph * tc_result - (int64_t)temp_cal_vpl * temp_cal_th * tc_result
	                   - (int64_t)temp_cal_tl * temp_cal_vch * tp_result
	                   + (int64_t)temp_cal_th * temp_cal_vcl * tp_result);
	const int32_t divisor = ((int32_t)temp_cal_vcl * tp_result - (int32_t)temp_cal_vch * tp_result
	           	   	   - (int32_t)temp_cal_vpl * tc_result + (int32_t)temp_cal_vph * tc_result);
	result = (divisor == 0) ? 0 : result/divisor;
	const float temperature = (float)result/16;

	uint32_t conversionsStarted, conversionsCompleted;
	AnalogIn::GetDebugInfo(conversionsStarted, conversionsCompleted);
	String<200> str;
	str.printf("{\"message\":\"Conv %u %u, %.1fV, %.1fdegC, ptat %d, ctat %d\"}\n",
		(unsigned int)conversionsStarted, (unsigned int)conversionsCompleted, (double)volts, (double)temperature, tp_result, tc_result);
	io_write(io, (const unsigned char *)str.c_str(), str.strlen());
}

// End
