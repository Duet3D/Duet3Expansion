#include <atmel_start.h>
#include <Config/peripheral_clk_config.h>

#include "RepRapFirmware.h"
#include "Tasks.h"
#include <HAL/DmacManager.h>
#include <HAL/AnalogIn.h>
#include "AdcAveragingFilter.h"
#include <Movement/Move.h>
#include <Movement/StepperDrivers/TMC51xx.h>

#define USE_RTOS	1

extern "C" void __cxa_pure_virtual() { while (1); }

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	SystemCoreClock = CONF_CPU_FREQUENCY;			// FreeRTOS needs this to be set correctly because it uses it to set the systick reload value
	DmacInit();

	AppMain();
}

void Init()
{
	RepRap::Init();
}

void Spin()
{
#if 1
	RepRap::Spin();
#else
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
	str.printf("{\"message\":\"Conv %u %u, %.1fV, %.1fdegC"
//					", ptat %d, ctat %d"
					", stat %08" PRIx32 " %08" PRIx32 " %08" PRIx32
					"\"}\n",
					(unsigned int)conversionsStarted, (unsigned int)conversionsCompleted, (double)volts, (double)temperature
//					, tp_result, tc_result
					, SmartDrivers::GetLiveStatus(0), SmartDrivers::GetLiveStatus(1), SmartDrivers::GetLiveStatus(1)
				);
	io_write(io, (const unsigned char *)str.c_str(), str.strlen());

	SmartDrivers::Spin(volts >= 10.0);
#endif
}

// End
