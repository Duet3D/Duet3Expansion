#include <atmel_start.h>
#include <Config/peripheral_clk_config.h>

#include "RepRapFirmware.h"
#include "Tasks.h"
#include <Hardware/DmacManager.h>
#include <Hardware/AnalogIn.h>
#include "AdcAveragingFilter.h"
#include <Movement/Move.h>
#include <Movement/StepperDrivers/TMC51xx.h>

#define USE_RTOS	1

extern "C" void __cxa_pure_virtual() { while (1); }

int main(void)
{
	atmel_start_init();								// Initialize MCU, drivers and middleware

	SystemCoreClock = CONF_CPU_FREQUENCY;			// FreeRTOS needs this to be set correctly because it uses it to set the systick reload value
#if SAME5x
	SystemPeripheralClock = CONF_CPU_FREQUENCY/2;
#elif SAMC21
	SystemPeripheralClock = CONF_CPU_FREQUENCY;
#else
# error Unknown processor
#endif

	DmacManager::Init();

	AppMain();
}

void Init()
{
	RepRap::Init();
}

void Spin()
{
	RepRap::Spin();
}

// End
