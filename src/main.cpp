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
	RepRap::Spin();
}

// End
