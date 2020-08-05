/*
 * Devices.cpp
 *
 *  Created on: 28 Jul 2020
 *      Author: David
 */

#include <Hardware/Devices.h>

#if SAMC21

#include <AnalogIn.h>
#include <AnalogOut.h>

// Analog input support
constexpr size_t AnalogInTaskStackWords = 300;
static Task<AnalogInTaskStackWords> analogInTask;

Uart uart0(4, 3, 512, 512);

extern "C" void SERCOM4_Handler()
{
	uart0.Interrupt();
}

void DeviceInit() noexcept
{
# ifdef SAMMYC21
	gpio_set_pin_function(PortBPin(2), PINMUX_PB02D_SERCOM5_PAD0);		// TxD
# else
	gpio_set_pin_function(PortAPin(12), PINMUX_PA12D_SERCOM4_PAD0);		// TxD
# endif

	AnalogIn::Init(DmacChanAdc0Rx, DmacPrioAdcRx);
	AnalogOut::Init();
	analogInTask.Create(AnalogIn::TaskLoop, "AIN", nullptr, TaskPriority::AinPriority);
}

#endif

// End
