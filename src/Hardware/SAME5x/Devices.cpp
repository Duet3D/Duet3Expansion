/*
 * Devices.cpp
 *
 *  Created on: 28 Jul 2020
 *      Author: David
 */

#include <Hardware/Devices.h>

#if SAME5x

#include <AnalogIn.h>
#include <AnalogOut.h>

// Analog input support
constexpr size_t AnalogInTaskStackWords = 300;
static Task<AnalogInTaskStackWords> analogInTask;

Uart uart0(3, 3, 512, 512);

extern "C" void SERCOM3_0_Handler()
{
	uart0.Interrupt0();
}

extern "C" void SERCOM3_2_Handler()
{
	uart0.Interrupt2();
}

extern "C" void SERCOM3_3_Handler()
{
	uart0.Interrupt3();
}

void DeviceInit() noexcept
{
	gpio_set_pin_function(PortBPin(20), PINMUX_PB20C_SERCOM3_PAD0);		// TxD
# if 0	// we don't use the receiver, but if we did we would need to do this:
	gpio_set_pin_function(PortBPin(21), PINMUX_PB21C_SERCOM3_PAD1);		// RxD
# endif

	AnalogIn::Init(DmacChanAdc0Tx, DmacPrioAdcTx, DmacPrioAdcRx);
	AnalogOut::Init();
	analogInTask.Create(AnalogIn::TaskLoop, "AIN", nullptr, TaskPriority::AinPriority);
}

#endif

// End
