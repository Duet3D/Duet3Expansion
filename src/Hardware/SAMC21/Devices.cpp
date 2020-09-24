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
	SetPinFunction(PortBPin(2), GpioPinFunction::D);		// TxD
# else
	SetPinFunction(PortAPin(12), GpioPinFunction::D);		// TxD
# endif

	AnalogIn::Init(DmacChanAdc0Rx, DmacPrioAdcRx);
	AnalogOut::Init();
	analogInTask.Create(AnalogIn::TaskLoop, "AIN", nullptr, TaskPriority::AinPriority);
}

#endif

// End
