/*
 * Devices.cpp
 *
 *  Created on: 28 Jul 2020
 *      Author: David
 */

#include <Hardware/Devices.h>

#include <AnalogIn.h>
#include <AnalogOut.h>
#include <TaskPriorities.h>
#include <RTOSIface/RTOSIface.h>


// Analog input support
constexpr size_t AnalogInTaskStackWords = 300;
static Task<AnalogInTaskStackWords> analogInTask;

//TODO add any UART support needed

void DeviceInit() noexcept
{
#if defined(EXP1HCLv0_3) || defined(EXP1HCLv1_0)
	pinMode(ClockGenPin, OUTPUT_LOW);			// default the TMC clock to its internal clock until we program the clock generator
#endif
	AnalogIn::Init(DmacChanAdcRx, DmacPrioAdcRx);
	AnalogOut::Init();
	analogInTask.Create(AnalogIn::TaskLoop, "AIN", nullptr, TaskPriority::AinPriority);
}

// End
