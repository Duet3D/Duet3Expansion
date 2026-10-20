/*
 * Devices.cpp
 *
 *  Created on: 28 Jul 2020
 *      Author: David
 */

#include <RepRapFirmware.h>
#include <Hardware/Devices.h>

#if STM32H5

#include <AnalogIn.h>
#include <AnalogOut.h>
#include <Platform/TaskPriorities.h>
#include <Platform/Platform.h>
#include <RTOSIface/RTOSIface.h>

// Analog input support
constexpr size_t AnalogInTaskStackWords = 300;
static Task<AnalogInTaskStackWords> analogInTask;

void DeviceInit() noexcept
{
#if defined(NODETRIX)
	SetPinFunction(TmcClockPin, TmcClockPinPeriphMode);			// enable the TMC clock output
#endif
	AnalogIn::Init(NvicPriorityAdc);
	AnalogOut::Init();
	analogInTask.Create(AnalogIn::TaskLoop, "AIN", nullptr, TaskPriority::AinPriority);
}

#endif

// End
