/*
 * AnalogIn.h
 *
 *  Created on: 6 Sep 2018
 *      Author: David
 */

#ifndef SRC_HAL_ANALOGIN_H_
#define SRC_HAL_ANALOGIN_H_

#include "RepRapFirmware.h"

typedef void (*AnalogInCallbackFunction)(CallbackParameter p, uint16_t reading);

namespace AnalogIn
{
	// Initialise the analog input subsystem. Call this just once.
	void Init();

	// Enable analog input on a pin.
	// Readings will be taken and about every 'ticksPerCall' milliseconds the callback function will be called with the specified parameter and ADC reading.
	// Set ticksPerCall to 0 to get a callback on every reading.
	// 'adcnum' specifies which ADC to use, for those pins that can be connected to more than one ADC.
	bool EnableChannel(Pin pin, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall = 1, int adcnum = -1);

	// Disable a previously-enabled channel
	bool DisableChannel(Pin pin);

	// Enable an on-chip MCU temperature sensor
	bool EnableTemperatureSensor(unsigned int sensorNumber, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall = 1, int adcnum = -1);

	// Get the number of conversions that were started
	void GetDebugInfo(uint32_t &convsStarted, uint32_t &convsCompleted);
}

#endif /* SRC_HAL_ANALOGIN_H_ */
