/*
 * AnalogIn.h
 *
 *  Created on: 6 Sep 2018
 *      Author: David
 */

#ifndef SRC_HARDWARE_ANALOGIN_H_
#define SRC_HARDWARE_ANALOGIN_H_

#include "RepRapFirmware.h"

typedef void (*AnalogInCallbackFunction)(CallbackParameter p, uint16_t reading);

namespace AnalogIn
{
	// Initialise the analog input subsystem. Call this just once.
	void Init();

	// Enable analog input on a pin.
	// Readings will be taken and about every 'ticksPerCall' milliseconds the callback function will be called with the specified parameter and ADC reading.
	// Set ticksPerCall to 0 to get a callback on every reading.
	// Warning! there is nothing to stpo myou enabling a channel twice, in which case it will be read twice in the sequence.
	bool EnableChannel(Pin pin, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall = 1);

	// Return whether or not the channel is enabled
	bool IsChannelEnabled(Pin pin);

	// Disable a previously-enabled channel
	//bool DisableChannel(Pin pin);

	// Enable an on-chip MCU temperature sensor
	bool EnableTemperatureSensor(unsigned int sensorNumber, AnalogInCallbackFunction fn, CallbackParameter param, uint32_t ticksPerCall, unsigned int adcnum);

	// Get the latest result form a channel. the channel must havew been enabled first.
	uint16_t ReadChannel(AdcInput adcin);

	// Get the number of conversions that were started
	void GetDebugInfo(uint32_t &convsStarted, uint32_t &convsCompleted);
}

#endif /* SRC_HARDWARE_ANALOGIN_H_ */
