#ifndef TEMPERATURESENSOR_H
#define TEMPERATURESENSOR_H

#include "RepRapFirmware.h"
#include "Heating/TemperatureError.h"		// for result codes
#include "GCodes/GCodeResult.h"

class CanMessageM305;

class TemperatureSensor
{
public:
	TemperatureSensor(unsigned int chan);

	// Try to get a temperature reading
	TemperatureError GetTemperature(float& t);

	// Configure the sensor from M305 parameters.
	// If we find any parameters, process them and return true. If an error occurs while processing them, set 'error' to true and write an error message to 'reply.
	// if we find no relevant parameters, report the current parameters to 'reply' and return 'false'.
	virtual GCodeResult Configure(unsigned int heater, const CanMessageM305& msg, const StringRef& reply);

	// Initialise or re-initialise the temperature sensor
	virtual void Init() = 0;

	// Return the channel number
	unsigned int GetSensorChannel() const { return sensorChannel; }

	// Virtual destructor
	virtual ~TemperatureSensor();

	// Factory method
	static TemperatureSensor *Create(unsigned int channel);

protected:
	// Try to get a temperature reading
	virtual TemperatureError TryGetTemperature(float& t) = 0;

	static TemperatureError GetPT100Temperature(float& t, uint16_t ohmsx100);		// shared function used by two derived classes

private:
	const unsigned int sensorChannel;
	TemperatureError lastError;
};

#endif // TEMPERATURESENSOR_H
