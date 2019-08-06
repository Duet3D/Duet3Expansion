#ifndef TEMPERATURESENSOR_H
#define TEMPERATURESENSOR_H

#include "RepRapFirmware.h"
#include "Heating/TemperatureError.h"		// for result codes
#include "GCodes/GCodeResult.h"
#include <Hardware/IoPorts.h>

class CanMessageGenericParser;

class TemperatureSensor
{
public:
	TemperatureSensor(unsigned int sensorNum, const char *type);

	// Virtual destructor
	virtual ~TemperatureSensor();

	// Try to get a temperature reading
	TemperatureError GetTemperature(float& t);

	// Configure the sensor from M305 parameters.
	// If we find any parameters, process them and return true. If an error occurs while processing them, return error and write an error message to 'reply.
	// If we find no relevant parameters, report the current parameters to 'reply' and return ok.
	virtual GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply);

	// Return the sensor type
	const char *GetSensorType() const { return sensorType; }

	// Return the sensor number
	unsigned int GetSensorNumber() const { return sensorNumber; }

	// Return the code for the most recent error
	TemperatureError GetLastError() const { return lastError; }

	// Copy the basic details to the reply buffer
	void CopyBasicDetails(const StringRef& reply) const;

	// Get/set the next sensor in the linked list
	TemperatureSensor *GetNext() const { return next; }
	void SetNext(TemperatureSensor *n) { next = n; }

	// Factory method
	static TemperatureSensor *Create(unsigned int sensorNum, const char *typeName, const StringRef& reply);

protected:
	// Try to get a temperature reading
	virtual TemperatureError TryGetTemperature(float& t) = 0;

	static TemperatureError GetPT100Temperature(float& t, uint16_t ohmsx100);		// shared function used by two derived classes

private:
	TemperatureSensor *next;
	unsigned int sensorNumber;
	const char * const sensorType;
	TemperatureError lastError;
};

#endif // TEMPERATURESENSOR_H
