#ifndef TEMPERATURESENSOR_H
#define TEMPERATURESENSOR_H

#include <RepRapFirmware.h>
#include <TemperatureError.h>		// for result codes
#include <Hardware/IoPorts.h>
#include <CanId.h>

class CanMessageGenericParser;
class CanSensorReport;

class TemperatureSensor
{
public:
	TemperatureSensor(unsigned int sensorNum, const char *type);

	// Virtual destructor
	virtual ~TemperatureSensor();

	// Configure the sensor from M308 parameters.
	// If we find any parameters, process them and return true. If an error occurs while processing them, return error and write an error message to 'reply.
	// If we find no relevant parameters, report the current parameters to 'reply' and return ok.
	virtual GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply);

	// Try to get a temperature reading
	virtual void Poll() = 0;

	// Try to get a temperature reading
	virtual TemperatureError GetLatestTemperature(float& t, uint8_t outputNumber = 0) noexcept;

	// How many additional outputs does this sensor have
	virtual const uint8_t GetNumAdditionalOutputs() const noexcept { return 0; }

	// How long after a reading before we consider the reading to be unreliable - this has to be increased for DHT sensors
	virtual uint32_t GetTemperatureReadingTimeout() const noexcept { return DefaultTemperatureReadingTimeout; }

	// Get the expansion board address. Overridden for remote sensors.
	virtual CanAddress GetBoardAddress() const;

	// Update the temperature, if it is a remote sensor. Overridden in class RemoteSensor.
	virtual void UpdateRemoteTemperature(CanAddress src, const CanSensorReport& report) noexcept;

	// Get the most recent reading without checking for timeout
	float GetStoredReading() const noexcept { return lastTemperature; }

	// Return the sensor type
	const char *GetSensorType() const { return sensorType; }

	// Return the sensor number
	unsigned int GetSensorNumber() const { return sensorNumber; }

	// Return the code for the most recent error
	TemperatureError GetLastError() const { return lastRealError; }

	// Copy the basic details to the reply buffer
	void CopyBasicDetails(const StringRef& reply) const noexcept;

	// Get/set the next sensor in the linked list
	TemperatureSensor *GetNext() const { return next; }
	void SetNext(TemperatureSensor *n) { next = n; }

	// Get the time of the last reading
	uint32_t GetLastReadingTime() const noexcept { return whenLastRead; }

	// Factory method
	static TemperatureSensor *Create(unsigned int sensorNum, const char *typeName, const StringRef& reply);

protected:
	void SetResult(float t, TemperatureError rslt);
	void SetResult(TemperatureError rslt);

	static TemperatureError GetPT100Temperature(float& t, uint16_t ohmsx100);		// shared function used by two derived classes

private:
	static constexpr uint32_t DefaultTemperatureReadingTimeout = 2000;

	TemperatureSensor *next;
	unsigned int sensorNumber;					// the number of this sensor
	const char * const sensorType;
	volatile float lastTemperature;
	volatile uint32_t whenLastRead;
	volatile TemperatureError lastResult, lastRealError;
};

#endif // TEMPERATURESENSOR_H
