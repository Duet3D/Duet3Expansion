/*
 * Heat.h
 *
 *  Created on: 30 Jun 2019
 *      Author: David
 */

#ifndef SRC_HEATING_HEAT_H_
#define SRC_HEATING_HEAT_H_

/**
 * The master class that controls all the heaters controlled by the expansion board
 */

#include "RepRapFirmware.h"
#include <Heating/LocalHeater.h>
#include "MessageType.h"
#include "GCodes/GCodeResult.h"
#include "CanMessageFormats.h"

class TemperatureSensor;
class HeaterProtection;
class FopDt;

namespace Heat
{
	// Methods that don't relate to a particular heater
	void Task();
	void Init();												// Set everything up
	void Exit();												// Shut everything down
	void ResetHeaterModels();									// Reset all active heater models to defaults

#if 0
	GCodeResult ProcessM307(const CanMessageGeneric& msg, const StringRef& reply);
#endif
	GCodeResult ProcessM308(const CanMessageGeneric& msg, const StringRef& reply);
	GCodeResult TuneHeater(const CanMessageGeneric& msg, const StringRef& reply);
	GCodeResult SetPidParameters(const CanMessageGeneric& msg, const StringRef& reply);

	void SwitchOffAll();										// Turn all heaters off
	void ResetFault(int heater);								// Reset a heater fault - only call this if you know what you are doing

	// Methods that relate to a particular heater
	void SetActiveTemperature(int heater, float t);
	float GetActiveTemperature(int heater);
	void SetStandbyTemperature(int heater, float t);
	float GetStandbyTemperature(int heater);
	float GetHighestTemperatureLimit(int heater);
	float GetLowestTemperatureLimit(int heater);
	void Activate(int heater);									// Turn on a heater
	float GetSensorTemperature(int sensorNum, TemperatureError& err); // Result is in degrees Celsius
	float GetTargetTemperature(int heater);						// Get the target temperature
	HeaterStatus GetStatus(int heater);							// Get the off/standby/active status
	void SwitchOff(int heater);									// Turn off a specific heater
																// Is a specific heater at temperature within tolerance?
	float GetAveragePWM(size_t heater)							// Return the running average PWM to the heater as a fraction in [0, 1].
	pre(heater < NumTotalHeaters);

	bool IsHeaterEnabled(size_t heater)							// Is this heater enabled?
	pre(heater < NumTotalHeaters);

	float GetHighestTemperatureLimit();							// Get the highest temperature limit of any heater

	int GetHeaterChannel(size_t heater);						// Return the channel used by a particular heater, or -1 if not configured
	bool SetHeaterChannel(size_t heater, int channel);			// Set the channel used by a heater, returning true if bad heater or channel number
	const char *GetHeaterSensorName(size_t heater);				// Get the name of the sensor for a heater, or nullptr if it hasn't been named

	float GetTemperature(size_t sensor, TemperatureError& err); // Result is in degrees Celsius

	HeaterProtection& AccessHeaterProtection(size_t index);		// Return the protection parameters of the given index
	void UpdateHeaterProtection();								// Updates the PIDs and HeaterProtection items when a heater is remapped

	void SuspendHeaters(bool sus);								// Suspend the heaters to conserve power

	TemperatureSensor *GetSensor(int sn);						// Get a pointer to the temperature sensor entry
	TemperatureSensor *GetSensorAtOrAbove(unsigned int sn);		// Get a pointer to the first temperature sensor with the specified or higher number
	void RemoveSensor(unsigned int sensorNum);
	void InsertSensor(TemperatureSensor *sensor);

	inline bool IsBedOrChamberHeater(int heater) { return false; }
};

#endif /* SRC_HEATING_HEAT_H_ */
