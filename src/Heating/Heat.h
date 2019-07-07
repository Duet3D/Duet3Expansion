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
#include "Pid.h"
#include "MessageType.h"
#include "GCodes/GCodeResult.h"
#include "CanMessageFormats.h"

class TemperatureSensor;
class HeaterProtection;
class FopDt;

namespace Heat
{
	// Enumeration to describe the status of a heater. Note that the web interface returns the numerical values, so don't change them.
	enum HeaterStatus { HS_off = 0, HS_standby = 1, HS_active = 2, HS_fault = 3, HS_tuning = 4 };

	void Task();
	void Init();												// Set everything up
	void Exit();												// Shut everything down
	void ResetHeaterModels();									// Reset all active heater models to defaults

	void SetActiveTemperature(int8_t heater, float t);
	float GetActiveTemperature(int8_t heater);
	void SetStandbyTemperature(int8_t heater, float t);
	float GetStandbyTemperature(int8_t heater);
	float GetHighestTemperatureLimit(int8_t heater);
	float GetLowestTemperatureLimit(int8_t heater);
	void Activate(int8_t heater);								// Turn on a heater
	float GetTemperature(int8_t heater);						// Get the temperature of a heater
	float GetTargetTemperature(int8_t heater);					// Get the target temperature
	HeaterStatus GetStatus(int8_t heater);						// Get the off/standby/active status
	void SwitchOff(int8_t heater);								// Turn off a specific heater
	void SwitchOffAll();										// Turn all heaters off
	void ResetFault(int8_t heater);								// Reset a heater fault - only call this if you know what you are doing
																// Is a specific heater at temperature within tolerance?
	float GetAveragePWM(size_t heater)							// Return the running average PWM to the heater as a fraction in [0, 1].
	pre(heater < NumTotalHeaters);

	uint32_t GetLastSampleTime(size_t heater)
	pre(heater < NumTotalHeaters);

	void StartAutoTune(size_t heater, float temperature, float maxPwm, const StringRef& reply) // Auto tune a PID
	pre(heater < NumTotalHeaters);

	bool IsTuning(size_t heater)								// Return true if the specified heater is auto tuning
	pre(heater < NumTotalHeaters);

	void GetAutoTuneStatus(const StringRef& reply);				// Get the status of the current or last auto tune

	const FopDt& GetHeaterModel(size_t heater)					// Get the process model for the specified heater
	pre(heater < NumTotalHeaters);

	bool SetHeaterModel(size_t heater, float gain, float tc, float td, float maxPwm, float voltage, bool usePid, bool inverted) // Set the heater process model
	pre(heater < NumTotalHeaters);

	void GetFaultDetectionParameters(size_t heater, float& maxTempExcursion, float& maxFaultTime)
	pre(heater < NumTotalHeaters);

	void SetFaultDetectionParameters(size_t heater, float maxTempExcursion, float maxFaultTime)
	pre(heater < NumTotalHeaters);

	bool IsHeaterEnabled(size_t heater)							// Is this heater enabled?
	pre(heater < NumTotalHeaters);

	float GetHighestTemperatureLimit();							// Get the highest temperature limit of any heater

	void SetM301PidParameters(size_t heater, const M301PidParameters& params)
	pre(heater < NumTotalHeaters);

	int GetHeaterChannel(size_t heater);						// Return the channel used by a particular heater, or -1 if not configured
	bool SetHeaterChannel(size_t heater, int channel);			// Set the channel used by a heater, returning true if bad heater or channel number
	GCodeResult ConfigureHeaterSensor(size_t heater, unsigned int mcode, CanMessageM305& msg, const StringRef& reply);	// Configure the temperature sensor for a channel
	const char *GetHeaterName(size_t heater);					// Get the name of a heater, or nullptr if it hasn't been named

	HeaterProtection& AccessHeaterProtection(size_t index);		// Return the protection parameters of the given index
	void UpdateHeaterProtection();								// Updates the PIDs and HeaterProtection items when a heater is remapped

	bool CheckHeater(size_t heater)								// Check if the heater is able to operate
	pre(heater < NumTotalHeaters);

	float GetTemperature(size_t heater, TemperatureError& err); // Result is in degrees Celsius

	void SuspendHeaters(bool sus);								// Suspend the heaters to conserve power
};

#endif /* SRC_HEATING_HEAT_H_ */
