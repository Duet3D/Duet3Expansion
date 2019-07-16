/****************************************************************************************************

RepRapFirmware - Heat

This is all the code to deal with heat and temperature.

-----------------------------------------------------------------------------------------------------

Version 0.1

18 November 2012

Adrian Bowyer
RepRap Professional Ltd
http://reprappro.com

Licence: GPL

****************************************************************************************************/

#include "Heat.h"
#include "HeaterProtection.h"
#include "Platform.h"
#include "Sensors/TemperatureSensor.h"

#if SUPPORT_DHT_SENSOR
# include "Sensors/DhtSensor.h"
#endif

#include "Tasks.h"

constexpr uint32_t HeaterTaskStackWords = 400;			// task stack size in dwords, must be large enough for auto tuning
static Task<HeaterTaskStackWords> heaterTask;

extern "C" [[noreturn]] void HeaterTask(void * pvParameters)
{
	Heat::Task();
}

namespace Heat
{
	// Private members
	TemperatureSensor *GetSensor(unsigned int heater);			// Get a pointer to the temperature sensor entry

	static HeaterProtection *heaterProtections[MaxHeaters + MaxExtraHeaterProtections];	// Heater protection instances to guarantee legal heater temperature ranges

	static PID* pids[MaxHeaters];								// A PID controller for each heater

	static TemperatureSensor *sensorsRoot = nullptr;			// The sensor list

	static uint32_t lastWakeTime;

	static float extrusionMinTemp;								// Minimum temperature to allow regular extrusion
	static float retractionMinTemp;								// Minimum temperature to allow regular retraction
	static bool coldExtrude;									// Is cold extrusion allowed?
	static int8_t heaterBeingTuned;								// which PID is currently being tuned
	static int8_t lastHeaterTuned;								// which PID we last finished tuning
}

// Get the process model for the specified heater
const FopDt& Heat::GetHeaterModel(size_t heater)
{
	return pids[heater]->GetModel();
}

// Set the heater process model
bool Heat::SetHeaterModel(size_t heater, float gain, float tc, float td, float maxPwm, float voltage, bool usePid, bool inverted)
{
	return pids[heater]->SetModel(gain, tc, td, maxPwm, voltage, usePid, inverted);
}

// Is the heater enabled?
bool Heat::IsHeaterEnabled(size_t heater)
{
	return pids[heater]->IsHeaterEnabled();
}

void Heat::GetFaultDetectionParameters(size_t heater, float& maxTempExcursion, float& maxFaultTime)
{
	pids[heater]->GetFaultDetectionParameters(maxTempExcursion, maxFaultTime);
}

void Heat::SetFaultDetectionParameters(size_t heater, float maxTempExcursion, float maxFaultTime)
{
	pids[heater]->SetFaultDetectionParameters(maxTempExcursion, maxFaultTime);
}

// Reset all heater models to defaults. Called when running M502.
void Heat::ResetHeaterModels()
{
	for (size_t heater : ARRAY_INDICES(pids))
	{
		if (pids[heater]->IsHeaterEnabled())
		{
			pids[heater]->SetModel(DefaultHotEndHeaterGain, DefaultHotEndHeaterTimeConstant, DefaultHotEndHeaterDeadTime, 1.0, 0.0, true, false);
		}
	}
}

void Heat::Init()
{
	coldExtrude = false;
	heaterBeingTuned = -1;
	lastHeaterTuned = -1;

	for (size_t index : ARRAY_INDICES(heaterProtections))
	{
		heaterProtections[index] = new HeaterProtection(index);
	}

	for (size_t heater : ARRAY_INDICES(pids))
	{
		pids[heater] = new PID(heater);
		pids[heater]->Init(DefaultHotEndHeaterGain, DefaultHotEndHeaterTimeConstant, DefaultHotEndHeaterDeadTime, true, false);
	}

	// Initialise the heater protection items first
	for (size_t index : ARRAY_INDICES(heaterProtections))
	{
		HeaterProtection * const prot = heaterProtections[index];

		const float tempLimit = DefaultExtruderTemperatureLimit;
		prot->Init(tempLimit);

		if (index < MaxHeaters)
		{
			pids[index]->SetHeaterProtection(prot);
		}
	}

	// Set up the temperature (and other) sensors
	sensorsRoot = nullptr;

#if SUPPORT_DHT_SENSOR
	// Initialise static fields of the DHT sensor
	DhtSensorHardwareInterface::InitStatic();
#endif

	extrusionMinTemp = HOT_ENOUGH_TO_EXTRUDE;
	retractionMinTemp = HOT_ENOUGH_TO_RETRACT;
	coldExtrude = false;

	heaterTask.Create(HeaterTask, "HEAT", nullptr, TaskPriority::HeatPriority);
}

void Heat::Exit()
{
	for (PID *pid : pids)
	{
		pid->SwitchOff();
	}

	heaterTask.Suspend();
}

[[noreturn]] void Heat::Task()
{
	lastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		for (PID *& p : pids)
		{
			p->Spin();
		}

		// See if we have finished tuning a PID
		if (heaterBeingTuned != -1 && !pids[heaterBeingTuned]->IsTuning())
		{
			lastHeaterTuned = heaterBeingTuned;
			heaterBeingTuned = -1;
		}

		Platform::KickHeatTaskWatchdog();

		// Delay until it is time again
		vTaskDelayUntil(&lastWakeTime, HeatSampleIntervalMillis);
	}
}

Heat::HeaterStatus Heat::GetStatus(int8_t heater)
{
	if (heater < 0 || heater >= (int)MaxHeaters)
	{
		return HS_off;
	}

	return (pids[heater]->FaultOccurred()) ? HS_fault
			: (pids[heater]->SwitchedOff()) ? HS_off
				: (pids[heater]->IsTuning()) ? HS_tuning
					: (pids[heater]->Active()) ? HS_active
						: HS_standby;
}

void Heat::SetActiveTemperature(int8_t heater, float t)
{
	if (heater >= 0 && heater < (int)MaxHeaters)
	{
		pids[heater]->SetActiveTemperature(t);
	}
}

float Heat::GetActiveTemperature(int8_t heater)
{
	return (heater >= 0 && heater < (int)MaxHeaters) ? pids[heater]->GetActiveTemperature() : ABS_ZERO;
}

void Heat::SetStandbyTemperature(int8_t heater, float t)
{
	if (heater >= 0 && heater < (int)MaxHeaters)
	{
		pids[heater]->SetStandbyTemperature(t);
	}
}

float Heat::GetStandbyTemperature(int8_t heater)
{
	return (heater >= 0 && heater < (int)MaxHeaters) ? pids[heater]->GetStandbyTemperature() : ABS_ZERO;
}

float Heat::GetHighestTemperatureLimit(int8_t heater)
{
	float limit = BadErrorTemperature;
	if (heater >= 0 && heater < (int)MaxHeaters)
	{
		for (const HeaterProtection *prot : heaterProtections)
		{
			if (prot->GetSupervisedHeater() == heater && prot->GetTrigger() == HeaterProtectionTrigger::TemperatureExceeded)
			{
				const float t = prot->GetTemperatureLimit();
				if (limit == BadErrorTemperature || t > limit)
				{
					limit = t;
				}
			}
		}
	}
	return limit;
}

float Heat::GetLowestTemperatureLimit(int8_t heater)
{
	float limit = ABS_ZERO;
	if (heater >= 0 && heater < (int)MaxHeaters)
	{
		for (const HeaterProtection *prot : heaterProtections)
		{
			if (prot->GetSupervisedHeater() == heater && prot->GetTrigger() == HeaterProtectionTrigger::TemperatureTooLow)
			{
				const float t = prot->GetTemperatureLimit();
				if (limit == ABS_ZERO || t < limit)
				{
					limit = t;
				}
			}
		}
	}
	return limit;
}

// Get the current temperature of a real or virtual heater
// Return ABS_ZERO if the heater doesn't exist. The Z probe class relies on this.
float Heat::GetHeaterTemperature(int8_t heater)
{
	if (heater >= 0 && (unsigned int)heater < MaxHeaters)
	{
		return pids[heater]->GetTemperature();
	}
	return ABS_ZERO;
}

// Get the target temperature of a heater
float Heat::GetTargetTemperature(int8_t heater)
{
	const Heat::HeaterStatus hs = GetStatus(heater);
	return (hs == HS_active) ? GetActiveTemperature(heater)
			: (hs == HS_standby) ? GetStandbyTemperature(heater)
				: 0.0;
}

void Heat::Activate(int8_t heater)
{
	if (heater >= 0 && heater < (int)MaxHeaters)
	{
		pids[heater]->Activate();
	}
}

void Heat::SwitchOff(int8_t heater)
{
	if (heater >= 0 && heater < (int)MaxHeaters)
	{
		pids[heater]->SwitchOff();
	}
}

void Heat::SwitchOffAll()
{
	for (int heater = 0; heater < (int)MaxHeaters; ++heater)
	{
		pids[heater]->SwitchOff();
	}
}

void Heat::ResetFault(int8_t heater)
{
	if (heater >= 0 && heater < (int)MaxHeaters)
	{
		pids[heater]->ResetFault();
	}
}

float Heat::GetAveragePWM(size_t heater)
{
	return pids[heater]->GetAveragePWM();
}

uint32_t Heat::GetLastSampleTime(size_t heater)
{
	return pids[heater]->GetLastSampleTime();
}

// Auto tune a PID
void Heat::StartAutoTune(size_t heater, float temperature, float maxPwm, const StringRef& reply)
{
	if (heaterBeingTuned == -1)
	{
		heaterBeingTuned = (int8_t)heater;
		pids[heater]->StartAutoTune(temperature, maxPwm, reply);
	}
	else
	{
		// Trying to start a new auto tune, but we are already tuning a heater
		reply.printf("Error: cannot start auto tuning heater %u because heater %d is being tuned", heater, heaterBeingTuned);
	}
}

bool Heat::IsTuning(size_t heater)
{
	return pids[heater]->IsTuning();
}

void Heat::GetAutoTuneStatus(const StringRef& reply)
{
	int8_t whichPid = (heaterBeingTuned == -1) ? lastHeaterTuned : heaterBeingTuned;
	if (whichPid != -1)
	{
		pids[whichPid]->GetAutoTuneStatus(reply);
	}
	else
	{
		reply.copy("No heater has been tuned yet");
	}
}

// Get the highest temperature limit of any heater
float Heat::GetHighestTemperatureLimit()
{
	float limit = ABS_ZERO;
	for (HeaterProtection *prot : heaterProtections)
	{
		if (prot->GetHeater() >= 0 && prot->GetTrigger() == HeaterProtectionTrigger::TemperatureExceeded)
		{
			const float t = prot->GetTemperatureLimit();
			if (t > limit)
			{
				limit = t;
			}
		}
	}
	return limit;
}

// Override the model-generated PID parameters
void Heat::SetM301PidParameters(size_t heater, const M301PidParameters& params)
{
	pids[heater]->SetM301PidParameters(params);
}

// Configure the temperature sensor for a channel
GCodeResult Heat::ConfigureSensor(unsigned int sensorNumber, unsigned int mcode, CanMessageM305& msg, const StringRef& reply)
{
	TemperatureSensor * const spp = GetSensor(sensorNumber);
	if (spp == nullptr)
	{
		reply.printf("sensor %u is not configured", sensorNumber);
		return GCodeResult::error;
	}

	return spp->Configure(msg, reply);
}

// Get a pointer to the temperature sensor entry, or nullptr if the heater number is bad
TemperatureSensor *Heat::GetSensor(unsigned int sensorNumber)
{
	for (TemperatureSensor *sensor = sensorsRoot; sensor != nullptr; sensor = sensor->GetNext())
	{
		if (sensor->GetSensorNumber() == sensorNumber)
		{
			return sensor;
		}
	}
	return nullptr;
}

// Return the protection parameters of the given index
HeaterProtection& Heat::AccessHeaterProtection(size_t index)
{
	if (index >= FirstExtraHeaterProtection && index < FirstExtraHeaterProtection + MaxExtraHeaterProtections)
	{
		return *heaterProtections[index + MaxHeaters - FirstExtraHeaterProtection];
	}
	return *heaterProtections[index];
}

// Updates the PIDs and HeaterProtection items after a heater change
void Heat::UpdateHeaterProtection()
{
	// Reassign the first mapped heater protection item of each PID where applicable
	// and rebuild the linked list of heater protection elements per heater
	for (size_t heater : ARRAY_INDICES(pids))
	{
		// Rebuild linked lists
		HeaterProtection *firstProtectionItem = nullptr;
		HeaterProtection *lastElementInList = nullptr;
		for (HeaterProtection *prot : heaterProtections)
		{
			if (prot->GetHeater() == (int)heater)
			{
				if (firstProtectionItem == nullptr)
				{
					firstProtectionItem = prot;
					prot->SetNext(nullptr);
				}
				else if (lastElementInList == nullptr)
				{
					firstProtectionItem->SetNext(prot);
					lastElementInList = prot;
				}
				else
				{
					lastElementInList->SetNext(prot);
					lastElementInList = prot;
				}
			}
		}

		// Update reference to the first item so that we can achieve better performance
		pids[heater]->SetHeaterProtection(firstProtectionItem);
	}
}

// Check if the heater is able to operate returning true if everything is OK
bool Heat::CheckHeater(size_t heater)
{
	return !pids[heater]->FaultOccurred() && pids[heater]->CheckProtection();
}

// Get the temperature of a real or virtual heater
float Heat::GetTemperature(size_t heater, TemperatureError& err)
{
	TemperatureSensor * const spp = GetSensor(heater);
	if (spp == nullptr)
	{
		err = TemperatureError::unknownSensor;
		return BadErrorTemperature;
	}

	float t;
	err = spp->GetTemperature(t);
	if (err != TemperatureError::success)
	{
		t = BadErrorTemperature;
	}
	return t;
}

// Suspend the heaters to conserve power or while doing Z probing
void Heat::SuspendHeaters(bool sus)
{
	for (PID *p : pids)
	{
		p->Suspend(sus);
	}
}

// End
