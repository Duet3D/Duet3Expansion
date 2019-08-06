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
#include "Heater.h"
#include "HeaterProtection.h"
#include "Platform.h"
#include "Sensors/TemperatureSensor.h"
#include "CanMessageGenericParser.h"

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
	static Heater* heaters[MaxHeaters];							// A PID controller for each heater

	static TemperatureSensor *sensorsRoot = nullptr;			// The sensor list
	HeaterProtection *heaterProtections[MaxHeaters + MaxExtraHeaterProtections];	// Heater protection instances to guarantee legal heater temperature ranges

	static uint32_t lastWakeTime;

	static float extrusionMinTemp;								// Minimum temperature to allow regular extrusion
	static float retractionMinTemp;								// Minimum temperature to allow regular retraction
	static bool coldExtrude;									// Is cold extrusion allowed?
	static int8_t heaterBeingTuned;								// which PID is currently being tuned
	static int8_t lastHeaterTuned;								// which PID we last finished tuning

	static Heater *FindHeater(int heater)
	{
		return (heater < 0 || heater >= (int)MaxHeaters) ? nullptr : heaters[heater];
	}
}

// Is the heater enabled?
bool Heat::IsHeaterEnabled(size_t heater)
{
	Heater * const h = FindHeater(heater);
	return h != nullptr && h->IsHeaterEnabled();
}

// Reset all heater models to defaults. Called when running M502.
void Heat::ResetHeaterModels()
{
	for (Heater *h : heaters)
	{
		if (h != nullptr && h->IsHeaterEnabled())
		{
			String<1> dummy;
			h->SetModel(DefaultHotEndHeaterGain, DefaultHotEndHeaterTimeConstant, DefaultHotEndHeaterDeadTime, 1.0, 0.0, true, false, dummy.GetRef());
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

	for (Heater *& h : heaters)
	{
		h = nullptr;
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
	for (Heater *h : heaters)
	{
		if (h != nullptr)
		{
			h->SwitchOff();
		}
	}

	heaterTask.Suspend();
}

[[noreturn]] void Heat::Task()
{
	lastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		for (Heater *h : heaters)
		{
			if (h != nullptr)
			{
				h->Spin();
			}
		}

		// See if we have finished tuning a PID
		if (heaterBeingTuned != -1 && heaters[heaterBeingTuned]->GetStatus() != HeaterStatus::tuning)
		{
			lastHeaterTuned = heaterBeingTuned;
			heaterBeingTuned = -1;
		}

		Platform::KickHeatTaskWatchdog();

		// Delay until it is time again
		vTaskDelayUntil(&lastWakeTime, HeatSampleIntervalMillis);
	}
}

#if 0
GCodeResult Heat::ProcessM307(const CanMessageGeneric& msg, const StringRef& reply)
{
	CanMessageGenericParser parser(msg, M307Params);
	uint16_t heater;
	if (parser.GetUintParam('H', heater))
	{
		Heater * const h = FindHeater(heater);
		if (h != nullptr)
		{
			const FopDt& model = h->GetModel();
			bool seen = false;
			float gain = model.GetGain(),
				tc = model.GetTimeConstant(),
				td = model.GetDeadTime(),
				maxPwm = model.GetMaxPwm(),
				voltage = model.GetVoltage();
			uint8_t dontUsePid = model.UsePid() ? 0 : 1;
			uint8_t inversionParameter = 0;

			seen = parser.GetFloatParam('A', gain) || seen;
			seen = parser.GetFloatParam('C', tc) || seen;
			seen = parser.GetFloatParam('D', td) || seen;
			seen = parser.GetUintParam('B', dontUsePid) || seen;
			seen = parser.GetFloatParam('S', maxPwm) || seen;
			seen = parser.GetFloatParam('V', voltage) || seen;
			seen = parser.GetUintParam('I', inversionParameter) || seen;

			if (seen)
			{
				const bool inverseTemperatureControl = (inversionParameter == 1 || inversionParameter == 3);
				const GCodeResult rslt = h->SetModel(gain, tc, td, maxPwm, voltage, dontUsePid == 0, inverseTemperatureControl, reply);
				if (rslt != GCodeResult::ok)
				{
					return rslt;
				}
			}
			else if (!model.IsEnabled())
			{
				reply.printf("Heater %u is disabled", heater);
			}
			else
			{
				const char* const mode = (!model.UsePid()) ? "bang-bang"
											: (model.ArePidParametersOverridden()) ? "custom PID"
												: "PID";
				reply.printf("Heater %u model: gain %.1f, time constant %.1f, dead time %.1f, max PWM %.2f, calibration voltage %.1f, mode %s", heater,
							 (double)model.GetGain(), (double)model.GetTimeConstant(), (double)model.GetDeadTime(), (double)model.GetMaxPwm(), (double)model.GetVoltage(), mode);
				if (model.IsInverted())
				{
					reply.cat(", inverted temperature control");
				}
				if (model.UsePid())
				{
					// When reporting the PID parameters, we scale them by 255 for compatibility with older firmware and other firmware
					M301PidParameters params = model.GetM301PidParameters(false);
					reply.catf("\nComputed PID parameters for setpoint change: P%.1f, I%.3f, D%.1f", (double)params.kP, (double)params.kI, (double)params.kD);
					params = model.GetM301PidParameters(true);
					reply.catf("\nComputed PID parameters for load change: P%.1f, I%.3f, D%.1f", (double)params.kP, (double)params.kI, (double)params.kD);
				}
			}
		}

		reply.printf("Heater %u not found", heater);
		return GCodeResult::error;
	}

	return GCodeResult::badOrMissingParameter;
}
#endif

GCodeResult Heat::ProcessM308(const CanMessageGeneric& msg, const StringRef& reply)
{
	CanMessageGenericParser parser(msg, M308Params);
	uint16_t sensorNum;
	if (parser.GetUintParam('S', sensorNum))
	{
		TemperatureSensor *sensor;
		String<StringLength20> sensorTypeName;
		bool newSensor = parser.GetStringParam('Y', sensorTypeName.GetRef());
		if (newSensor)
		{
			RemoveSensor(sensorNum);
			sensor = TemperatureSensor::Create(sensorNum, sensorTypeName.c_str(), reply);
			if (sensor == nullptr)
			{
				return GCodeResult::error;
			}
		}
		else
		{
			sensor = GetSensor(sensorNum);
			if (sensor == nullptr)
			{
				reply.printf("Sensor %u does not exist", sensorNum);
				return GCodeResult::error;
			}

		}
		const GCodeResult rslt = sensor->Configure(parser, reply);
		if (newSensor)
		{
			if (rslt == GCodeResult::ok)
			{
				InsertSensor(sensor);
			}
			else
			{
				delete sensor;
			}
		}
		return rslt;
	}

	reply.copy("Missing sensor number parameter");
	return GCodeResult::error;
}

HeaterStatus Heat::GetStatus(int heater)
{
	Heater * const h = FindHeater(heater);
	return (h == nullptr) ? HeaterStatus::off : heaters[heater]->GetStatus();
}

void Heat::SetActiveTemperature(int heater, float t)
{
	Heater * const h = FindHeater(heater);
	if (h != nullptr)
	{
		h->SetActiveTemperature(t);
	}
}

float Heat::GetActiveTemperature(int heater)
{
	Heater * const h = FindHeater(heater);
	return (h == nullptr) ? ABS_ZERO : h->GetActiveTemperature();
}

void Heat::SetStandbyTemperature(int heater, float t)
{
	Heater * const h = FindHeater(heater);
	if (h != nullptr)
	{
		h->SetStandbyTemperature(t);
	}
}

float Heat::GetStandbyTemperature(int heater)
{
	Heater * const h = FindHeater(heater);
	return (h == nullptr) ? ABS_ZERO : h->GetStandbyTemperature();
}

float Heat::GetHighestTemperatureLimit(int heater)
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

float Heat::GetLowestTemperatureLimit(int heater)
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

// Updates the PIDs and HeaterProtection items after a heater change
void Heat::UpdateHeaterProtection()
{
	// Reassign the first mapped heater protection item of each PID where applicable
	// and rebuild the linked list of heater protection elements per heater
	for (size_t heater : ARRAY_INDICES(heaters))
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
		if (heaters[heater] != nullptr)
		{
			heaters[heater]->SetHeaterProtection(firstProtectionItem);
		}
	}
}

// Get the temperature of a sensor
float Heat::GetSensorTemperature(int sensorNum, TemperatureError& err)
{
	TemperatureSensor * const sensor = GetSensor(sensorNum);
	if (sensor != nullptr)
	{
		float temp;
		err = sensor->GetTemperature(temp);
		return temp;
	}

	err = TemperatureError::unknownSensor;
	return BadErrorTemperature;
}

// Get the target temperature of a heater
float Heat::GetTargetTemperature(int heater)
{
	const HeaterStatus hs = GetStatus(heater);
	return (hs == HeaterStatus::active) ? GetActiveTemperature(heater)
			: (hs == HeaterStatus::standby) ? GetStandbyTemperature(heater)
				: 0.0;
}

void Heat::Activate(int heater)
{
	Heater * const h = FindHeater(heater);
	if (h != nullptr)
	{
		h->Activate();
	}
}

void Heat::SwitchOff(int heater)
{
	Heater * const h = FindHeater(heater);
	if (h != nullptr)
	{
		h->SwitchOff();
	}
}

void Heat::SwitchOffAll()
{
	for (int heater = 0; heater < (int)MaxHeaters; ++heater)
	{
		Heater * const h = heaters[heater];
		if (h != nullptr)
		{
			h->SwitchOff();
		}
	}
}

void Heat::ResetFault(int heater)
{
	Heater * const h = FindHeater(heater);
	if (h != nullptr)
	{
		h->ResetFault();
	}
}

float Heat::GetAveragePWM(size_t heater)
{
	Heater * const h = FindHeater(heater);
	return (h == nullptr) ? 0.0 : h->GetAveragePWM();
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

// Get a pointer to the temperature sensor entry, or nullptr if the heater number is bad
TemperatureSensor *Heat::GetSensor(int sn)
{
	if (sn >= 0)
	{
		TaskCriticalSectionLocker lock;		// make sure the linked list doesn't change while we are searching it
		for (TemperatureSensor *sensor = sensorsRoot; sensor != nullptr; sensor = sensor->GetNext())
		{
			if (sensor->GetSensorNumber() == (unsigned int)sn)
			{
				return sensor;
			}
		}
	}
	return nullptr;
}

// Get a pointer to the first temperature sensor with the specified or higher number
TemperatureSensor *Heat::GetSensorAtOrAbove(unsigned int sn)
{
	TaskCriticalSectionLocker lock;		// make sure the linked list doesn't change while we are searching it

	for (TemperatureSensor *sensor = sensorsRoot; sensor != nullptr; sensor = sensor->GetNext())
	{
		if (sensor->GetSensorNumber() >= sn)
		{
			return sensor;
		}
	}
	return nullptr;
}

// Remove any existing sensor with the specified number
void Heat::RemoveSensor(unsigned int sensorNum)
{
	TaskCriticalSectionLocker lock;		// make sure nothing searches the linked list while we are changing it

	TemperatureSensor *prev = nullptr;
	for (TemperatureSensor *ts = sensorsRoot; ts != nullptr; ts = ts->GetNext())
	{
		const unsigned int sn = ts->GetSensorNumber();
		if (sn == sensorNum)
		{
			if (prev == nullptr)
			{
				sensorsRoot = ts->GetNext();
			}
			else
			{
				prev->SetNext(ts->GetNext());
			}
			delete ts;
			break;
		}
		if (sn > sensorNum)
		{
			break;
		}
		prev = ts;
	}
}

void Heat::InsertSensor(TemperatureSensor *sensor)
{
	TaskCriticalSectionLocker lock;		// make sure nothing searches the linked list while we are changing it

	TemperatureSensor *prev = nullptr;
	TemperatureSensor *ts = sensorsRoot;
	for (;;)
	{
		if (ts == nullptr || ts->GetSensorNumber() > sensor->GetSensorNumber())
		{
			sensor->SetNext(ts);
			if (prev == nullptr)
			{
				sensorsRoot = sensor;
			}
			else
			{
				prev->SetNext(sensor);
			}
			break;
		}
		prev = ts;
		ts = ts->GetNext();
	}
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
	for (Heater *h : heaters)
	{
		if (h != nullptr)
		{
			h->Suspend(sus);
		}
	}
}

// End
