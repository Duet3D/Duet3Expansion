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
#include <CanMessageBuffer.h>
#include "CAN/CanInterface.h"

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

	static TemperatureSensor *sensorsRoot = nullptr;			// The sensor list. Only the Heat task is allowed to modify the linkeage.
	static TemperatureSensor *newSensors = nullptr;				// New sensors waiting to be linked into the main list
	HeaterProtection *heaterProtections[MaxHeaters + MaxExtraHeaterProtections];	// Heater protection instances to guarantee legal heater temperature ranges

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
	uint32_t lastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		// Walk the sensor list and poll all sensors except those flagged for deletion. Don'y mess with the list during this pass because polling may need to acquire mutexes.
		TemperatureSensor *currentSensor = sensorsRoot;
		bool sawDeletedSensor = false;
		while (currentSensor != nullptr)
		{
			if (currentSensor->GetSensorNumber() < 0)
			{
				sawDeletedSensor = true;
			}
			else
			{
				currentSensor->Poll();
			}
			currentSensor = currentSensor->GetNext();
		}

		// If we saw any sensors flagged for deletion, delete them, locking out other tasks while we do this
		if (sawDeletedSensor)
		{
			TaskCriticalSectionLocker lock;

			currentSensor = sensorsRoot;
			TemperatureSensor *lastSensor = nullptr;
			while (currentSensor != nullptr)
			{
				if (currentSensor->GetSensorNumber() < 0)
				{
					TemperatureSensor *sensorToDelete = currentSensor;
					currentSensor = currentSensor->GetNext();
					if (lastSensor == nullptr)
					{
						sensorsRoot = currentSensor;
					}
					else
					{
						lastSensor->SetNext(currentSensor);
					}
					delete sensorToDelete;
				}
				else
				{
					lastSensor = currentSensor;
					currentSensor = currentSensor->GetNext();
				}
			}
		}

		// Insert any new sensors. We don't poll them yet because they may only just have finished being initialised so they may not accept another transaction yet.
		for (;;)
		{
			TaskCriticalSectionLocker lock;
			TemperatureSensor *currentNewSensor = newSensors;
			if (currentNewSensor == nullptr)
			{
				break;
			}
			newSensors = currentNewSensor->GetNext();
			TemperatureSensor *prev = nullptr;
			TemperatureSensor *ts = sensorsRoot;
			for (;;)
			{
				if (ts == nullptr || ts->GetSensorNumber() > currentNewSensor->GetSensorNumber())
				{
					currentNewSensor->SetNext(ts);
					if (prev == nullptr)
					{
						sensorsRoot = currentNewSensor;
					}
					else
					{
						prev->SetNext(currentNewSensor);
					}
					break;
				}
				prev = ts;
				ts = ts->GetNext();
			}
		}

		// Spin the heaters
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

		// Broadcast our sensor temperatures
		CanMessageBuffer * buf = CanMessageBuffer::Allocate();
		if (buf != nullptr)
		{
			CanMessageSensorTemperatures * const msg = buf->SetupBroadcastMessage<CanMessageSensorTemperatures>(CanInterface::GetCanAddress());
			msg->whichSensors = 0;
			unsigned int sensorsFound = 0;
			unsigned int currentSensorNumber = 0;
			for (;;)
			{
				TemperatureSensor * const sensor = GetSensorAtOrAbove(currentSensorNumber);
				if (sensor == nullptr)
				{
					break;
				}
				const int sn = sensor->GetSensorNumber();
				if (sn >= 0)
				{
					msg->whichSensors |= (uint64_t)1u << (unsigned int)sn;
					float temperature;
					msg->temperatureReports[sensorsFound].errorCode = (uint8_t)sensor->GetLatestTemperature(temperature);
					msg->temperatureReports[sensorsFound].temperature = temperature;
					++sensorsFound;
					currentSensorNumber = (unsigned int)sn + 1u;
				}
			}
			if (sensorsFound == 0)
			{
				// Don't send an empty report
				CanMessageBuffer::Free(buf);
			}
			else
			{
				buf->dataLength = msg->GetActualDataLength(sensorsFound);
				CanInterface::Send(buf);
			}
		}

		// Delay until it is time again
		vTaskDelayUntil(&lastWakeTime, HeatSampleIntervalMillis);
	}
}

GCodeResult Heat::ConfigureHeater(const CanMessageGeneric& msg, const StringRef& reply)
{
	CanMessageGenericParser parser(msg, M950HeaterParams);
	uint16_t heater;
	if (!parser.GetUintParam('H', heater))
	{
		return GCodeResult::remoteInternalError;
	}

	if (heater < MaxHeaters)
	{
		Heater *oldHeater = heaters[heater];

		if (oldHeater == nullptr)
		{
			heaters[heater] = new LocalHeater(heater);
		}
		return heaters[heater]->ConfigurePortAndSensor(parser, reply);
	}

	reply.copy("Heater number out of range");
	return GCodeResult::error;
}

GCodeResult Heat::ProcessM307(const CanMessageUpdateHeaterModel& msg, const StringRef& reply)
{
	Heater * const h = FindHeater(msg.heater);
	if (h == nullptr)
	{
		reply.printf("Unknown heater %u", msg.heater);
		return GCodeResult::error;
	}

	const GCodeResult rslt = h->SetModel(msg.gain, msg.timeConstant, msg.deadTime, msg.maxPwm, msg.standardVoltage, msg.usePid, msg.inverted, reply);
	if (msg.pidParametersOverridden && (rslt == GCodeResult::ok || rslt == GCodeResult::warning))
	{
		h->SetRawPidParameters(msg.kP, msg.recipTi, msg.tD);
	}
	return rslt;
}

GCodeResult Heat::ProcessM308(const CanMessageGeneric& msg, const StringRef& reply)
{
	CanMessageGenericParser parser(msg, M308Params);
	uint16_t sensorNum;
	if (parser.GetUintParam('S', sensorNum))
	{
		if (sensorNum < MaxSensorsInSystem)
		{
			TemperatureSensor *sensor;
			String<StringLength20> sensorTypeName;
			bool newSensor = parser.GetStringParam('Y', sensorTypeName.GetRef());
			if (newSensor)
			{
				TemperatureSensor * const oldSensor = GetSensor(sensorNum);
				if (oldSensor != nullptr)
				{
					oldSensor->FlagForDeletion();
				}

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
					TaskCriticalSectionLocker lock;
					sensor->SetNext(newSensors);
					newSensors = sensor;
				}
				else
				{
					delete sensor;
				}
			}
			return rslt;
		}
		else
		{
			reply.copy("Sensor number our of range");
			return GCodeResult::error;
		}
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
		err = sensor->GetLatestTemperature(temp);
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
			if (sensor->GetSensorNumber() == sn)
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
		if (sensor->GetSensorNumber() >= (int)sn)
		{
			return sensor;
		}
	}
	return nullptr;
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
	err = spp->GetLatestTemperature(t);
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
