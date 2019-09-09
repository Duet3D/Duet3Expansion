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
	HeaterProtection *heaterProtections[MaxHeaters + MaxExtraHeaterProtections];	// Heater protection instances to guarantee legal heater temperature ranges

	static float extrusionMinTemp;								// Minimum temperature to allow regular extrusion
	static float retractionMinTemp;								// Minimum temperature to allow regular retraction
	static bool coldExtrude;									// Is cold extrusion allowed?
	static int8_t heaterBeingTuned;								// which PID is currently being tuned
	static int8_t lastHeaterTuned;								// which PID we last finished tuning

	static ReadWriteLock heatersLock;
	static ReadWriteLock sensorsLock;

	static ReadLockedPointer<Heater> FindHeater(int heater)
	{
		ReadLocker locker(heatersLock);
		return ReadLockedPointer<Heater>(locker, (heater < 0 || heater >= (int)MaxHeaters) ? nullptr : heaters[heater]);
	}

	// Delete a sensor, if there is one. Must write-lock the sensors lock before calling this.
	static void DeleteSensor(unsigned int sn)
	{
		TemperatureSensor *currentSensor = sensorsRoot;
		TemperatureSensor *lastSensor = nullptr;

		while (currentSensor != nullptr)
		{
			if (currentSensor->GetSensorNumber() == sn)
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
				break;
			}

			lastSensor = currentSensor;
			currentSensor = currentSensor->GetNext();
		}
	}

	// Insert a sensor. Must write-lock the sensors lock before calling this.
	static void InsertSensor(TemperatureSensor *newSensor)
	{
		TemperatureSensor *prev = nullptr;
		TemperatureSensor *ts = sensorsRoot;
		for (;;)
		{
			if (ts == nullptr || ts->GetSensorNumber() > newSensor->GetSensorNumber())
			{
				newSensor->SetNext(ts);
				if (prev == nullptr)
				{
					sensorsRoot = newSensor;
				}
				else
				{
					prev->SetNext(newSensor);
				}
				break;
			}
			prev = ts;
			ts = ts->GetNext();
		}
	}
}

// Is the heater enabled?
bool Heat::IsHeaterEnabled(size_t heater)
{
	const auto h = FindHeater(heater);
	return h.IsNotNull() && h->IsHeaterEnabled();
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
		// Walk the sensor list and poll all sensors
		{
			ReadLocker lock(sensorsLock);
			TemperatureSensor *currentSensor = sensorsRoot;
			while (currentSensor != nullptr)
			{
				currentSensor->Poll();
				currentSensor = currentSensor->GetNext();
			}
		}

		// Spin the heaters
		{
			ReadLocker lock(heatersLock);
			for (Heater *h : heaters)
			{
				if (h != nullptr)
				{
					h->Spin();
				}
			}
		}

		// See if we have finished tuning a PID
		if (heaterBeingTuned != -1)
		{
			const auto h = FindHeater(heaterBeingTuned);
			if (h.IsNull() || !h->IsTuning())
			{
				lastHeaterTuned = heaterBeingTuned;
				heaterBeingTuned = -1;
			}
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
				const auto sensor = FindSensorAtOrAbove(currentSensorNumber);
				if (sensor.IsNull())
				{
					break;
				}
				const unsigned int sn = sensor->GetSensorNumber();
				if (sensor->GetBoardAddress() == CanInterface::GetCanAddress())
				{
					msg->whichSensors |= (uint64_t)1u << sn;
					float temperature;
					msg->temperatureReports[sensorsFound].errorCode = (uint8_t)sensor->GetLatestTemperature(temperature);
					msg->temperatureReports[sensorsFound].temperature = temperature;
					++sensorsFound;
				}
				currentSensorNumber = (unsigned int)sn + 1u;
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

		// Broadcast our heater statuses
		buf = CanMessageBuffer::Allocate();
		if (buf != nullptr)
		{
			CanMessageHeatersStatus * const msg = buf->SetupBroadcastMessage<CanMessageHeatersStatus>(CanInterface::GetCanAddress());
			msg->whichHeaters = 0;
			unsigned int heatersFound = 0;

			ReadLocker lock(heatersLock);

			for (size_t heater = 0; heater < MaxHeaters; ++heater)
			{
				Heater * const h = heaters[heater];
				if (h != nullptr)
				{
					msg->whichHeaters |= (uint64_t)1u << heater;
					msg->reports[heatersFound].mode = h->GetModeByte();
					msg->reports[heatersFound].averagePwm = (uint8_t)(h->GetAveragePWM() * 255.0);
					msg->reports[heatersFound].temperature = h->GetTemperature();
					++heatersFound;
				}
			}
			if (heatersFound == 0)
			{
				// Don't send an empty report
				CanMessageBuffer::Free(buf);
			}
			else
			{
				buf->dataLength = msg->GetActualDataLength(heatersFound);
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

	if (heater > MaxHeaters)
	{
		reply.copy("Heater number out of range");
		return GCodeResult::error;
	}

	PwmFrequency freq = DefaultFanPwmFreq;
	const bool seenFreq = parser.GetUintParam('Q', freq);

	String<StringLength20> pinName;
	if (parser.GetStringParam('C', pinName.GetRef()))
	{
		uint16_t sensorNumber;
		if (!parser.GetUintParam('T', sensorNumber))
		{
			reply.copy("Missing sensor number");
			return GCodeResult::error;
		}

		WriteLocker lock(heatersLock);

		Heater *oldHeater = nullptr;
		std::swap(oldHeater, heaters[heater]);
		delete oldHeater;

		Heater *newHeater = new LocalHeater(heater);
		const GCodeResult rslt = newHeater->ConfigurePortAndSensor(pinName.c_str(), freq, sensorNumber, reply);
		if (rslt == GCodeResult::ok)
		{
			heaters[heater] = newHeater;
		}
		else
		{
			delete newHeater;
		}
		return rslt;
	}

	const auto h = FindHeater(heater);
	if (h.IsNull())
	{
		reply.printf("No heater %u on board %u", (unsigned int)heater, CanInterface::GetCanAddress());
		return GCodeResult::error;
	}

	if (seenFreq)
	{
		return h->SetPwmFrequency(freq, reply);
	}

	return h->ReportDetails(reply);
}

GCodeResult Heat::ProcessM307(const CanMessageUpdateHeaterModel& msg, const StringRef& reply)
{
	const auto h = FindHeater(msg.heater);
	if (h.IsNull())
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
			String<StringLength20> sensorTypeName;
			bool newSensor = parser.GetStringParam('Y', sensorTypeName.GetRef());
			if (newSensor)
			{
				WriteLocker lock(sensorsLock);

				DeleteSensor(sensorNum);

				TemperatureSensor * const newSensor = TemperatureSensor::Create(sensorNum, sensorTypeName.c_str(), reply);
				if (newSensor == nullptr)
				{
					return GCodeResult::error;
				}

				const GCodeResult rslt = newSensor->Configure(parser, reply);
				if (rslt == GCodeResult::ok)
				{
					InsertSensor(newSensor);
				}
				else
				{
					delete newSensor;
				}
				return rslt;
			}

			const auto sensor = FindSensor(sensorNum);
			if (sensor.IsNull())
			{
				reply.printf("Sensor %u does not exist", sensorNum);
				return GCodeResult::error;
			}
			return sensor->Configure(parser, reply);
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

GCodeResult Heat::SetFaultDetection(const CanMessageSetHeaterFaultDetectionParameters& msg, const StringRef& reply)
{
	const auto h = FindHeater(msg.heater);
	if (h.IsNull())
	{
		reply.printf("Unknown heater %u", msg.heater);
		return GCodeResult::error;
	}

	return h->SetFaultDetectionParameters(msg.maxTempExcursion, msg.maxFaultTime);
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
	const auto sensor = FindSensor(sensorNum);
	if (sensor.IsNotNull())
	{
		float temp;
		err = sensor->GetLatestTemperature(temp);
		return temp;
	}

	err = TemperatureError::unknownSensor;
	return BadErrorTemperature;
}

void Heat::SwitchOff(int heater)
{
	const auto h = FindHeater(heater);
	if (h.IsNotNull())
	{
		h->SwitchOff();
	}
}

void Heat::SwitchOffAll()
{
	ReadLocker lock(heatersLock);

	for (size_t heater = 0; heater < MaxHeaters; ++heater)
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
	const auto h = FindHeater(heater);
	if (h.IsNotNull())
	{
		h->ResetFault();
	}
}

GCodeResult Heat::SetTemperature(const CanMessageSetHeaterTemperature& msg, const StringRef& reply)
{
	const auto h = FindHeater(msg.heaterNumber);
	if (h.IsNotNull())
	{
		return h->SetTemperature(msg, reply);
	}
	reply.printf("No heater %u on board %u", msg.heaterNumber, CanInterface::GetCanAddress());
	return GCodeResult::error;
}

float Heat::GetAveragePWM(size_t heater)
{
	const auto h = FindHeater(heater);
	return (h.IsNull()) ? 0.0 : h->GetAveragePWM();
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
ReadLockedPointer<TemperatureSensor> Heat::FindSensor(int sn)
{
	ReadLocker locker(sensorsLock);

	for (TemperatureSensor *sensor = sensorsRoot; sensor != nullptr; sensor = sensor->GetNext())
	{
		if ((int)sensor->GetSensorNumber() == sn)
		{
			return ReadLockedPointer<TemperatureSensor>(locker, sensor);
		}
	}
	return ReadLockedPointer<TemperatureSensor>(locker, nullptr);
}

// Get a pointer to the first temperature sensor with the specified or higher number
ReadLockedPointer<TemperatureSensor> Heat::FindSensorAtOrAbove(unsigned int sn)
{
	ReadLocker locker(sensorsLock);

	for (TemperatureSensor *sensor = sensorsRoot; sensor != nullptr; sensor = sensor->GetNext())
	{
		if (sensor->GetSensorNumber() >= sn)
		{
			return ReadLockedPointer<TemperatureSensor>(locker, sensor);
		}
	}
	return ReadLockedPointer<TemperatureSensor>(locker, nullptr);
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
