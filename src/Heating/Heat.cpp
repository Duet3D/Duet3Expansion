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
#include <Platform.h>
#include <TaskPriorities.h>
#include "Sensors/TemperatureSensor.h"
#include "CanMessageGenericParser.h"
#include <CanMessageBuffer.h>
#include "CAN/CanInterface.h"
#include "Fans/FansManager.h"

#if SUPPORT_DHT_SENSOR
# include "Sensors/DhtSensor.h"
#endif

#include "Tasks.h"

constexpr uint32_t HeaterTaskStackWords = 120;					// task stack size in dwords, must be large enough for auto tuning when we implement it
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

	static float extrusionMinTemp;								// Minimum temperature to allow regular extrusion
	static float retractionMinTemp;								// Minimum temperature to allow regular retraction
	static bool coldExtrude;									// Is cold extrusion allowed?
	static int8_t heaterBeingTuned;								// which PID is currently being tuned
	static int8_t lastHeaterTuned;								// which PID we last finished tuning

	static ReadWriteLock heatersLock;
	static ReadWriteLock sensorsLock;

	static uint64_t lastSensorsBroadcastWhich = 0;				// for diagnostics
	static uint32_t lastSensorsBroadcastWhen = 0;				// for diagnostics
	static unsigned int lastSensorsFound = 0;					// for diagnostics
	static uint32_t heatTaskLoopTime = 0;						// for diagnostics

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

	static GCodeResult UnknownHeater(unsigned int heater, const StringRef& reply) noexcept
	{
		reply.printf("Board %u does not have heater %u", CanInterface::GetCanAddress(), heater);
		return GCodeResult::error;
	}
}

// Is the heater enabled?
bool Heat::IsHeaterEnabled(size_t heater)
{
	const auto h = FindHeater(heater);
	return h.IsNotNull() && h->IsHeaterEnabled();
}

void Heat::Init()
{
	coldExtrude = false;
	heaterBeingTuned = -1;
	lastHeaterTuned = -1;

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
	// Get a message buffer. We use the same one all the time and never release it.
	CanMessageBuffer *buf;
	while ((buf = CanMessageBuffer::Allocate()) == nullptr)
	{
		delay(5);
	}

	uint32_t lastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		const uint32_t startTime = millis();
		{
			// Walk the sensor list and poll all sensors
			// Also prepare to broadcast our sensor temperatures
			CanMessageSensorTemperatures * const sensorTempsMsg = buf->SetupBroadcastMessage<CanMessageSensorTemperatures>(CanInterface::GetCanAddress());
			sensorTempsMsg->whichSensors = 0;
			unsigned int sensorsFound = 0;
			{
				ReadLocker lock(sensorsLock);
				for (TemperatureSensor *currentSensor = sensorsRoot; currentSensor != nullptr; currentSensor = currentSensor->GetNext())
				{
					currentSensor->Poll();
					if (currentSensor->GetBoardAddress() == CanInterface::GetCanAddress() && sensorsFound < ARRAY_SIZE(sensorTempsMsg->temperatureReports))
					{
						sensorTempsMsg->whichSensors |= (uint64_t)1u << currentSensor->GetSensorNumber();
						float temperature;
						sensorTempsMsg->temperatureReports[sensorsFound].errorCode = (uint8_t)(currentSensor->GetLatestTemperature(temperature));
						sensorTempsMsg->temperatureReports[sensorsFound].SetTemperature(temperature);
						++sensorsFound;
					}
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

			// Announce ourselves to the main board
			CanInterface::SendAnnounce(buf);

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

			// Broadcast our sensor temperatures
			lastSensorsBroadcastWhich = sensorTempsMsg->whichSensors;	// for diagnostics
			lastSensorsBroadcastWhen = millis();						// for diagnostics
			lastSensorsFound = sensorsFound;
			if (sensorsFound != 0)
			{
				buf->dataLength = sensorTempsMsg->GetActualDataLength(sensorsFound);
				CanInterface::Send(buf);
			}
		}

		// Broadcast our heater statuses
		{
			CanMessageHeatersStatus * const msg = buf->SetupStatusMessage<CanMessageHeatersStatus>(CanInterface::GetCanAddress(), CanId::MasterAddress);
			msg->whichHeaters = 0;
			unsigned int heatersFound = 0;

			{
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
			}

			if (heatersFound != 0)
			{
				buf->dataLength = msg->GetActualDataLength(heatersFound);
				CanInterface::Send(buf);
			}
		}

		// Broadcast our fan RPMs
		{
			CanMessageFansReport * const msg = buf->SetupStatusMessage<CanMessageFansReport>(CanInterface::GetCanAddress(), CanId::MasterAddress);
			const unsigned int numReported = FansManager::PopulateFansReport(*msg);
			if (numReported != 0)
			{
				buf->dataLength = msg->GetActualDataLength(numReported);
				CanInterface::Send(buf);
			}
		}

		Platform::KickHeatTaskWatchdog();

		heatTaskLoopTime = millis() - startTime;

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

	if (heater >= MaxHeaters)
	{
		reply.copy("Heater number out of range");
		return GCodeResult::error;
	}

	PwmFrequency freq = DefaultFanPwmFreq;
	const bool seenFreq = parser.GetUintParam('Q', freq);

	String<StringLength50> pinName;
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
		if (rslt == GCodeResult::ok || rslt == GCodeResult::warning)
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
		return UnknownHeater(heater, reply);
	}

	if (seenFreq)
	{
		return h->SetPwmFrequency(freq, reply);
	}

	return h->ReportDetails(reply);
}

GCodeResult Heat::ProcessM307Old(const CanMessageUpdateHeaterModelOld& msg, const StringRef& reply)
{
	const auto h = FindHeater(msg.heater);
	return (h.IsNotNull()) ? h->SetOrReportModelOld(msg.heater, msg, reply) : UnknownHeater(msg.heater, reply);
}

GCodeResult Heat::ProcessM307New(const CanMessageUpdateHeaterModelNew& msg, const StringRef& reply)
{
	const auto h = FindHeater(msg.heater);
	return (h.IsNotNull()) ? h->SetOrReportModelNew(msg.heater, msg, reply) : UnknownHeater(msg.heater, reply);
}

GCodeResult Heat::ProcessM308(const CanMessageGeneric& msg, const StringRef& reply)
{
	CanMessageGenericParser parser(msg, M308NewParams);
	uint16_t sensorNum;
	if (parser.GetUintParam('S', sensorNum))
	{
		if (sensorNum < MaxSensors)
		{
			String<StringLength20> sensorTypeName;
			if (parser.GetStringParam('Y', sensorTypeName.GetRef()))
			{
				WriteLocker lock(sensorsLock);

				DeleteSensor(sensorNum);

				TemperatureSensor * const newSensor = TemperatureSensor::Create(sensorNum, sensorTypeName.c_str(), reply);
				if (newSensor == nullptr)
				{
					return GCodeResult::error;
				}

				const GCodeResult rslt = newSensor->Configure(parser, reply);
				if (rslt == GCodeResult::ok || rslt == GCodeResult::warning)
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
			reply.copy("Sensor number out of range");
			return GCodeResult::error;
		}
	}

	reply.copy("Missing sensor number parameter");
	return GCodeResult::error;
}

GCodeResult Heat::SetFaultDetection(const CanMessageSetHeaterFaultDetectionParameters& msg, const StringRef& reply)
{
	const auto h = FindHeater(msg.heater);
	return (h.IsNotNull())
			? h->SetFaultDetectionParameters(msg.maxTempExcursion, msg.maxFaultTime)
				: UnknownHeater(msg.heater, reply);
}

GCodeResult Heat::SetHeaterMonitors(const CanMessageSetHeaterMonitors& msg, const StringRef& reply)
{
	const auto h = FindHeater(msg.heater);
	return (h.IsNotNull()) ? h->SetHeaterMonitors(msg, reply) : UnknownHeater(msg.heater, reply);
}

float Heat::GetHighestTemperatureLimit(int heater) noexcept
{
	const auto h = FindHeater(heater);
	return (h.IsNull()) ? BadErrorTemperature : h->GetHighestTemperatureLimit();
}

float Heat::GetLowestTemperatureLimit(int heater) noexcept
{
	const auto h = FindHeater(heater);
	return (h.IsNull()) ? ABS_ZERO : h->GetLowestTemperatureLimit();
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
	return (h.IsNotNull()) ? h->SetTemperature(msg, reply) : UnknownHeater(msg.heaterNumber, reply);
}

float Heat::GetAveragePWM(size_t heater)
{
	const auto h = FindHeater(heater);
	return (h.IsNull()) ? 0.0 : h->GetAveragePWM();
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

void Heat::Diagnostics(const StringRef& reply)
{
	reply.lcatf("Last sensors broadcast 0x%08" PRIx64 " found %u %" PRIu32 " ticks ago, loop time %" PRIu32,
					lastSensorsBroadcastWhich, lastSensorsFound, millis() - lastSensorsBroadcastWhen, heatTaskLoopTime);
}

// End
