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
#include <Platform/Platform.h>
#include <Platform/TaskPriorities.h>
#include "Sensors/TemperatureSensor.h"
#include "Sensors/RemoteSensor.h"
#include <CanMessageGenericParser.h>
#include <CanMessageBuffer.h>
#include <CanMessageGenericTables.h>
#include <CAN/CanInterface.h>
#include <Fans/FansManager.h>
#include <InputMonitors/InputMonitor.h>
#include <AppNotifyIndices.h>

#if SUPPORT_DHT_SENSOR
# include "Sensors/DhtSensor.h"
#endif

#if SUPPORT_TMC22xx
# include <Movement/StepperDrivers/TMC22xx.h>
#endif

#if SUPPORT_TMC51xx
# include <Movement/StepperDrivers/TMC51xx.h>
#endif

#if SUPPORT_LIS3DH
# include <CommandProcessing/AccelerometerHandler.h>
#endif

#include <Platform/Tasks.h>

// The task stack size must be large enough for calls to debugPrintf when a heater fault occurs.
constexpr uint32_t HeaterTaskStackWords = 230;					// task stack size in dwords

static Task<HeaterTaskStackWords> *heaterTask;

namespace Heat
{
	// Private members
	static Heater* heaters[MaxHeaters];							// A PID controller for each heater

	static TemperatureSensor * volatile sensorsRoot = nullptr;	// The sensor list, which must be maintained in sensor number order because the Heat task assumes that

	static float extrusionMinTemp;								// Minimum temperature to allow regular extrusion
	static float retractionMinTemp;								// Minimum temperature to allow regular retraction
	static bool coldExtrude;									// Is cold extrusion allowed?
	static int8_t heaterBeingTuned;								// which PID is currently being tuned

	static ReadWriteLock heatersLock;
	static ReadWriteLock sensorsLock;

	static uint64_t lastSensorsBroadcastWhich = 0;				// for diagnostics
	static uint32_t lastSensorsBroadcastWhen = 0;				// for diagnostics
	static unsigned int lastSensorsFound = 0;					// for diagnostics
	static uint32_t heatTaskLoopTime = 0;						// for diagnostics
	static unsigned int sensorOrderingErrors = 0;				// for diagnostics

	static uint8_t newDriverFaultState = 0;
	static uint8_t newHeaterFaultState = 0;

	static ReadLockedPointer<Heater> FindHeater(int heater)
	{
		return ReadLockedPointer<Heater>(heatersLock, (heater < 0 || heater >= (int)MaxHeaters) ? nullptr : heaters[heater]);
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

	// Broadcast our heater statuses
	static void SendHeatersStatus(CanMessageBuffer& buf)
	{
		CanMessageHeatersStatus * const msg = buf.SetupStatusMessage<CanMessageHeatersStatus>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress());
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
					msg->reports[heatersFound].SetTemperature(h->GetTemperature());
					++heatersFound;
				}
			}
		}

		if (heatersFound != 0)
		{
			buf.dataLength = msg->GetActualDataLength(heatersFound);
			CanInterface::Send(&buf);
		}
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

	for (Heater *& h : heaters)
	{
		h = nullptr;
	}

	// Set up the temperature (and other) sensors
	sensorsRoot = nullptr;

	extrusionMinTemp = DefaultMinExtrusionTemperature;
	retractionMinTemp = DefaultMinRetractionTemperature;
	coldExtrude = false;

	heaterTask = new Task<HeaterTaskStackWords>;
	heaterTask->Create(Heat::TaskLoop, "HEAT", nullptr, TaskPriority::HeatPriority);
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

	heaterTask->Suspend();
}

// This is the task loop executed by the Heat task. This task performs the following functions:
// - Spin the PIDs every 250ms. They must be spun at regular intervals for the I and D terms to work consistently.
// - Broadcast the sensor temperatures
// - Broadcast the status of our heaters
// - Broadcast the status of our fans
// - Broadcast the status of our motor drivers
[[noreturn]] void Heat::TaskLoop(void *)
{
	uint32_t nextWakeTime = millis();
	for (;;)
	{
		// Wait until we are woken or it's time to send another regular broadcast. If we are really unlucky, we could end up waiting for one tick too long.
		nextWakeTime += HeatSampleIntervalMillis;
		{
			const uint32_t now = millis();
			int32_t delayTime = (int32_t)(nextWakeTime - now);
			// If we're late (e.g. due to problems sending CAN messages), wait at least until the next tick to let other tasks run
			if (delayTime < 1)
			{
				nextWakeTime = now + 1;
				delayTime = 1;
			}
			TaskBase::TakeIndexed(NotifyIndices::Heat, (uint32_t)delayTime);
		}

		CanMessageBuffer buf;

#if SUPPORT_DRIVERS
		// Check whether we have any urgent messages to send
		if (newDriverFaultState == 1)
		{
			newDriverFaultState = 2;
			Platform::SendDriversStatus(buf);
		}
#endif

		// Check whether we have new heater fault status messages to send
		if (newHeaterFaultState == 1)
		{
			newHeaterFaultState = 2;
			SendHeatersStatus(buf);
		}

		// Check whether it is time to poll sensors and PIDs and send regular messages
		const uint32_t startTime = millis();
		if ((int32_t)(startTime - nextWakeTime) >= 0)
		{
			{
				// Walk the sensor list and poll all sensors
				// Also prepare to broadcast our sensor temperatures
				CanMessageSensorTemperatures * const sensorTempsMsg = buf.SetupBroadcastMessage<CanMessageSensorTemperatures>(CanInterface::GetCanAddress());
				sensorTempsMsg->whichSensors = 0;
				unsigned int sensorsFound = 0;
				{
					unsigned int nextUnreportedSensor = 0;
					ReadLocker lock(sensorsLock);
					for (TemperatureSensor *currentSensor = sensorsRoot; currentSensor != nullptr; currentSensor = currentSensor->GetNext())
					{
						currentSensor->Poll();
						if (currentSensor->GetBoardAddress() == CanInterface::GetCanAddress() && sensorsFound < ARRAY_SIZE(sensorTempsMsg->temperatureReports))
						{
							const unsigned int sn = currentSensor->GetSensorNumber();
							if (sn >= nextUnreportedSensor && sn < 64)
							{
								sensorTempsMsg->whichSensors |= (uint64_t)1u << sn;
								float temperature;
								sensorTempsMsg->temperatureReports[sensorsFound].errorCode = (uint8_t)(currentSensor->GetLatestTemperature(temperature).ToBaseType());
								sensorTempsMsg->temperatureReports[sensorsFound].SetTemperature(temperature);
								++sensorsFound;
								nextUnreportedSensor = sn + 1;
							}
							else
							{
								// We have a duplicate sensor number, or the sensors list is not ordered by sensor number, or the sensor number is out of range
								// Don't send its temperature because that will mess up the relationship between the bitmap and the sensor data in the message
								++sensorOrderingErrors;
							}
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

				// Broadcast our sensor temperatures
				lastSensorsBroadcastWhich = sensorTempsMsg->whichSensors;	// for diagnostics
				lastSensorsBroadcastWhen = millis();						// for diagnostics
				lastSensorsFound = sensorsFound;
				if (sensorsFound != 0)
				{
					buf.dataLength = sensorTempsMsg->GetActualDataLength(sensorsFound);
					CanInterface::Send(&buf);
				}
			}

			// See if we are tuning a heater, or have finished tuning one
			if (heaterBeingTuned != -1)
			{
				const auto h = FindHeater(heaterBeingTuned);
				if (h.IsNotNull() && h->IsTuning())
				{
					auto msg = buf.SetupStatusMessage<CanMessageHeaterTuningReport>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress());
					if (LocalHeater::GetTuningCycleData(*msg))
					{
						msg->SetStandardFields(heaterBeingTuned);
						CanInterface::Send(&buf);
					}
				}
				else
				{
					heaterBeingTuned = -1;
				}
			}

			if (newHeaterFaultState == 0)
			{
				SendHeatersStatus(buf);						// send the status of our heaters
			}
			else
			{
				newHeaterFaultState = 0;					// we recently sent it, so send it again next time
			}

			// Broadcast our fan RPMs
			{
				CanMessageFansReport * const msg = buf.SetupStatusMessage<CanMessageFansReport>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress());
				const unsigned int numReported = FansManager::PopulateFansReport(*msg);
				if (numReported != 0)
				{
					buf.dataLength = msg->GetActualDataLength(numReported);
					CanInterface::Send(&buf);
				}
			}

#if SUPPORT_DRIVERS
			if (newDriverFaultState == 0)
			{
				Platform::SendDriversStatus(buf);			// send the status of our drivers
			}
			else
			{
				newDriverFaultState = 0;					// we recently sent it, so send it again next time
			}
#endif

			// Announce ourselves to the main board, if it hasn't acknowledged us already
			if (!CanInterface::SendAnnounce(&buf))
			{
				// We didn't need to send an announcement so send a board health message instead
				CanMessageBoardStatus * const boardStatusMsg = buf.SetupStatusMessage<CanMessageBoardStatus>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress());
				boardStatusMsg->Clear();
				boardStatusMsg->neverUsedRam = Tasks::GetNeverUsedRam();

				// We must add fields in the following order: VIN, V12, MCU temperature
				size_t index = 0;
#if HAS_VOLTAGE_MONITOR
				boardStatusMsg->values[index++] = Platform::GetPowerVoltages(false);
				boardStatusMsg->hasVin = true;
#endif
#if HAS_12V_MONITOR
				boardStatusMsg->values[index++] = Platform::GetV12Voltages(false);
				boardStatusMsg->hasV12 = true;
#endif
#if HAS_CPU_TEMP_SENSOR
				boardStatusMsg->values[index++] = Platform::GetMcuTemperatures();
				boardStatusMsg->hasMcuTemp = true;
#endif
#if SUPPORT_LIS3DH
				boardStatusMsg->hasAccelerometer = AccelerometerHandler::IsPresent();
#endif
#if SUPPORT_CLOSED_LOOP
				boardStatusMsg->hasClosedLoop = true;
#endif
#if SUPPORT_LDC1612
				boardStatusMsg->hasInductiveSensor = true;
#endif
				// Add the analog handle data
				const size_t currentDataLength = boardStatusMsg->GetAnalogHandlesOffset();
				boardStatusMsg->numAnalogHandles = InputMonitor::AddAnalogHandleData((uint8_t*)boardStatusMsg + currentDataLength, 64 - currentDataLength);
				buf.dataLength = boardStatusMsg->GetActualDataLength();
				CanInterface::Send(&buf);
			}

			Platform::KickHeatTaskWatchdog();				// tell Platform that we are alive
			heatTaskLoopTime = millis() - startTime;
		}
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
		if (Succeeded(rslt))
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

GCodeResult Heat::ProcessM307New(const CanMessageHeaterModelNewNew& msg, const StringRef& reply)
{
	const auto h = FindHeater(msg.heater);
	return (h.IsNotNull()) ? h->SetModel(msg.heater, msg, reply) : UnknownHeater(msg.heater, reply);
}

GCodeResult Heat::ProcessM308(const CanMessageGeneric& msg, const StringRef& reply)
{
	CanMessageGenericParser parser(msg, M308NewParams);
	uint16_t sensorNum;
	if (parser.GetUintParam('S', sensorNum))
	{
		if (sensorNum < MaxSensors)
		{
			// Check for deleting the sensor by assigning a null port. Borrow the sensor type name string temporarily for this.
			String<StringLength50> sensorTypeName;							// StringLength20 is too short for "thermocouple-max31856"
			if (parser.GetStringParam('P', sensorTypeName.GetRef()) && sensorTypeName.EqualsIgnoreCase(NoPinName))
			{
				WriteLocker lock(sensorsLock);
				DeleteSensor(sensorNum);
				return GCodeResult::ok;
			}

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
				if (Succeeded(rslt))
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
			? h->SetFaultDetectionParameters(msg, reply)
				: UnknownHeater(msg.heater, reply);
}

GCodeResult Heat::SetHeaterMonitors(const CanMessageSetHeaterMonitors& msg, const StringRef& reply)
{
	const auto h = FindHeater(msg.heater);
	return (h.IsNotNull()) ? h->SetHeaterMonitors(msg, reply) : UnknownHeater(msg.heater, reply);
}

// Get the temperature of a sensor
float Heat::GetSensorTemperature(int sensorNum, TemperatureError& err) noexcept
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

void Heat::ProcessRemoteSensorsReport(CanAddress src, const CanMessageSensorTemperatures& msg) noexcept
{
	Bitmap<uint64_t> sensorsReported(msg.whichSensors);
	sensorsReported.Iterate([src, &msg](unsigned int sensor, unsigned int index)
								{
									if (index < ARRAY_SIZE(msg.temperatureReports) && src != CanInterface::GetCanAddress())
									{
										const CanSensorReport& sr = msg.temperatureReports[index];
										auto ts = FindSensor(sensor);
										if (ts.IsNotNull())
										{
											ts->UpdateRemoteTemperature(src, sr);
										}
										else
										{
											// Create a new RemoteSensor
											ts.Release();
											RemoteSensor * const rs = new RemoteSensor(sensor, src);
											rs->UpdateRemoteTemperature(src, sr);
											InsertSensor(rs);
										}
									}
								}
							);
}

void Heat::SwitchOffAll()
{
	ReadLocker lock(heatersLock);

	for (Heater * const h : heaters)
	{
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

GCodeResult Heat::TuningCommand(const CanMessageHeaterTuningCommand& msg, const StringRef& reply)
{
	if (heaterBeingTuned != -1 && heaterBeingTuned != (int)msg.heaterNumber)
	{
		reply.printf("Heater %d is already being tuned", heaterBeingTuned);
		return GCodeResult::error;
	}
	const auto h = FindHeater(msg.heaterNumber);
	if (h.IsNull())
	{
		return UnknownHeater(msg.heaterNumber, reply);
	}
	heaterBeingTuned = (int)msg.heaterNumber;			// setting this is OK even if we are stopping or fail to start tuning, because we check it in the heater task loop
	return h->TuningCommand(msg, reply);
}

GCodeResult Heat::FeedForward(const CanMessageHeaterFeedForward& msg, const StringRef& reply)
{
	const auto h = FindHeater(msg.heaterNumber);
	return (h.IsNull()) ? UnknownHeater(msg.heaterNumber, reply) : h->FeedForwardAdjustment(msg.fanPwmAdjustment, msg.extrusionAdjustment);
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
	reply.lcatf("Last sensors broadcast 0x%08" PRIx64 " found %u %" PRIu32 " ticks ago, %u ordering errs, loop time %" PRIu32,
					lastSensorsBroadcastWhich, lastSensorsFound, millis() - lastSensorsBroadcastWhen, sensorOrderingErrors, heatTaskLoopTime);
	sensorOrderingErrors = 0;
#if 0	// temporary to debug a board that reports bad Vssa
	reply.catf(", Vref %u Vssa %u",
		(unsigned int)(Platform::GetVrefFilter(0)->GetSum()/ThermistorAveragingFilter::NumAveraged()),
		(unsigned int)(Platform::GetVssaFilter(0)->GetSum()/ThermistorAveragingFilter::NumAveraged()));
#endif
}

void Heat::NewDriverFault()
{
	newDriverFaultState = 1;
	heaterTask->Give(NotifyIndices::Heat);
}

void Heat::NewHeaterFault()
{
	newHeaterFaultState = 1;
	heaterTask->Give(NotifyIndices::Heat);
}

// End
