/*
 * FansManager.cpp
 *
 *  Created on: 3 Sep 2019
 *      Author: David
 */

#include "FansManager.h"

#include "LocalFan.h"
#include <CanMessageFormats.h>
#include <CanMessageGenericParser.h>
#include <CanMessageGenericTables.h>
#include <CAN/CanInterface.h>

#include <utility>

static ReadWriteLock fansLock;
static Fan *fans[MaxFans] = { 0 };

// Retrieve the pointer to a fan, or nullptr if it doesn't exist.
// Lock the fan system before calling this, so that the fan can't be deleted while we are accessing it.
static ReadLockedPointer<Fan> FindFan(uint32_t fanNum)
{
	return ReadLockedPointer<Fan>(fansLock, (fanNum < ARRAY_SIZE(fans)) ? fans[fanNum] : nullptr);
}

// Create and return a local fan. if it fails, return nullptr with the error message in 'reply'.
static LocalFan *CreateLocalFan(uint32_t fanNum, const char *pinNames, PwmFrequency freq, float ppr, const StringRef& reply)
{
	LocalFan *newFan = new LocalFan(fanNum);
	if (!newFan->AssignPorts(pinNames, reply))
	{
		delete newFan;
		return nullptr;
	}
	newFan->SetPwmFrequency(freq);
	newFan->SetTachoPulsesPerRev(ppr);
	return newFan;
}

// Check and if necessary update all fans. Return true if a thermostatic fan is running.
bool FansManager::CheckFans(bool checkSensors)
{
	ReadLocker lock(fansLock);
	bool thermostaticFanRunning = false;
	for (Fan* fan : fans)
	{
		if (fan != nullptr && fan->Check(checkSensors))
		{
			thermostaticFanRunning = true;
		}
	}
	return thermostaticFanRunning;
}

// This is called by M950 to create a fan or change its PWM frequency or report its port
GCodeResult FansManager::ConfigureFanPort(const CanMessageGeneric& msg, const StringRef& reply)
{
	CanMessageGenericParser parser(msg, M950FanParams);
	uint16_t fanNum;
	if (!parser.GetUintParam('F', fanNum))
	{
		reply.copy("Missing F parameter");
		return GCodeResult::error;
	}

	if (fanNum >= MaxFans)
	{
		reply.printf("Fan number %u too high", (unsigned int)fanNum);
		return GCodeResult::error;
	}

	PwmFrequency freq = DefaultFanPwmFreq;
	const bool seenFreq = parser.GetUintParam('Q', freq);
	float pulsesPerRev = DefaultFanTachoPulsesPerRev;
	const bool seenPpr = parser.GetFloatParam('K', pulsesPerRev);

	String<StringLength50> pinNames;
	if (parser.GetStringParam('C', pinNames.GetRef()))
	{
		WriteLocker lock(fansLock);

		Fan *oldFan = nullptr;
		std::swap(oldFan, fans[fanNum]);
		delete oldFan;

		fans[fanNum] = CreateLocalFan(fanNum, pinNames.c_str(), freq, pulsesPerRev, reply);
		return (fans[fanNum] == nullptr) ? GCodeResult::error : GCodeResult::ok;
	}

	const auto fan = FindFan(fanNum);
	if (fan.IsNull())
	{
		reply.printf("Board %u doesn't have fan %u", CanInterface::GetCanAddress(), fanNum);
		return GCodeResult::error;
	}

	if (seenFreq)
	{
		fan->SetPwmFrequency(freq);
	}
	if (seenPpr)
	{
		fan->SetTachoPulsesPerRev(pulsesPerRev);
	}

	if (!seenFreq && !seenPpr)
	{
		fan->ReportPortDetails(reply);
	}
	return GCodeResult::ok;
}

// Set or report the parameters for the specified fan
// If 'mCode' is an M-code used to set parameters for the current kinematics (which should only ever be 106 or 107)
// then search for parameters used to configure the fan. If any are found, perform appropriate actions and return true.
// If errors were discovered while processing parameters, put an appropriate error message in 'reply' and set 'error' to true.
// If no relevant parameters are found, print the existing ones to 'reply' and return false.
GCodeResult FansManager::ConfigureFan(const CanMessageFanParameters& msg, const StringRef& reply)
{
	auto fan = FindFan(msg.fanNumber);
	if (fan.IsNull())
	{
		reply.printf("Board %u doesn't have fan %u", CanInterface::GetCanAddress(), msg.fanNumber);
		return GCodeResult::error;
	}

	return fan->Configure(msg, reply);
}

GCodeResult FansManager::SetFanSpeed(const CanMessageSetFanSpeed& msg, const StringRef& reply)
{
	auto fan = FindFan(msg.fanNumber);
	if (fan.IsNull())
	{
		reply.printf("Board %u doesn't have fan %u", CanInterface::GetCanAddress(), msg.fanNumber);
		return GCodeResult::error;
	}

	fan->SetPwm(msg.pwm);
	return GCodeResult::ok;
}

#if 0

void FansManager::SetFanValue(uint32_t fanNum, float speed)
{
	auto fan = FindFan(fanNum);
	if (fan.IsNotNull())
	{
		fan->SetPwm(speed);
	}
}

#endif

// Initialise fans. Call this only once, and only during initialisation.
void FansManager::Init()
{
	for (Fan*& f : fans)
	{
		f = nullptr;
	}
}

// Construct a fan RPM report message. Returns the number of fans reported in it.
unsigned int FansManager::PopulateFansReport(CanMessageFansReport& msg)
{
	ReadLocker locker(fansLock);

	msg.whichFans = 0;
	unsigned int numReported = 0;
	for (Fan* f : fans)
	{
		if (f != nullptr)
		{
			msg.fanReports[numReported].actualPwm = (uint16_t)(f->GetLastVal() * 65535);
			msg.fanReports[numReported].rpm = f->GetRPM();
			msg.whichFans |= (uint64_t)1 << f->GetNumber();
			++numReported;
		}
	}
	return numReported;
}

// End
