/*
 * CommandProcessor.cpp
 *
 *  Created on: 26 Jul 2019
 *      Author: David
 */

#include "CommandProcessor.h"
#include <CAN/CanInterface.h>
#include "CanMessageBuffer.h"
#include "GCodes/GCodeResult.h"
#include "Heating/Heat.h"
#include "Fans/FansManager.h"
#include "CanMessageGenericParser.h"
#include <Endstops/EndstopsManager.h>
#include <InputMonitors/InputMonitor.h>
#include <GPIO/GpioPorts.h>
#include <Platform.h>
#include <Movement/Move.h>
#include <Tasks.h>
#include <Version.h>
#include <Hardware/AnalogIn.h>
#include <hpl_user_area.h>
#include <cctype>				// for tolower()

#if SUPPORT_TMC22xx
# include "Movement/StepperDrivers/TMC22xx.h"
#endif
#if SUPPORT_TMC51xx
# include "Movement/StepperDrivers/TMC51xx.h"
#endif

constexpr float MinVin = 11.0;
constexpr float MaxVin = 32.0;
constexpr float MinV12 = 10.0;
constexpr float MaxV12 = 13.5;
constexpr float MinTemp = -20.0;
constexpr float MaxTemp = 55.0;

static void GenerateTestReport(const StringRef& reply)
{
	bool testFailed = false;

#if HAS_CPU_TEMP_SENSOR
	// Check the MCU temperature
	{
		float minMcuTemperature, currentMcuTemperature, maxMcuTemperature;
		Platform::GetMcuTemperatures(minMcuTemperature, currentMcuTemperature, maxMcuTemperature);
		if (currentMcuTemperature < MinTemp)
		{
			reply.lcatf("MCU temperature %.1fC is lower than expected", (double)currentMcuTemperature);
			testFailed = true;
		}
		else if (currentMcuTemperature > MaxTemp)
		{
			reply.lcatf("MCU temperature %.1fC is higher than expected", (double)currentMcuTemperature);
			testFailed = true;
		}
		else
		{
			reply.lcatf("MCU temperature reading OK (%.1fC)", (double)currentMcuTemperature);
		}
	}
#endif

#if HAS_VOLTAGE_MONITOR
	// Check the supply voltage
	{
		const float voltage = Platform::GetCurrentVinVoltage();
		if (voltage < MinVin)
		{
			reply.lcatf("VIN voltage reading %.1f is lower than expected", (double)voltage);
			testFailed = true;
		}
		else if (voltage > MaxVin)
		{
			reply.lcatf("VIN voltage reading %.1f is higher than expected", (double)voltage);
			testFailed = true;
		}
		else
		{
			reply.lcatf("VIN voltage reading OK (%.1fV)", (double)voltage);
		}
	}
#endif

#if HAS_12V_MONITOR
	// Check the 12V rail voltage
	{
		const float voltage = Platform::GetCurrentV12Voltage();
		if (voltage < MinV12)
		{
			reply.lcatf("12V voltage reading %.1f is lower than expected", (double)voltage);
			testFailed = true;
		}
		else if (voltage > MaxV12)
		{
			reply.lcatf("12V voltage reading %.1f is higher than expected", (double)voltage);
			testFailed = true;
		}
		else
		{
			reply.lcatf("12V voltage reading OK (%.1fV)", (double)voltage);
		}
	}
#endif

#if HAS_SMART_DRIVERS
	// Check the stepper driver status
	bool driversOK = true;
	for (size_t driver = 0; driver < NumDrivers; ++driver)
	{
		const uint32_t stat = SmartDrivers::GetAccumulatedStatus(driver, 0xFFFFFFFF);
		if ((stat & (TMC_RR_OT || TMC_RR_OTPW)) != 0)
		{
			reply.lcatf("Driver %u reports over temperature", driver);
			driversOK = false;
		}
		if ((stat & TMC_RR_S2G) != 0)
		{
			reply.lcatf("Driver %u reports short-to-ground", driver);
			driversOK = false;
		}
	}
	if (driversOK)
	{
		reply.lcatf("Driver status OK");
	}
	else
	{
		testFailed = true;
	}
#endif

	reply.lcatf((testFailed) ? "***** ONE OR MORE CHECKS FAILED *****" : "All checks passed");

	if (!testFailed)
	{
		reply.lcat("Board ID: ");
		Platform::AppendUniqueId(reply);
	}
}

static GCodeResult SetMotorCurrents(const CanMessageMultipleDrivesRequest& msg, const StringRef& reply)
{
#if HAS_SMART_DRIVERS
	//TODO check message is long enough for the number of drivers specified
	const uint16_t *p = msg.values;
	for (unsigned int driver = 0; driver < NumDrivers; ++driver)
	{
		if (IsBitSet(msg.driversToUpdate, driver))
		{
			Platform::SetMotorCurrent(driver, (float)*p);		//TODO avoid the int->float->int conversion
			++p;
		}
	}
	return GCodeResult::ok;
#else
	reply.copy("Setting not available for external drivers");
	return GCodeResult::error;
#endif
}

static GCodeResult SetStandstillCurrentFactor(const CanMessageMultipleDrivesRequest& msg, const StringRef& reply)
{
#if HAS_SMART_DRIVERS
	//TODO check message is long enough for the number of drivers specified
	const uint16_t *p = msg.values;
	for (unsigned int driver = 0; driver < NumDrivers; ++driver)
	{
		if (IsBitSet(msg.driversToUpdate, driver))
		{
			SmartDrivers::SetStandstillCurrentPercent(driver, (float)*p);		//TODO avoid the int->float->int conversion
			++p;
		}
	}
	return GCodeResult::ok;
#else
	reply.copy("Setting not available for external drivers");
	return GCodeResult::error;
#endif
}

static GCodeResult HandlePressureAdvance(const CanMessageMultipleDrivesRequest& msg, const StringRef& reply)
{
	//TODO check message is long enough for the number of drivers specified
	const uint16_t *p = msg.values;
	for (unsigned int driver = 0; driver < NumDrivers; ++driver)
	{
		if (IsBitSet(msg.driversToUpdate, driver))
		{
			Platform::SetPressureAdvance(driver, (float)*p * 0.001);
		}
	}
	return GCodeResult::ok;
}

static GCodeResult SetMicrostepping(const CanMessageMultipleDrivesRequest& msg, const StringRef& reply)
{
#if HAS_SMART_DRIVERS
	//TODO check message is long enough for the number of drivers specified
	const uint16_t *p = msg.values;
	GCodeResult rslt = GCodeResult::ok;
	for (unsigned int driver = 0; driver < NumDrivers; ++driver)
	{
		if (IsBitSet(msg.driversToUpdate, driver))
		{
			const uint16_t microstepping = *p & 0x03FF;
			const bool interpolate = (*p & 0x8000) != 0;
			if (!SmartDrivers::SetMicrostepping(driver, microstepping, interpolate))
			{
				reply.lcatf("Driver %u.%u does not support x%u microstepping", CanInterface::GetCanAddress(), driver, microstepping);
				if (interpolate)
				{
					reply.cat(" with interpolation");
				}
				rslt = GCodeResult::error;
			}
			++p;
		}
	}
	return rslt;
#else
	reply.copy("Setting not available for external drivers");
	return GCodeResult::error;
#endif
}

static GCodeResult ProcessM569(const CanMessageGeneric& msg, const StringRef& reply)
{
	CanMessageGenericParser parser(msg, M569Params);
	uint8_t drive;
	if (!parser.GetUintParam('P', drive))
	{
		reply.copy("Missing P parameter in CAN message");
		return GCodeResult::error;
	}

	if (drive >= NumDrivers)
	{
		reply.printf("Driver number %u.%u out of range", CanInterface::GetCanAddress(), drive);
		return GCodeResult::error;
	}

	bool seen = false;
	uint8_t direction;
	if (parser.GetUintParam('S', direction))
	{
		seen = true;
		Platform::SetDirectionValue(drive, direction != 0);
	}
	int8_t rValue;
	if (parser.GetIntParam('R', rValue))
	{
		seen = true;
		Platform::SetEnableValue(drive, rValue);
	}
	size_t numTimings;
	const float *timings;
	if (parser.GetFloatArrayParam('T', numTimings, timings))
	{
		seen = true;
		if (numTimings != 4)
		{
			reply.copy("bad timing parameter");
			return GCodeResult::error;
		}
		//TODO timings is unaligned, so we should really either copy it or change SetDriverStepTiming to accept a pointer to unaligned data
		Platform::SetDriverStepTiming(drive, timings);
	}

#if HAS_SMART_DRIVERS
	{
		uint32_t val;
		if (parser.GetUintParam('D', val))	// set driver mode
		{
			seen = true;
			if (!SmartDrivers::SetDriverMode(drive, val))
			{
				reply.printf("Driver %u.%u does not support mode '%s'", CanInterface::GetCanAddress(), drive, TranslateDriverMode(val));
				return GCodeResult::error;
			}
		}

		if (parser.GetUintParam('F', val))		// set off time
		{
			seen = true;
			if (!SmartDrivers::SetRegister(drive, SmartDriverRegister::toff, val))
			{
				reply.printf("Bad off time for driver %u", drive);
				return GCodeResult::error;
			}
		}

		if (parser.GetUintParam('B', val))		// set blanking time
		{
			seen = true;
			if (!SmartDrivers::SetRegister(drive, SmartDriverRegister::tblank, val))
			{
				reply.printf("Bad blanking time for driver %u", drive);
				return GCodeResult::error;
			}
		}

		if (parser.GetUintParam('V', val))		// set microstep interval for changing from stealthChop to spreadCycle
		{
			seen = true;
			if (!SmartDrivers::SetRegister(drive, SmartDriverRegister::tpwmthrs, val))
			{
				reply.printf("Bad mode change microstep interval for driver %u", drive);
				return GCodeResult::error;
			}
		}

#if SUPPORT_TMC51xx
		if (parser.GetUintParam('H', val))		// set coolStep threshold
		{
			seen = true;
			if (!SmartDrivers::SetRegister(drive, SmartDriverRegister::thigh, val))
			{
				reply.printf("Bad high speed microstep interval for driver %u", drive);
				return GCodeResult::error;
			}
		}
#endif
	}

	size_t numHvalues;
	const uint8_t *hvalues;
	if (parser.GetUint8ArrayParam('Y', numHvalues, hvalues))		// set spread cycle hysteresis
	{
		seen = true;
		if (numHvalues == 2 || numHvalues == 3)
		{
			// There is a constraint on the sum of HSTRT and HEND, so set HSTART then HEND then HSTART again because one may go up and the other down
			(void)SmartDrivers::SetRegister(drive, SmartDriverRegister::hstart, hvalues[0]);
			bool ok = SmartDrivers::SetRegister(drive, SmartDriverRegister::hend, hvalues[1]);
			if (ok)
			{
				ok = SmartDrivers::SetRegister(drive, SmartDriverRegister::hstart, hvalues[0]);
			}
			if (ok && numHvalues == 3)
			{
				ok = SmartDrivers::SetRegister(drive, SmartDriverRegister::hdec, hvalues[2]);
			}
			if (!ok)
			{
				reply.printf("Bad hysteresis setting for driver %u", drive);
				return GCodeResult::error;
			}
		}
		else
		{
			reply.copy("Expected 2 or 3 Y values");
			return GCodeResult::error;
		}
	}
#endif
	if (!seen)
	{
		reply.printf("Drive %u.%u runs %s, active %s enable, step timing ",
						CanInterface::GetCanAddress(),
						drive,
						(Platform::GetDirectionValue(drive)) ? "forwards" : "in reverse",
						(Platform::GetEnableValue(drive)) ? "high" : "low");
		if (Platform::IsSlowDriver(drive))
		{
			reply.catf("%.1f:%.1f:%.1f:%.1fus",
						(double)Platform::GetSlowDriverStepHighClocks(),
						(double)Platform::GetSlowDriverStepLowClocks(),
						(double)Platform::GetSlowDriverDirSetupClocks(),
						(double)Platform::GetSlowDriverDirHoldClocks());
		}
		else
		{
			reply.cat("fast");
		}
#if HAS_SMART_DRIVERS
		reply.catf(", mode %s, ccr 0x%05" PRIx32 ", toff %" PRIu32 ", tblank %" PRIu32 ", hstart/hend/hdec %" PRIu32 "/%" PRIu32 "/%" PRIu32,
				TranslateDriverMode(SmartDrivers::GetDriverMode(drive)),
				SmartDrivers::GetRegister(drive, SmartDriverRegister::chopperControl),
				SmartDrivers::GetRegister(drive, SmartDriverRegister::toff),
				SmartDrivers::GetRegister(drive, SmartDriverRegister::tblank),
				SmartDrivers::GetRegister(drive, SmartDriverRegister::hstart),
				SmartDrivers::GetRegister(drive, SmartDriverRegister::hend),
				SmartDrivers::GetRegister(drive, SmartDriverRegister::hdec)
			);

# if SUPPORT_TMC2660
		{
			const uint32_t mstepPos = SmartDrivers::GetRegister(drive, SmartDriverRegister::mstepPos);
			if (mstepPos < 1024)
			{
				reply.catf(", pos %" PRIu32, mstepPos);
			}
			else
			{
				reply.cat(", pos unknown");
			}
		}
# elif SUPPORT_TMC22xx || SUPPORT_TMC51xx
		{
			const uint32_t tpwmthrs = SmartDrivers::GetRegister(drive, SmartDriverRegister::tpwmthrs);
			const uint32_t mstepPos = SmartDrivers::GetRegister(drive, SmartDriverRegister::mstepPos);
			bool bdummy;
			const float mmPerSec = (12000000.0 * SmartDrivers::GetMicrostepping(drive, bdummy))/(256 * tpwmthrs * Platform::DriveStepsPerUnit(drive));
			reply.catf(", pos %" PRIu32", tpwmthrs %" PRIu32 " (%.1f mm/sec)", mstepPos, tpwmthrs, (double)mmPerSec);
		}
# endif

# if SUPPORT_TMC51xx
		{
			const uint32_t thigh = SmartDrivers::GetRegister(drive, SmartDriverRegister::thigh);
			bool bdummy;
			const float mmPerSec = (12000000.0 * SmartDrivers::GetMicrostepping(drive, bdummy))/(256 * thigh * Platform::DriveStepsPerUnit(drive));
			reply.catf(", thigh %" PRIu32 " (%.1f mm/sec)", thigh, (double)mmPerSec);
		}
# endif
#endif

	}
	return GCodeResult::ok;
}

static GCodeResult HandleSetDriverStates(const CanMessageMultipleDrivesRequest& msg, const StringRef& reply)
{
	//TODO check message is long enough for the number of drivers specified
	const uint16_t *p = msg.values;
	for (unsigned int driver = 0; driver < NumDrivers; ++driver)
	{
		if (IsBitSet(msg.driversToUpdate, driver))
		{
			switch (*p)
			{
			case CanMessageMultipleDrivesRequest::driverActive:
				Platform::EnableDrive(driver);
				break;

			case CanMessageMultipleDrivesRequest::driverIdle:
				Platform::SetDriverIdle(driver);
				break;

			case CanMessageMultipleDrivesRequest::driverDisabled:
			default:
				Platform::DisableDrive(driver);
				break;
			}
			++p;
		}
	}
	return GCodeResult::ok;
}

static GCodeResult ProcessM915(const CanMessageGeneric& msg, const StringRef& reply)
{
	reply.printf("M915 not yet implemented for remote drivers");
	return GCodeResult::error;
}

static GCodeResult InitiateFirmwareUpdate(const CanMessageUpdateYourFirmware& msg, const StringRef& reply)
{
	if (msg.boardId != CanInterface::GetCanAddress() || msg.invertedBoardId != (uint8_t)~CanInterface::GetCanAddress())
	{
		reply.printf("Invalid firmware update command received");
		return GCodeResult::error;
	}
	reply.printf("Board %u about to start firmware update", CanInterface::GetCanAddress());
	Platform::StartFirmwareUpdate();
	return GCodeResult::ok;
}

static GCodeResult GetInfo(const CanMessageReturnInfo& msg, const StringRef& reply, uint8_t& extra)
{
	static constexpr uint8_t LastDiagnosticsPart = 3;				// the last diagnostics part is typeDiagnosticsPart0 + 3

	switch (msg.type)
	{
	case CanMessageReturnInfo::typeFirmwareVersion:
	default:
		reply.printf("Board %s firmware %s", BoardTypeName, FirmwareVersion);
		break;

	case CanMessageReturnInfo::typeBoardName:
		reply.copy(BoardTypeName);
		break;

	case CanMessageReturnInfo::typeM408:
		// For now we ignore the parameter and always return the same set of info
		// This command is currently only used by the ATE, which needs the board type and the voltages
		reply.copy("{\"firmwareElectronics\":\"Duet 3 ");
		reply.cat(BoardTypeName);
		reply.cat("\"");
#if HAS_VOLTAGE_MONITOR
		reply.catf(",\"vin\":{\"min\":%.1f,\"cur\":%.1f,\"max\":%.1f}",
					(double)Platform::GetMinVinVoltage(), (double)Platform::GetCurrentVinVoltage(), (double)Platform::GetMaxVinVoltage());
#endif
#if HAS_12V_MONITOR
		reply.catf(",\"v12\":{\"min\":%.1f,\"cur\":%.1f,\"max\":%.1f}",
					(double)Platform::GetMinV12Voltage(), (double)Platform::GetCurrentV12Voltage(), (double)Platform::GetMaxV12Voltage());
#endif
		reply.cat('}');
		break;

	case CanMessageReturnInfo::typeDiagnosticsPart0:
		if (msg.param == 1)
		{
			GenerateTestReport(reply);
		}
		else
		{
			extra = LastDiagnosticsPart;
			reply.lcatf("Board %s firmware %s", BoardTypeName, FirmwareVersion);
			Tasks::Diagnostics(reply);
		}
		break;

	case CanMessageReturnInfo::typeDiagnosticsPart0 + 1:
		extra = LastDiagnosticsPart;
#if HAS_SMART_DRIVERS
		for (size_t driver = 0; driver < NumDrivers; ++driver)
		{
			reply.lcatf("Driver %u:", driver);
			SmartDrivers::AppendDriverStatus(driver, reply);
		}
#else
		reply.copy("External motor driver(s)");			// to avoid a blank line in the M122 report
#endif
		break;

	case CanMessageReturnInfo::typeDiagnosticsPart0 + 2:
		extra = LastDiagnosticsPart;
		{
			float minTemp, currentTemp, maxTemp;
			Platform::GetMcuTemperatures(minTemp, currentTemp, maxTemp);
			reply.printf("Moves scheduled %" PRIu32 ", completed %" PRIu32 ", hiccups %" PRIu32,
							moveInstance->GetScheduledMoves(), moveInstance->GetCompletedMoves(), moveInstance->GetAndClearHiccups());
#if HAS_VOLTAGE_MONITOR && HAS_12V_MONITOR
			reply.catf("\nVIN: %.1fV, V12: %.1fV", (double)Platform::GetCurrentVinVoltage(), (double)Platform::GetCurrentV12Voltage());
#elif HAS_VOLTAGE_MONITOR
			reply.catf("\nVIN: %.1fV", (double)Platform::GetCurrentVinVoltage());
#elif HAS_12V_MONITOR
			reply.catf("\nV12: %.1fV", (double)Platform::GetCurrentV12Voltage());
#endif
			reply.catf("\nMCU temperature: min %.1fC, current %.1fC, max %.1fC", (double)minTemp, (double)currentTemp, (double)maxTemp);
			uint32_t conversionsStarted, conversionsCompleted, conversionTimeouts;
			AnalogIn::GetDebugInfo(conversionsStarted, conversionsCompleted, conversionTimeouts);
			reply.catf("\nTicks since heat task active %" PRIu32 ", ADC conversions started %" PRIu32 ", completed %" PRIu32 ", timed out %" PRIu32,
						Platform::GetHeatTaskIdleTicks(), conversionsStarted, conversionsCompleted, conversionTimeouts);
		}
		break;

	case CanMessageReturnInfo::typeDiagnosticsPart0 + 3:
		extra = LastDiagnosticsPart;
		Heat::Diagnostics(reply);
		{
			uint32_t nvmUserRow0 = *reinterpret_cast<const uint32_t*>(NVMCTRL_USER);
			uint32_t nvmUserRow1 = *reinterpret_cast<const uint32_t*>(NVMCTRL_USER+4);
			uint32_t nvmUserRow2 = *reinterpret_cast<const uint32_t*>(NVMCTRL_USER+8);
			uint32_t nvmUserRow3 = *reinterpret_cast<const uint32_t*>(NVMCTRL_USER+12);
			reply.lcatf("NVM user row %" PRIx32 " %" PRIx32 " %" PRIx32 " %" PRIx32, nvmUserRow0, nvmUserRow1, nvmUserRow2, nvmUserRow3);

#ifdef SAMC21
			reply.lcatf("TSENS %06" PRIx32 " GAIN %06" PRIx32 " OFFS %06" PRIx32 " CAL %04" PRIx32,
						TSENS->VALUE.reg & 0x00FFFFFF, TSENS->GAIN.reg & 0x00FFFFFF, TSENS->OFFSET.reg & 0x00FFFFFF, TSENS->CAL.reg & 0x0000FFFF);
#endif
		}
		break;

#if 1	//debug
	case CanMessageReturnInfo::typePressureAdvance:
		reply.copy("Pressure advance:");
		for (size_t i = 0; i < NumDrivers; ++i)
		{
			reply.catf(" %.2f", (double)Platform::GetPressureAdvance(i));
		}
		break;
#endif
	}
	return GCodeResult::ok;
}

void CommandProcessor::Spin()
{
	CanMessageBuffer *buf = CanInterface::GetCanCommand();
	if (buf != nullptr)
	{
		String<FormatStringLength> reply;
		const StringRef& replyRef = reply.GetRef();
		const CanMessageType id = buf->id.MsgType();
		GCodeResult rslt;
		CanRequestId requestId;
		uint8_t extra = 0;

		switch (id)
		{
		case CanMessageType::returnInfo:
			requestId = buf->msg.getInfo.requestId;
			rslt = GetInfo(buf->msg.getInfo, replyRef, extra);
			break;

		case CanMessageType::updateHeaterModel:
			requestId = buf->msg.heaterModel.requestId;
			rslt = Heat::ProcessM307(buf->msg.heaterModel, replyRef);
			break;

		case CanMessageType::setHeaterTemperature:
			requestId = buf->msg.setTemp.requestId;
			rslt = Heat::SetTemperature(buf->msg.setTemp, replyRef);
			break;

		case CanMessageType::m308:
			requestId = buf->msg.generic.requestId;
			rslt = Heat::ProcessM308(buf->msg.generic, replyRef);
			break;

		case CanMessageType::m950Fan:
			requestId = buf->msg.generic.requestId;
			rslt = FansManager::ConfigureFanPort(buf->msg.generic, replyRef);
			break;

		case CanMessageType::m950Heater:
			requestId = buf->msg.generic.requestId;
			rslt = Heat::ConfigureHeater(buf->msg.generic, replyRef);
			break;

		case CanMessageType::m950Gpio:
			requestId = buf->msg.generic.requestId;
			rslt = GpioPorts::HandleM950Gpio(buf->msg.generic, replyRef);
			break;

		case CanMessageType::writeGpio:
			requestId = buf->msg.writeGpio.requestId;
			rslt = GpioPorts::HandleGpioWrite(buf->msg.writeGpio, replyRef);
			break;

		case CanMessageType::setMotorCurrents:
			requestId = buf->msg.multipleDrivesRequest.requestId;
			rslt = SetMotorCurrents(buf->msg.multipleDrivesRequest, replyRef);
			break;

		case CanMessageType::m569:
			requestId = buf->msg.generic.requestId;
			rslt = ProcessM569(buf->msg.generic, replyRef);
			break;

		case CanMessageType::setStandstillCurrentFactor:
			requestId = buf->msg.multipleDrivesRequest.requestId;
			rslt = SetStandstillCurrentFactor(buf->msg.multipleDrivesRequest, replyRef);
			break;

		case CanMessageType::setMicrostepping:
			requestId = buf->msg.multipleDrivesRequest.requestId;
			rslt = SetMicrostepping(buf->msg.multipleDrivesRequest, replyRef);
			break;

		case CanMessageType::updateFirmware:
			requestId = buf->msg.updateYourFirmware.requestId;
			rslt = InitiateFirmwareUpdate(buf->msg.updateYourFirmware, replyRef);
			break;

		case CanMessageType::fanParameters:
			requestId = buf->msg.fanParameters.requestId;
			rslt = FansManager::ConfigureFan(buf->msg.fanParameters, replyRef);
			break;

		case CanMessageType::setFanSpeed:
			requestId = buf->msg.setFanSpeed.requestId;
			rslt = FansManager::SetFanSpeed(buf->msg.setFanSpeed, replyRef);
			break;

		case CanMessageType::setHeaterFaultDetection:
			requestId = buf->msg.setHeaterFaultDetection.requestId;
			rslt = Heat::SetFaultDetection(buf->msg.setHeaterFaultDetection, replyRef);
			break;

		case CanMessageType::setDriverStates:
			requestId = buf->msg.multipleDrivesRequest.requestId;
			rslt = HandleSetDriverStates(buf->msg.multipleDrivesRequest, replyRef);
			break;

		case CanMessageType::m915:
			requestId = buf->msg.generic.requestId;
			rslt = ProcessM915(buf->msg.generic, replyRef);
			break;

		case CanMessageType::setPressureAdvance:
			requestId = buf->msg.multipleDrivesRequest.requestId;
			rslt = HandlePressureAdvance(buf->msg.multipleDrivesRequest, replyRef);
			break;

		case CanMessageType::createZProbe:
			requestId = buf->msg.createZProbe.requestId;
			rslt = EndstopsManager::CreateZProbe(buf->msg.createZProbe, buf->dataLength, replyRef);
			break;

		case CanMessageType::configureZProbe:
			requestId = buf->msg.configureZProbe.requestId;
			rslt = EndstopsManager::ConfigureZProbe(buf->msg.configureZProbe, replyRef, extra);
			break;

		case CanMessageType::getZProbePinNames:
			requestId = buf->msg.getZProbePinNames.requestId;
			rslt = EndstopsManager::GetZProbePinNames(buf->msg.getZProbePinNames, replyRef);
			break;

		case CanMessageType::destroyZProbe:
			requestId = buf->msg.destroyZProbe.requestId;
			rslt = EndstopsManager::DestroyZProbe(buf->msg.destroyZProbe, replyRef);
			break;

		case CanMessageType::setProbing:
			requestId = buf->msg.setProbing.requestId;
			rslt = EndstopsManager::SetProbing(buf->msg.setProbing, replyRef);
			break;

		case CanMessageType::createInputMonitor:
			requestId = buf->msg.createInputMonitor.requestId;
			rslt = InputMonitor::Create(buf->msg.createInputMonitor, buf->dataLength, replyRef, extra);
			break;

		case CanMessageType::changeInputMonitor:
			requestId = buf->msg.changeInputMonitor.requestId;
			rslt = InputMonitor::Change(buf->msg.changeInputMonitor, replyRef, extra);
			break;

		default:
			requestId = CanRequestIdAcceptAlways;
			reply.printf("Board %u received unknown msg type %u", CanInterface::GetCanAddress(), (unsigned int)buf->id.MsgType());
			rslt = GCodeResult::error;
			break;
		}

		// Re-use the message buffer to send a standard reply
		const CanAddress srcAddress = buf->id.Src();
		CanMessageStandardReply *msg = buf->SetupResponseMessage<CanMessageStandardReply>(requestId, CanInterface::GetCanAddress(), srcAddress);
		msg->resultCode = (uint16_t)rslt;
		msg->extra = extra;
		const size_t totalLength = reply.strlen();
		size_t lengthDone = 0;
		uint8_t fragmentNumber = 0;
		for (;;)
		{
			size_t fragmentLength = min<size_t>(totalLength - lengthDone, CanMessageStandardReply::MaxTextLength);
			memcpy(msg->text, reply.c_str() + lengthDone, fragmentLength);
			lengthDone += fragmentLength;
			if (fragmentLength < ARRAY_SIZE(msg->text))
			{
				msg->text[fragmentLength] = 0;
				++fragmentLength;
			}
			buf->dataLength = msg->GetActualDataLength(fragmentLength);
			msg->fragmentNumber = fragmentNumber;
			if (lengthDone == totalLength)
			{
				msg->moreFollows = false;
				CanInterface::SendAndFree(buf);
				break;
			}
			msg->moreFollows = true;
			CanInterface::Send(buf);
			++fragmentNumber;
		}
	}
}

// End
