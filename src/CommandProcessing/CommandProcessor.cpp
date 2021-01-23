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
#include <FilamentMonitors/FilamentMonitor.h>
#include "CanMessageGenericParser.h"
#include <InputMonitors/InputMonitor.h>
#include <GPIO/GpioPorts.h>
#include <Platform.h>
#include <Movement/Move.h>
#include <Tasks.h>
#include <Version.h>
#include <AnalogIn.h>
#include <Hardware/NonVolatileMemory.h>
#include <hpl_user_area.h>
#include <cctype>				// for tolower()

#if SUPPORT_DRIVERS
# if SUPPORT_TMC22xx
#  include "Movement/StepperDrivers/TMC22xx.h"
# endif
# if SUPPORT_TMC51xx
#  include "Movement/StepperDrivers/TMC51xx.h"
 #endif
# if SUPPORT_CLOSED_LOOP
#  include <ClosedLoop/ClosedLoop.h>
# endif
#endif

#if HAS_VOLTAGE_MONITOR
constexpr float MinVin = 11.0;
constexpr float MaxVin = 32.0;
#endif

#if HAS_12V_MONITOR
constexpr float MinV12 = 10.0;
constexpr float MaxV12 = 13.5;
#endif

#if HAS_CPU_TEMP_SENSOR
constexpr float MinTemp = -20.0;
constexpr float MaxTemp = 55.0;
#endif

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

#if SUPPORT_DRIVERS

static GCodeResult SetMotorCurrents(const CanMessageMultipleDrivesRequest<float>& msg, size_t dataLength, const StringRef& reply)
{
# if HAS_SMART_DRIVERS
	const auto drivers = Bitmap<uint16_t>::MakeFromRaw(msg.driversToUpdate);
	if (dataLength < msg.GetActualDataLength(drivers.CountSetBits()))
	{
		reply.copy("bad data length");
		return GCodeResult::error;
	}

	GCodeResult rslt = GCodeResult::ok;
	drivers.Iterate([msg, reply, &rslt](unsigned int driver, unsigned int count) -> void
						{
							if (driver >= NumDrivers)
							{
								reply.lcatf("No such driver %u.%u", CanInterface::GetCanAddress(), driver);
								rslt = GCodeResult::error;
							}
							else
							{
								Platform::SetMotorCurrent(driver, msg.values[count]);
							}
						}
				   );
	return rslt;
# else
	reply.copy("Setting not available for external drivers");
	return GCodeResult::error;
# endif
}

static GCodeResult SetStandstillCurrentFactor(const CanMessageMultipleDrivesRequest<float>& msg, size_t dataLength, const StringRef& reply)
{
# if HAS_SMART_DRIVERS
	const auto drivers = Bitmap<uint16_t>::MakeFromRaw(msg.driversToUpdate);
	if (dataLength < msg.GetActualDataLength(drivers.CountSetBits()))
	{
		reply.copy("bad data length");
		return GCodeResult::error;
	}

	GCodeResult rslt = GCodeResult::ok;
	drivers.Iterate([msg, reply, &rslt](unsigned int driver, unsigned int count) -> void
						{
							if (driver >= NumDrivers)
							{
								reply.lcatf("No such driver %u.%u", CanInterface::GetCanAddress(), driver);
								rslt = GCodeResult::error;
							}
							else
							{
								SmartDrivers::SetStandstillCurrentPercent(driver, msg.values[count]);
							}
						}
				   );
	return rslt;
# else
	reply.copy("Setting not available for external drivers");
	return GCodeResult::error;
# endif
}

static GCodeResult HandlePressureAdvance(const CanMessageMultipleDrivesRequest<float>& msg, size_t dataLength, const StringRef& reply)
{
	const auto drivers = Bitmap<uint16_t>::MakeFromRaw(msg.driversToUpdate);
	if (dataLength < msg.GetActualDataLength(drivers.CountSetBits()))
	{
		reply.copy("bad data length");
		return GCodeResult::error;
	}

	GCodeResult rslt = GCodeResult::ok;
	drivers.Iterate([msg, reply, &rslt](unsigned int driver, unsigned int count) -> void
						{
							if (driver >= NumDrivers)
							{
								reply.lcatf("No such driver %u.%u", CanInterface::GetCanAddress(), driver);
								rslt = GCodeResult::error;
							}
							else
							{
								Platform::SetPressureAdvance(driver, msg.values[count]);
							}
						}
				   );
	return rslt;
}

static GCodeResult SetStepsPerMmAndMicrostepping(const CanMessageMultipleDrivesRequest<StepsPerUnitAndMicrostepping>& msg, size_t dataLength, const StringRef& reply)
{
	const auto drivers = Bitmap<uint16_t>::MakeFromRaw(msg.driversToUpdate);
	if (dataLength < msg.GetActualDataLength(drivers.CountSetBits()))
	{
		reply.copy("bad data length");
		return GCodeResult::error;
	}

	GCodeResult rslt = GCodeResult::ok;
	drivers.Iterate([msg, reply, &rslt](unsigned int driver, unsigned int count) -> void
						{
							if (driver >= NumDrivers)
							{
								reply.lcatf("No such driver %u.%u", CanInterface::GetCanAddress(), driver);
								rslt = GCodeResult::error;
							}
							else
							{
								Platform::SetDriveStepsPerUnit(driver, msg.values[count].GetStepsPerUnit());
#if HAS_SMART_DRIVERS
								const uint16_t microstepping = msg.values[count].GetMicrostepping() & 0x03FF;
								const bool interpolate = (msg.values[count].GetMicrostepping() & 0x8000) != 0;
								if (!SmartDrivers::SetMicrostepping(driver, microstepping, interpolate))
								{
									reply.lcatf("Driver %u.%u does not support x%u microstepping", CanInterface::GetCanAddress(), driver, microstepping);
									if (interpolate)
									{
										reply.cat(" with interpolation");
									}
									rslt = GCodeResult::error;
								}
#endif
							}
						}
					);
	return rslt;
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

#if SUPPORT_SLOW_DRIVERS
	float timings[4];
	size_t numTimings = 4;
	if (parser.GetFloatArrayParam('T', numTimings, timings))
	{
		seen = true;
		if (numTimings == 1)
		{
			timings[1] = timings[2] = timings[3] = timings[0];
		}
		else if (numTimings != 4)
		{
			reply.copy("bad timing parameter, expected 1 or 4 values");
			return GCodeResult::error;
		}
		Platform::SetDriverStepTiming(drive, timings);
	}
#endif

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

	size_t numHvalues = 3;
	const uint8_t *hvalues;
	if (parser.GetArrayParam('Y', ParamDescriptor::ParamType::uint8_array, numHvalues, hvalues))		// set spread cycle hysteresis
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
		reply.printf("Driver %u.%u runs %s, active %s enable",
						CanInterface::GetCanAddress(),
						drive,
						(Platform::GetDirectionValue(drive)) ? "forwards" : "in reverse",
						(Platform::GetEnableValue(drive)) ? "high" : "low");

#if SUPPORT_SLOW_DRIVERS
# if SINGLE_DRIVER
		if (Platform::IsSlowDriver())
# else
		if (Platform::IsSlowDriver(drive))
# endif
		{
			reply.catf(", step timing %.1f:%.1f:%.1f:%.1fus",
						(double)Platform::GetSlowDriverStepHighMicroseconds(),
						(double)Platform::GetSlowDriverStepLowMicroseconds(),
						(double)Platform::GetSlowDriverDirSetupMicroseconds(),
						(double)Platform::GetSlowDriverDirHoldMicroseconds());
		}
		else
		{
			reply.cat(", step timing fast");
		}
#endif

#if HAS_SMART_DRIVERS
		// It's a smart driver, so print the parameters common to all modes, except for the position
		reply.catf(", mode %s, ccr 0x%05" PRIx32 ", toff %" PRIu32 ", tblank %" PRIu32,
				TranslateDriverMode(SmartDrivers::GetDriverMode(drive)),
				SmartDrivers::GetRegister(drive, SmartDriverRegister::chopperControl),
				SmartDrivers::GetRegister(drive, SmartDriverRegister::toff),
				SmartDrivers::GetRegister(drive, SmartDriverRegister::tblank)
			);

# if SUPPORT_TMC51xx
		{
			const uint32_t thigh = SmartDrivers::GetRegister(drive, SmartDriverRegister::thigh);
			bool bdummy;
			const float mmPerSec = (12000000.0 * SmartDrivers::GetMicrostepping(drive, bdummy))/(256 * thigh * Platform::DriveStepsPerUnit(drive));
			reply.catf(", thigh %" PRIu32 " (%.1f mm/sec)", thigh, (double)mmPerSec);
		}
# endif

		// Print the additional parameters that are relevant in the current mode
		if (SmartDrivers::GetDriverMode(drive) == DriverMode::spreadCycle)
		{
			reply.catf(", hstart/hend/hdec %" PRIu32 "/%" PRIu32 "/%" PRIu32,
						SmartDrivers::GetRegister(drive, SmartDriverRegister::hstart),
						SmartDrivers::GetRegister(drive, SmartDriverRegister::hend),
						SmartDrivers::GetRegister(drive, SmartDriverRegister::hdec)
					  );
		}

# if SUPPORT_TMC22xx || SUPPORT_TMC51xx
		if (SmartDrivers::GetDriverMode(drive) == DriverMode::stealthChop)
		{
			const uint32_t tpwmthrs = SmartDrivers::GetRegister(drive, SmartDriverRegister::tpwmthrs);
			bool bdummy;
			const float mmPerSec = (12000000.0 * SmartDrivers::GetMicrostepping(drive, bdummy))/(256 * tpwmthrs * Platform::DriveStepsPerUnit(drive));
			const uint32_t pwmScale = SmartDrivers::GetRegister(drive, SmartDriverRegister::pwmScale);
			const uint32_t pwmAuto = SmartDrivers::GetRegister(drive, SmartDriverRegister::pwmAuto);
			const unsigned int pwmScaleSum = pwmScale & 0xFF;
			const int pwmScaleAuto = (int)((((pwmScale >> 16) & 0x01FF) ^ 0x0100) - 0x0100);
			const unsigned int pwmOfsAuto = pwmAuto & 0xFF;
			const unsigned int pwmGradAuto = (pwmAuto >> 16) & 0xFF;
			reply.catf(", tpwmthrs %" PRIu32 " (%.1f mm/sec), pwmScaleSum %u, pwmScaleAuto %d, pwmOfsAuto %u, pwmGradAuto %u",
						tpwmthrs, (double)mmPerSec, pwmScaleSum, pwmScaleAuto, pwmOfsAuto, pwmGradAuto);
		}
# endif
		// Finally, print the microstep position
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
#endif

	}
	return GCodeResult::ok;
}

static GCodeResult HandleSetDriverStates(const CanMessageMultipleDrivesRequest<DriverStateControl>& msg, const StringRef& reply)
{
	//TODO check message is long enough for the number of drivers specified
	const auto drivers = Bitmap<uint16_t>::MakeFromRaw(msg.driversToUpdate);
	drivers.Iterate([msg](unsigned int driver, unsigned int count) -> void
		{
			switch (msg.values[count].mode)
			{
			case DriverStateControl::driverActive:
				Platform::EnableDrive(driver);
				break;

			case DriverStateControl::driverIdle:
				Platform::SetDriverIdle(driver, msg.values[count].idlePercent);
				break;

			case DriverStateControl::driverDisabled:
			default:
				Platform::DisableDrive(driver);
				break;
			}
		});
	return GCodeResult::ok;
}

static GCodeResult ProcessM915(const CanMessageGeneric& msg, const StringRef& reply)
{
#if HAS_SMART_DRIVERS
	CanMessageGenericParser parser(msg, M915Params);
	uint16_t driverBits;
	if (!parser.GetUintParam('d', driverBits))
	{
		reply.copy("missing parameter in M915 message");
		return GCodeResult::error;
	}

	const auto drivers = DriversBitmap::MakeFromRaw(driverBits);

	bool seen = false;
	{
		int8_t sgThreshold;
		if (parser.GetIntParam('S', sgThreshold))
		{
			seen = true;
			drivers.Iterate([sgThreshold](unsigned int drive, unsigned int) noexcept { SmartDrivers::SetStallThreshold(drive, sgThreshold); });
		}
	}

	{
		uint16_t stepsPerSecond;
		if (parser.GetUintParam('H', stepsPerSecond))
		{
			seen = true;
			drivers.Iterate([stepsPerSecond](unsigned int drive, unsigned int) noexcept { SmartDrivers::SetStallMinimumStepsPerSecond(drive, stepsPerSecond); });
		}
	}

	{
		uint16_t coolStepConfig;
		if (parser.GetUintParam('T', coolStepConfig))
		{
			seen = true;
			drivers.Iterate([coolStepConfig](unsigned int drive, unsigned int) noexcept { SmartDrivers::SetRegister(drive, SmartDriverRegister::coolStep, coolStepConfig); } );
		}
	}

	if (!seen)
	{
		drivers.Iterate([&reply](unsigned int drive, unsigned int) noexcept
									{
										reply.lcatf("Driver %u.%u: ", CanInterface::GetCanAddress(), drive);
										SmartDrivers::AppendStallConfig(drive, reply);
									}
					   );
	}

	return GCodeResult::ok;
#else
	reply.copy("stall detection not supported by this board");
	return GCodeResult::error;
#endif
}

#endif	//SUPPORT_DRIVERS

static GCodeResult InitiateFirmwareUpdate(const CanMessageUpdateYourFirmware& msg, const StringRef& reply)
{
	if (msg.boardId != CanInterface::GetCanAddress() || msg.invertedBoardId != (uint8_t)~CanInterface::GetCanAddress() || (msg.module != 0 && msg.module != 3))
	{
		reply.printf("Invalid firmware update command received");
		return GCodeResult::error;
	}

	switch (msg.module)
	{
	case 0:		// main firmware
		reply.printf("Board %u starting firmware update", CanInterface::GetCanAddress());
		Platform::StartFirmwareUpdate();
		break;

	case 3:		// bootloader
#ifndef DEBUG			// debug builds of this firmware normally live at the start of memory and don't use a bootloader
		reply.printf("Board %u starting bootloader update", CanInterface::GetCanAddress());
		Platform::StartBootloaderUpdate();
#endif
		break;
	}
	return GCodeResult::ok;
}

static GCodeResult InitiateReset(const CanMessageReset& msg, const StringRef& reply)
{
	reply.printf("Board %u resetting", CanInterface::GetCanAddress());
	Platform::StartReset();
	return GCodeResult::ok;
}

static GCodeResult GetInfo(const CanMessageReturnInfo& msg, const StringRef& reply, uint8_t& extra)
{
	static constexpr uint8_t LastDiagnosticsPart = 7;				// the last diagnostics part is typeDiagnosticsPart0 + 7

	switch (msg.type)
	{
	case CanMessageReturnInfo::typeFirmwareVersion:
	default:
		reply.printf("%s (%s%s)", VersionText, IsoDate, TIME_SUFFIX);
		break;

	case CanMessageReturnInfo::typeBoardName:
		reply.copy(BOARD_TYPE_NAME);
		break;

	case CanMessageReturnInfo::typeBootloaderName:
		reply.copy(BOOTLOADER_NAME);
		break;

	case CanMessageReturnInfo::typeM408:
		// For now we ignore the parameter and always return the same set of info
		// This command is currently only used by the ATE, which needs the board type and the voltages
		reply.copy("{\"firmwareElectronics\":\"Duet 3 ");
		reply.cat(BOARD_TYPE_NAME);
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
			reply.lcatf("%s (%s%s)", VersionText, IsoDate, TIME_SUFFIX);
			const char *bootloaderVersionText = *reinterpret_cast<const char**>(0x20);		// offset of vectors.pvReservedM8
			reply.lcatf("Bootloader ID: %s", (bootloaderVersionText == nullptr) ? "not available" : bootloaderVersionText);
		}
		break;

	case CanMessageReturnInfo::typeDiagnosticsPart0 + 1:
		extra = LastDiagnosticsPart;
		Tasks::Diagnostics(reply);
		break;

	case CanMessageReturnInfo::typeDiagnosticsPart0 + 2:
		extra = LastDiagnosticsPart;
		{
			// We split the software reset data into two parts, because currently the buffer that the main board uses to receive each fragment isn't big enough to hold it all
			NonVolatileMemory mem;
			unsigned int slot;
			const SoftwareResetData *srd = mem.GetLastWrittenResetData(slot);
			if (srd == nullptr)
			{
				reply.copy("Last software reset data not available");
			}
			else
			{
				srd->PrintPart1(slot, reply);
			}
		}
		break;

	case CanMessageReturnInfo::typeDiagnosticsPart0 + 3:
		extra = LastDiagnosticsPart;
		{
			NonVolatileMemory mem;
			unsigned int slot;
			const SoftwareResetData *srd = mem.GetLastWrittenResetData(slot);
			if (srd != nullptr)
			{
				srd->PrintPart2(reply);
			}
		}
		break;

	case CanMessageReturnInfo::typeDiagnosticsPart0 + 4:
		extra = LastDiagnosticsPart;
#if SUPPORT_DRIVERS
# if SUPPORT_CLOSED_LOOP
		ClosedLoop::Diagnostics(reply);
# endif
		for (size_t driver = 0; driver < NumDrivers; ++driver)
		{

			reply.lcatf("Driver %u: position %" PRIi32 ", %.1f steps/mm"
# if HAS_SMART_DRIVERS
				", "
# endif
				, driver, moveInstance->GetPosition(driver), (double)Platform::DriveStepsPerUnit(driver));
# if HAS_SMART_DRIVERS
			SmartDrivers::AppendDriverStatus(driver, reply);
# endif
			reply.catf(", steps req %" PRIu32 " done %" PRIu32, DDA::stepsRequested[driver], DDA::stepsDone[driver]);
			DDA::stepsRequested[driver] = DDA::stepsDone[driver] = 0;
		}
#endif
		break;

	case CanMessageReturnInfo::typeDiagnosticsPart0 + 5:
		extra = LastDiagnosticsPart;
		{
#if SUPPORT_DRIVERS
			moveInstance->Diagnostics(reply);
#endif
			StepTimer::Diagnostics(reply);

#if HAS_VOLTAGE_MONITOR && HAS_12V_MONITOR
			reply.lcatf("VIN: %.1fV, V12: %.1fV", (double)Platform::GetCurrentVinVoltage(), (double)Platform::GetCurrentV12Voltage());
#elif HAS_VOLTAGE_MONITOR
			reply.lcatf("VIN: %.1fV", (double)Platform::GetCurrentVinVoltage());
#elif HAS_12V_MONITOR
			reply.lcatf("V12: %.1fV", (double)Platform::GetCurrentV12Voltage());
#endif

#if HAS_CPU_TEMP_SENSOR
			float minTemp, currentTemp, maxTemp;
			Platform::GetMcuTemperatures(minTemp, currentTemp, maxTemp);
			reply.lcatf("MCU temperature: min %.1fC, current %.1fC, max %.1fC", (double)minTemp, (double)currentTemp, (double)maxTemp);
#endif
			uint32_t conversionsStarted, conversionsCompleted, conversionTimeouts;
			AnalogIn::GetDebugInfo(conversionsStarted, conversionsCompleted, conversionTimeouts);
			reply.lcatf("Ticks since heat task active %" PRIu32 ", ADC conversions started %" PRIu32 ", completed %" PRIu32 ", timed out %" PRIu32,
						Platform::GetHeatTaskIdleTicks(), conversionsStarted, conversionsCompleted, conversionTimeouts);
		}
		break;

	case CanMessageReturnInfo::typeDiagnosticsPart0 + 6:
		extra = LastDiagnosticsPart;
		Heat::Diagnostics(reply);
		CanInterface::Diagnostics(reply);
#if 0
		{
			uint32_t nvmUserRow0 = *reinterpret_cast<const uint32_t*>(NVMCTRL_USER);
			uint32_t nvmUserRow1 = *reinterpret_cast<const uint32_t*>(NVMCTRL_USER+4);
			uint32_t nvmUserRow2 = *reinterpret_cast<const uint32_t*>(NVMCTRL_USER+8);
			uint32_t nvmUserRow3 = *reinterpret_cast<const uint32_t*>(NVMCTRL_USER+12);
			reply.lcatf("NVM user row %" PRIx32 " %" PRIx32 " %" PRIx32 " %" PRIx32, nvmUserRow0, nvmUserRow1, nvmUserRow2, nvmUserRow3);

#if SAMC21
			reply.lcatf("TSENS %06" PRIx32 " GAIN %06" PRIx32 " OFFS %06" PRIx32 " CAL %04" PRIx32,
						TSENS->VALUE.reg & 0x00FFFFFF, TSENS->GAIN.reg & 0x00FFFFFF, TSENS->OFFSET.reg & 0x00FFFFFF, TSENS->CAL.reg & 0x0000FFFF);
#endif
		}
#endif
		break;

	case CanMessageReturnInfo::typeDiagnosticsPart0 + 7:
		extra = LastDiagnosticsPart;
#if SUPPORT_DRIVERS
		FilamentMonitor::GetDiagnostics(reply);
#endif
		break;
	}
	return GCodeResult::ok;
}

void CommandProcessor::Spin()
{
	CanMessageBuffer *buf = CanInterface::GetCanCommand(0);
	if (buf != nullptr)
	{
		Platform::OnProcessingCanMessage();
		String<StringLength500> reply;
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

		case CanMessageType::updateHeaterModelOld:
			requestId = buf->msg.heaterModelOld.requestId;
			rslt = Heat::ProcessM307Old(buf->msg.heaterModelOld, replyRef);
			break;

		case CanMessageType::updateHeaterModelNew:
			requestId = buf->msg.heaterModelNew.requestId;
			rslt = Heat::ProcessM307New(buf->msg.heaterModelNew, replyRef);
			break;

		case CanMessageType::setHeaterTemperature:
			requestId = buf->msg.setTemp.requestId;
			rslt = Heat::SetTemperature(buf->msg.setTemp, replyRef);
			break;

		case CanMessageType::heaterTuningCommand:
			requestId = buf->msg.heaterTuningCommand.requestId;
			rslt = Heat::TuningCommand(buf->msg.heaterTuningCommand, replyRef);
			break;

		case CanMessageType::m308New:
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

#if SUPPORT_DRIVERS
		case CanMessageType::setMotorCurrents:
			requestId = buf->msg.multipleDrivesRequestFloat.requestId;
			rslt = SetMotorCurrents(buf->msg.multipleDrivesRequestFloat, buf->dataLength, replyRef);
			break;

		case CanMessageType::m569:
			requestId = buf->msg.generic.requestId;
			rslt = ProcessM569(buf->msg.generic, replyRef);
			break;

		case CanMessageType::m569p1:
			requestId = buf->msg.generic.requestId;
# if SUPPORT_CLOSED_LOOP
			rslt = ClosedLoop::ProcessM569Point1(buf->msg.generic, replyRef);
# else
			rslt = GCodeResult::errorNotSupported;
# endif
			break;

		case CanMessageType::setStandstillCurrentFactor:
			requestId = buf->msg.multipleDrivesRequestFloat.requestId;
			rslt = SetStandstillCurrentFactor(buf->msg.multipleDrivesRequestFloat, buf->dataLength, replyRef);
			break;

		case CanMessageType::setStepsPerMmAndMicrostepping:
			requestId = buf->msg.multipleDrivesStepsPerUnitAndMicrostepping.requestId;
			rslt = SetStepsPerMmAndMicrostepping(buf->msg.multipleDrivesStepsPerUnitAndMicrostepping, buf->dataLength, replyRef);
			break;

		case CanMessageType::setDriverStates:
			requestId = buf->msg.multipleDrivesRequestUint16.requestId;
			rslt = HandleSetDriverStates(buf->msg.multipleDrivesRequestDriverState, replyRef);
			break;

		case CanMessageType::m915:
			requestId = buf->msg.generic.requestId;
			rslt = ProcessM915(buf->msg.generic, replyRef);
			break;

		case CanMessageType::setPressureAdvance:
			requestId = buf->msg.multipleDrivesRequestFloat.requestId;
			rslt = HandlePressureAdvance(buf->msg.multipleDrivesRequestFloat, buf->dataLength, replyRef);
			break;
#endif

		case CanMessageType::updateFirmware:
			requestId = buf->msg.updateYourFirmware.requestId;
			rslt = InitiateFirmwareUpdate(buf->msg.updateYourFirmware, replyRef);
			break;

		case CanMessageType::reset:
			requestId = buf->msg.reset.requestId;
			rslt = InitiateReset(buf->msg.reset, replyRef);
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

		case CanMessageType::setHeaterMonitors:
			requestId = buf->msg.setHeaterMonitors.requestId;
			rslt = Heat::SetHeaterMonitors(buf->msg.setHeaterMonitors, replyRef);
			break;

		case CanMessageType::createInputMonitor:
			requestId = buf->msg.createInputMonitor.requestId;
			rslt = InputMonitor::Create(buf->msg.createInputMonitor, buf->dataLength, replyRef, extra);
			break;

		case CanMessageType::changeInputMonitor:
			requestId = buf->msg.changeInputMonitor.requestId;
			rslt = InputMonitor::Change(buf->msg.changeInputMonitor, replyRef, extra);
			break;

		case CanMessageType::readInputsRequest:
			// This one has its own reply message type
			InputMonitor::ReadInputs(buf);
			CanInterface::SendAndFree(buf);
			return;

		case CanMessageType::setAddressAndNormalTiming:
			requestId = buf->msg.setAddressAndNormalTiming.requestId;
			rslt = CanInterface::ChangeAddressAndDataRate(buf->msg.setAddressAndNormalTiming, replyRef);
			break;

#if 0
		case CanMessageType::setFastTiming:
			requestId = buf->msg.setFastTiming.requestId;
			rslt = CanInterface::SetFastTiming(buf->msg.setFastTiming, replyRef);
			break;
#endif

		case CanMessageType::diagnosticTest:
			requestId = buf->msg.diagnosticTest.requestId;
			rslt = Platform::DoDiagnosticTest(buf->msg.diagnosticTest, replyRef);
			break;

#if SUPPORT_DRIVERS
		case CanMessageType::createFilamentMonitor:
			requestId = buf->msg.createFilamentMonitor.requestId;
			rslt = FilamentMonitor::Create(buf->msg.createFilamentMonitor, replyRef);
			break;

		case CanMessageType::deleteFilamentMonitor:
			requestId = buf->msg.deleteFilamentMonitor.requestId;
			rslt = FilamentMonitor::Delete(buf->msg.deleteFilamentMonitor, replyRef);
			break;

		case CanMessageType::configureFilamentMonitor:
			requestId = buf->msg.generic.requestId;
			rslt = FilamentMonitor::Configure(buf->msg.generic, replyRef);
			break;
#endif
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
			const size_t fragmentLength = min<size_t>(totalLength - lengthDone, CanMessageStandardReply::MaxTextLength);
			memcpy(msg->text, reply.c_str() + lengthDone, fragmentLength);
			lengthDone += fragmentLength;
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
