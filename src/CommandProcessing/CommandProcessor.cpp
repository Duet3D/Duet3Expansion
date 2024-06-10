/*
 * CommandProcessor.cpp
 *
 *  Created on: 26 Jul 2019
 *      Author: David
 */

#include "CommandProcessor.h"
#include <CAN/CanInterface.h>
#include <CanMessageBuffer.h>
#include <Heating/Heat.h>
#include <Fans/FansManager.h>
#include <FilamentMonitors/FilamentMonitor.h>
#include <CanMessageGenericParser.h>
#include <CanMessageGenericTables.h>
#include <InputMonitors/InputMonitor.h>
#include <GPIO/GpioPorts.h>
#include <LedStrips/LedStripManager.h>
#include <Platform/Platform.h>
#include <Movement/Move.h>
#include <Platform/Tasks.h>
#include <AnalogIn.h>
#include <Hardware/NonVolatileMemory.h>

#if !RP2040
# include <hpl_user_area.h>
#endif

#include <cctype>				// for tolower()

#if SUPPORT_DRIVERS
# if SUPPORT_TMC22xx
#  include "Movement/StepperDrivers/TMC22xx.h"
# endif
# if SUPPORT_TMC51xx || SUPPORT_TMC2160
#  include "Movement/StepperDrivers/TMC51xx.h"
# endif
# if SUPPORT_CLOSED_LOOP
#  include <ClosedLoop/ClosedLoop.h>
# endif
#endif

#if SUPPORT_LIS3DH
# include "AccelerometerHandler.h"
#endif

#if SUPPORT_LDC1612
# include "ScanningSensorHandler.h"
#endif

#if SUPPORT_AS5601
# include "MFMHandler.h"
#endif

// Check a value against the specified min and max parameters returning true if the value was outside limits
static bool CheckMinMax(CanMessageGenericParser& parser, const StringRef& reply, char c, float val, const char *text) noexcept
{
	float minMaxValues[2];
	size_t numValues = 2;
	if (parser.GetFloatArrayParam(c, numValues, minMaxValues) && numValues == 2)
	{
		reply.lcatf("%s is %.1f, ", text, (double)val);
		if (val < minMaxValues[0])
		{
			reply.cat("too low");
			return true;
		}
		else if (val > minMaxValues[1])
		{
			reply.cat("too high");
			return true;
		}
		else
		{
			reply.cat("OK");
		}
	}
	else
	{
		reply.lcatf("Missing %c parameter", c);
		return true;
	}
	return false;
}

// Generate a test report
static GCodeResult GenerateTestReport(const CanMessageGeneric &msg, const StringRef& reply) noexcept
{
	CanMessageGenericParser parser(msg, M122P1Params);
	bool testFailed = false;

#if HAS_CPU_TEMP_SENSOR
	// Check the MCU temperature
	const MinCurMax& mcuTemperature = Platform::GetMcuTemperatures();
	testFailed |= CheckMinMax(parser, reply, 'T', mcuTemperature.current, "MCU temp");
#endif

#if HAS_VOLTAGE_MONITOR
	// Check the supply voltage
	testFailed |= CheckMinMax(parser, reply, 'V', Platform::GetCurrentVinVoltage(), "VIN voltage");
#endif

#if HAS_12V_MONITOR
	// Check the 12V rail voltage
	testFailed |= CheckMinMax(parser, reply, 'W', Platform::GetCurrentV12Voltage(), "12V voltage");
#endif

#if HAS_SMART_DRIVERS
	// Check the stepper driver status
	bool driversOK = true;
	for (size_t driver = 0; driver < NumDrivers; ++driver)
	{
		const StandardDriverStatus stat = SmartDrivers::GetStatus(driver, true, false);
		if (stat.ot || stat.otpw)
		{
			reply.lcatf("Driver %u reports over temperature", driver);
			driversOK = false;
		}
		if (stat.s2ga || stat.s2gb || stat.s2vsa || stat.s2vsb)
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

#if SUPPORT_LIS3DH
	if (Platform::AlwaysHasAccelerometer())
	{
		if (AccelerometerHandler::IsPresent())
		{
			reply.lcat("Accelerometer detected");
		}
		else
		{
			reply.lcat("Accelerometer NOT detected");
			testFailed = true;
		}
	}
#endif

#if SUPPORT_LDC1612
	if (Platform::AlwaysHasLDC1612())
	{
		testFailed |= CheckMinMax(parser, reply, 'F', ScanningSensorHandler::GetFrequency() * 1000.0, "Inductive sensor frequency");
	}
#endif

	reply.lcat((testFailed) ? "***** ONE OR MORE CHECKS FAILED *****" : "All checks passed");

	if (!testFailed)
	{
		reply.lcat("Board ID: ");
		Platform::GetUniqueId().AppendCharsToString(reply);
	}

	return (testFailed) ? GCodeResult::warning : GCodeResult::ok;
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
	drivers.Iterate([&msg, &reply, &rslt](unsigned int driver, unsigned int count) -> void
						{
							if (driver >= NumDrivers)
							{
								reply.lcatf("No such driver %u.%u", CanInterface::GetCanAddress(), driver);
								rslt = GCodeResult::error;
							}
							else
							{
								Platform::SetMotorCurrent(driver, msg.values[count]);
#if SUPPORT_CLOSED_LOOP
								ClosedLoop::GetClosedLoopInstance(driver)->UpdateStandstillCurrent();
#endif
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
	drivers.Iterate([&msg, &reply, &rslt](unsigned int driver, unsigned int count) -> void
						{
							if (driver >= NumDrivers)
							{
								reply.lcatf("No such driver %u.%u", CanInterface::GetCanAddress(), driver);
								rslt = GCodeResult::error;
							}
							else
							{
								SmartDrivers::SetStandstillCurrentPercent(driver, msg.values[count]);
#if SUPPORT_CLOSED_LOOP
								ClosedLoop::GetClosedLoopInstance(driver)->UpdateStandstillCurrent();
#endif
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
	drivers.Iterate([&msg, &reply, &rslt](unsigned int driver, unsigned int count) -> void
						{
							if (driver >= NumDrivers)
							{
								reply.lcatf("No such driver %u.%u", CanInterface::GetCanAddress(), driver);
								rslt = GCodeResult::error;
							}
							else
							{
								moveInstance->GetExtruderShaper(driver).SetKseconds(msg.values[count]);
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
	drivers.Iterate([&msg, &reply, &rslt](unsigned int driver, unsigned int count) -> void
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
								if (!moveInstance->SetMicrostepping(driver, microstepping, interpolate))
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
# if SUPPORT_CLOSED_LOOP
			// Enable/disabled closed loop control
			const ClosedLoopMode mode = (val == (uint32_t)DriverMode::direct) ? ClosedLoopMode::closed
										: (val == (uint32_t)DriverMode::direct + 1) ? ClosedLoopMode::assistedOpen
											: ClosedLoopMode::open;
			if (!ClosedLoop::GetClosedLoopInstance(drive)->SetClosedLoopEnabled(mode, reply))
			{
				// reply.printf is done in ClosedLoop::SetClosedLoopEnabled()
				return GCodeResult::error;
			}
# endif
			if (!SmartDrivers::SetDriverMode(drive, val))
			{
				reply.printf("Driver %u.%u does not support mode '%s'", CanInterface::GetCanAddress(), drive, TranslateDriverMode(val));
				return GCodeResult::error;
			}
# if SUPPORT_CLOSED_LOOP
			if (mode != ClosedLoopMode::open)
			{
				ClosedLoop::GetClosedLoopInstance(drive)->DriverSwitchedToClosedLoop();
			}
# endif
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

#if SUPPORT_TMC51xx || SUPPORT_TMC2160
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
		const DriverMode dmode = SmartDrivers::GetDriverMode(drive);
		reply.catf(", mode %s", TranslateDriverMode(dmode));
# if SUPPORT_CLOSED_LOOP
		if (dmode == DriverMode::direct)
		{
			reply.catf(" (%s)", ClosedLoop::GetClosedLoopInstance(drive)->GetModeText());
		}
# endif
		reply.catf(", ccr 0x%05" PRIx32 ", toff %" PRIu32 ", tblank %" PRIu32,
					SmartDrivers::GetRegister(drive, SmartDriverRegister::chopperControl),
					SmartDrivers::GetRegister(drive, SmartDriverRegister::toff),
					SmartDrivers::GetRegister(drive, SmartDriverRegister::tblank)
				  );

# if SUPPORT_TMC51xx || SUPPORT_TMC2160
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

# if SUPPORT_TMC22xx || SUPPORT_TMC51xx || SUPPORT_TMC2160
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

static GCodeResult ProcessM569Point2(const CanMessageGeneric& msg, const StringRef& reply)
{
#if SUPPORT_TMC22xx || SUPPORT_TMC51xx || SUPPORT_TMC2160
	CanMessageGenericParser parser(msg, M569Point2Params);
	uint8_t drive;
	uint8_t regNum;
	if (!parser.GetUintParam('P', drive) || !parser.GetUintParam('R', regNum))
	{
		reply.copy("Missing P or R parameter in CAN message");
		return GCodeResult::error;
	}

	if (drive >= NumDrivers)
	{
		reply.printf("Driver number %u.%u out of range", CanInterface::GetCanAddress(), drive);
		return GCodeResult::error;
	}

	uint32_t regVal;
	if (parser.GetUintParam('V', regVal))
	{
		return SmartDrivers::SetAnyRegister(drive, reply, regNum, regVal);
	}

	const uint32_t startTime = millis();
	GCodeResult rslt;
	while ((rslt = SmartDrivers::GetAnyRegister(drive, reply, regNum)) == GCodeResult::notFinished)
	{
		if (millis() - startTime >= 50)
		{
			reply.copy("Failed to read register");
			return GCodeResult::error;
		}
	}

	return rslt;
#else
	return GCodeResult::errorNotSupported;
#endif
}

static GCodeResult HandleSetDriverStates(const CanMessageMultipleDrivesRequest<DriverStateControl>& msg, const StringRef& reply)
{
	//TODO check message is long enough for the number of drivers specified
	const auto drivers = Bitmap<uint16_t>::MakeFromRaw(msg.driversToUpdate);
	drivers.Iterate([&msg](unsigned int driver, unsigned int count) -> void
		{
			switch (msg.values[count].mode)
			{
			case DriverStateControl::driverActive:
				Platform::EnableDrive(driver, msg.values[count].idlePercentOrDelayAfterBrakeOn);
				break;

			case DriverStateControl::driverIdle:
				Platform::SetDriverIdle(driver, msg.values[count].idlePercentOrDelayAfterBrakeOn >> 4);
				break;

			case DriverStateControl::driverDisabled:
			default:
				{
					const uint16_t delay = msg.values[count].idlePercentOrDelayAfterBrakeOn;
					Platform::DisableDrive(driver, (delay == 0) ? DefaultDelayAfterBrakeOn : delay);
				}
				break;
			}
		});
	return GCodeResult::ok;
}

static GCodeResult ProcessM915(const CanMessageGeneric& msg, const StringRef& reply)
{
#if HAS_SMART_DRIVERS && HAS_STALL_DETECT
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

	{
		uint8_t rParam;
		if (parser.GetUintParam('R', rParam))
		{
			seen = true;
			Platform::SetOrResetEventOnStall(drivers, rParam != 0);
		}
	}

	if (!seen)
	{
		drivers.Iterate([&reply](unsigned int drive, unsigned int) noexcept
									{
										reply.lcatf("Driver %u.%u: ", CanInterface::GetCanAddress(), drive);
										SmartDrivers::AppendStallConfig(drive, reply);
										reply.cat(", event on stall: ");
										reply.cat((Platform::GetEventOnStall(drive)) ? "yes" : "no");
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

	default:
		reply.copy("unknown firmware module number");
		return GCodeResult::error;
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
		// This must be formatted in a specific way for the ATE
		Platform::AppendBoardAndFirmwareDetails(reply);
		break;

	case CanMessageReturnInfo::typeBoardName:
		reply.copy(BOARD_TYPE_NAME);
		// Set bit 0 of 'extra' to 1 if the board takes a .uf2 file, leave it as 0 if it takes a .bin file
#if BOARD_USES_UF2_BINARY
		extra |= 0x01;
#endif
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
		{
			const MinCurMax vinVoltage = Platform::GetPowerVoltages(false);
			reply.catf(",\"vin\":{\"min\":%.1f,\"cur\":%.1f,\"max\":%.1f}",
						(double)vinVoltage.minimum, (double)vinVoltage.current, (double)vinVoltage.maximum);
		}
#endif
#if HAS_12V_MONITOR
		{
			const MinCurMax v12Voltage = Platform::GetV12Voltages(false);
			reply.catf(",\"v12\":{\"min\":%.1f,\"cur\":%.1f,\"max\":%.1f}",
						(double)v12Voltage.minimum, (double)v12Voltage.current, (double)v12Voltage.maximum);
		}
#endif
		reply.cat('}');
		break;

	case CanMessageReturnInfo::typeBoardUniqueId:
		Platform::GetUniqueId().AppendCharsToString(reply);
		break;

	case CanMessageReturnInfo::typeDiagnosticsPart0:
		extra = LastDiagnosticsPart;
		{
			Platform::AppendBoardAndFirmwareDetails(reply);
			// GCC 12.2 produces a spurious diagnostic for the following line of code, see https://gcc.gnu.org/bugzilla/show_bug.cgi?id=105523
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Warray-bounds"
			const char *bootloaderVersionText = *reinterpret_cast<const char**>(0x20);		// offset of vectors.pvReservedM8
#pragma GCC diagnostic pop
			reply.lcatf("Bootloader ID: %s", (bootloaderVersionText == nullptr) ? "not available" : bootloaderVersionText);
			Platform::AppendDiagnostics(reply);
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
			NonVolatileMemory mem(NvmPage::common);
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
			NonVolatileMemory mem(NvmPage::common);
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
		for (size_t driver = 0; driver < NumDrivers; ++driver)
		{
			reply.lcatf("Driver %u: pos %" PRIi32 ", %.1f steps/mm"
# if HAS_SMART_DRIVERS
				", "
# endif
				, driver, moveInstance->GetPosition(driver), (double)Platform::DriveStepsPerUnit(driver));
# if HAS_SMART_DRIVERS
			const StandardDriverStatus status = SmartDrivers::GetStatus(driver, false, false);
			status.AppendText(reply, 0);
			if (!status.notPresent)
			{
				SmartDrivers::AppendDriverStatus(driver, reply);
			}
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

#if HAS_VOLTAGE_MONITOR
			{
				const MinCurMax vin = Platform::GetPowerVoltages(true);
				reply.lcatf("VIN voltage: min %.1f, current %.1f, max %.1f", (double)vin.minimum, (double)vin.current, (double)vin.maximum);
			}
#endif
#if HAS_12V_MONITOR
			{
				const MinCurMax v12 = Platform::GetV12Voltages(true);
				reply.lcatf("V12 voltage: min %.1f, current %.1f, max %.1f", (double)v12.minimum, (double)v12.current, (double)v12.maximum);
			}
#endif

#if HAS_CPU_TEMP_SENSOR
			const MinCurMax& mcuTemperature = Platform::GetMcuTemperatures();
			reply.lcatf("MCU temperature: min %.1fC, current %.1fC, max %.1fC", (double)mcuTemperature.minimum, (double)mcuTemperature.current, (double)mcuTemperature.maximum);
#endif
#if 0	// the following are no longer needed
			uint32_t conversionsStarted, conversionsCompleted, conversionTimeouts, errors;
			AnalogIn::GetDebugInfo(conversionsStarted, conversionsCompleted, conversionTimeouts, errors);
			reply.lcatf("Ticks since heat task active %" PRIu32 ", ADC conversions started %" PRIu32 ", completed %" PRIu32 ", timed out %" PRIu32 ", errs %" PRIu32,
						Platform::GetHeatTaskIdleTicks(), conversionsStarted, conversionsCompleted, conversionTimeouts, errors);
#endif
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
#if SUPPORT_CLOSED_LOOP
		ClosedLoop::Diagnostics(reply);
#endif
#if SUPPORT_LIS3DH
		AccelerometerHandler::Diagnostics(reply);
#endif
#if SUPPORT_LDC1612
		ScanningSensorHandler::AppendDiagnostics(reply);
#endif
#if SUPPORT_AS5601
		MFMHandler::AppendDiagnostics(reply);
#endif
#if SUPPORT_I2C_SENSORS
		Platform::GetSharedI2C().Diagnostics(reply);
#endif
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
		if (buf->id.Dst() != CanId::BroadcastAddress)
		{
			Platform::OnProcessingCanMessage();
		}
		String<StringLength500> reply;
		const StringRef& replyRef = reply.GetRef();
		const CanMessageType id = buf->id.MsgType();
		GCodeResult rslt;
		CanRequestId requestId;
		uint8_t extra = 0;

		switch (id)
		{
		case CanMessageType::sensorTemperaturesReport:
			requestId = CanRequestIdNoReplyNeeded;
			Heat::ProcessRemoteSensorsReport(buf->id.Src(), buf->msg.sensorTemperaturesBroadcast);
			break;

		case CanMessageType::returnInfo:
			requestId = buf->msg.getInfo.requestId;
			rslt = GetInfo(buf->msg.getInfo, replyRef, extra);
			break;

		case CanMessageType::heaterModelNewNew:
			requestId = buf->msg.heaterModelNewNew.requestId;
			rslt = Heat::ProcessM307New(buf->msg.heaterModelNewNew, replyRef);
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

		case CanMessageType::heaterFeedForward:
			requestId = buf->msg.heaterFeedForward.requestId;
			rslt = Heat::FeedForward(buf->msg.heaterFeedForward, replyRef);
			break;

		case CanMessageType::m950Gpio:
			requestId = buf->msg.generic.requestId;
			rslt = GpioPorts::HandleM950Gpio(buf->msg.generic, replyRef);
			break;

		case CanMessageType::writeGpio:
			requestId = buf->msg.writeGpio.requestId;
			rslt = GpioPorts::HandleGpioWrite(buf->msg.writeGpio, replyRef);
			break;

		// LED strip commands
		case CanMessageType::m950Led:
			requestId = buf->msg.generic.requestId;
			rslt = LedStripManager::HandleM950Led(buf->msg.generic, replyRef, extra);
			break;

		case CanMessageType::writeLedStrip:
			requestId = buf->msg.generic.requestId;
			rslt = LedStripManager::HandleLedSetColours(buf->msg.generic, replyRef);
			break;

		case CanMessageType::testReport:
			requestId = buf->msg.generic.requestId;
			rslt = GenerateTestReport(buf->msg.generic, replyRef);
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

		case CanMessageType::m569p2:		// read/write smart driver register
			requestId = buf->msg.generic.requestId;
			rslt = ProcessM569Point2(buf->msg.generic, replyRef);
			break;

		case CanMessageType::m569p4:		// set torque mode
			requestId = buf->msg.generic.requestId;
# if SUPPORT_CLOSED_LOOP
			rslt = ClosedLoop::ProcessM569Point4(buf->msg.generic, replyRef);
# else
			rslt = GCodeResult::errorNotSupported;
# endif
			break;

		case CanMessageType::m569p6:
			requestId = buf->msg.generic.requestId;
# if SUPPORT_CLOSED_LOOP
			rslt = ClosedLoop::ProcessM569Point6(buf->msg.generic, replyRef);
# else
			rslt = GCodeResult::errorNotSupported;
# endif
			break;

		case CanMessageType::m569p7:
			requestId = buf->msg.generic.requestId;
			rslt = Platform::ProcessM569Point7(buf->msg.generic, replyRef);
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

		case CanMessageType::setInputShaping:
			requestId = buf->msg.setInputShaping.requestId;
			rslt = moveInstance->HandleInputShaping(buf->msg.setInputShaping, buf->dataLength, replyRef);
			break;

		case CanMessageType::setInputShapingNew:
			// This message is sent by main boards running 3.6.0. Ignore it but return OK to prevent them reporting CAN timeouts.
			requestId = buf->msg.setInputShapingNew.requestId;
			rslt = GCodeResult::ok;
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

		case CanMessageType::createInputMonitorNew:
			requestId = buf->msg.createInputMonitorNew.requestId;
			rslt = InputMonitor::Create(buf->msg.createInputMonitorNew, buf->dataLength, replyRef, extra);
			break;

		case CanMessageType::changeInputMonitorNew:
			requestId = buf->msg.changeInputMonitorNew.requestId;
			rslt = InputMonitor::Change(buf->msg.changeInputMonitorNew, replyRef, extra);
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

#if SUPPORT_CLOSED_LOOP
		case CanMessageType::startClosedLoopDataCollection:
			requestId = buf->msg.startClosedLoopDataCollection.requestId;
			rslt = ClosedLoop::ProcessM569Point5(buf->msg.startClosedLoopDataCollection, replyRef);
			break;
#endif

#if SUPPORT_LIS3DH
		case CanMessageType::accelerometerConfig:
			requestId = buf->msg.generic.requestId;
			rslt = AccelerometerHandler::ProcessConfigRequest(buf->msg.generic, replyRef);
			break;

		case CanMessageType::startAccelerometer:
			requestId = buf->msg.startAccelerometer.requestId;
			rslt = AccelerometerHandler::ProcessStartRequest(buf->msg.startAccelerometer, replyRef);
			break;
#endif
		default:
			// We received a message type that we don't recognise. If it's a broadcast, ignore it. If it's addressed to us, send a reply.
			if (buf->id.Src() != CanInterface::GetCanAddress())
			{
				CanMessageBuffer::Free(buf);
				return;
			}
			requestId = CanRequestIdAcceptAlways;
			reply.printf("Board %u received unknown msg type %u", CanInterface::GetCanAddress(), (unsigned int)buf->id.MsgType());
			rslt = GCodeResult::error;
			break;
		}

		if (requestId == CanRequestIdNoReplyNeeded)
		{
			CanMessageBuffer::Free(buf);					// no reply wanted so discard the response and free the buffer
		}
		else
		{
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
}

// End
