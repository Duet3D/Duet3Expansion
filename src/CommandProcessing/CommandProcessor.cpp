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
#include <Platform.h>
#include <Version.h>

#if SUPPORT_TMC22xx
# include "Movement/StepperDrivers/TMC22xx.h"
#endif
#if SUPPORT_TMC51xx
# include "Movement/StepperDrivers/TMC51xx.h"
#endif

static GCodeResult SetMotorCurrents(const CanMessageMultipleDrivesRequest& msg, const StringRef& reply)
{
	//TODO check message is long enough for the number of drivers specified
	const uint16_t *p = msg.values;
	for (unsigned int driver = 0; driver < NumDrivers; ++driver)
	{
		if (IsBitSet(msg.driversToUpdate, driver))
		{
			SmartDrivers::SetCurrent(driver, (float)*p);		//TODO avoid the int->float->int conversion
			++p;
		}
	}
	return GCodeResult::ok;
}

static GCodeResult SetStandstillCurrentFactor(const CanMessageMultipleDrivesRequest& msg, const StringRef& reply)
{
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
}

static GCodeResult SetMicrostepping(const CanMessageMultipleDrivesRequest& msg, const StringRef& reply)
{
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

void CommandProcessor::Spin()
{
	CanMessageBuffer *buf = CanInterface::GetCanCommand();
	if (buf != nullptr)
	{
		String<CanMessageStandardReply::MaxTextLength> reply;
		const StringRef& replyRef = reply.GetRef();
		const CanMessageType id = buf->id.MsgType();
		GCodeResult rslt;
		CanRequestId requestId;

		switch (id)
		{
		case CanMessageType::m122:
			requestId = buf->msg.diagnostics.requestId;
			reply.printf("Board %s firmware %s", BoardTypeName, FirmwareVersion);
			rslt = GCodeResult::ok;
			break;

		case CanMessageType::updateHeaterModel:
			requestId = buf->msg.heaterModel.requestId;
			rslt = Heat::ProcessM307(buf->msg.heaterModel, replyRef);
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
			reply.copy("GPIO configuration not implemented");
			rslt = GCodeResult::error;
			break;

		case CanMessageType::setMotorCurrents:
			requestId = buf->msg.multipleDrivesRequest.requestId;
			rslt = SetMotorCurrents(buf->msg.multipleDrivesRequest, replyRef);
			break;

		case CanMessageType::m569:
			requestId = buf->msg.generic.requestId;
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

		case CanMessageType::m915:
		case CanMessageType::setDriverStates:
		default:
			requestId = CanRequestIdAcceptAlways;
			reply.printf("Unknown message type %u", (unsigned int)buf->id.MsgType());
			rslt = GCodeResult::error;
			break;
		}

		// Re-use the message buffer to send the reply
		const CanAddress srcAddress = buf->id.Src();
		CanMessageStandardReply *msg = buf->SetupResponseMessage<CanMessageStandardReply>(requestId, CanInterface::GetCanAddress(), srcAddress);
		msg->resultCode = (uint16_t)rslt;
		size_t textLength = min<size_t>(reply.strlen(), CanMessageStandardReply::MaxTextLength);
		memcpy(msg->text, reply.c_str(), textLength);
		if (textLength < ARRAY_SIZE(msg->text))
		{
			msg->text[textLength] = 0;
			++textLength;
		}
		buf->dataLength = msg->GetActualDataLength(textLength);
		CanInterface::Send(buf);
	}
}

// End
