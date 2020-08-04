/*
 * ClosedLoop.cpp
 *
 *  Created on: 9 Jun 2020
 *      Author: David
 */

#include "ClosedLoop.h"

#if SUPPORT_CLOSED_LOOP

#include <CanMessageGenericParser.h>
#include "ClockGen.h"
#include "SpiEncoder.h"

bool ClosedLoop::closedLoopEnabled = false;
EncoderType ClosedLoop::encoderType = EncoderType::none;

void ClosedLoop::Init() noexcept
{
	ClockGen::Init();
	SpiEncoder::Init();
}

GCodeResult ClosedLoop::ProcessM569Point1(const CanMessageGeneric &msg, const StringRef &reply) noexcept
{
	CanMessageGenericParser parser(msg, M569Point1Params);
	bool seen = false;
	uint8_t temp;

	// Check closed loop enable/disable
	if (parser.GetUintParam('S', temp))
	{
		seen = true;
		//TODO
		if (temp != 0)
		{
			reply.copy("Closed loop mode not supported yet");
			return GCodeResult::error;
		}
	}
	if (parser.GetUintParam('T', temp))
	{
		seen = true;
		if (temp < EncoderType::NumValues)
		{
			encoderType.Assign(temp);
			//TODO change the encoder type
		}
		else
		{
			reply.copy("Encoder type out of range");
			return GCodeResult::error;
		}
	}

	if (!seen)
	{
		reply.printf("Closed loop mode %s, encoder type %s", (closedLoopEnabled) ? "enabled" : "disabled", encoderType.ToString());
	}
	return GCodeResult::ok;
}

#endif

// End
