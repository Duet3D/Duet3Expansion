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
#include "AS5047D.h"
#include "QuadratureEncoder.h"
#include "TLI5012B.h"

bool ClosedLoop::closedLoopEnabled = false;
SpiEncoder *ClosedLoop::encoder = nullptr;

void ClosedLoop::Init() noexcept
{
	ClockGen::Init();
	QuadratureEncoder::InitAttiny();
	SpiEncoder::Init();
}

EncoderType ClosedLoop::GetEncoderType() noexcept
{
	return (encoder == nullptr) ? EncoderType::none : encoder->GetType();
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
			if (temp != GetEncoderType().ToBaseType())
			{
				//TODO need to get a lock here in case there is any movement
				delete encoder;
				switch (temp)
				{
				case EncoderType::none:
				default:
					break;

				case EncoderType::as5047:
					encoder = new AS5047D(EncoderCsPin);
					break;

				case EncoderType::tli5012:
					encoder = new TLI5012B(EncoderCsPin);
					break;

				case EncoderType::linearQuadrature:
					encoder = new QuadratureEncoder(true);
					break;

				case EncoderType::rotaryQuadrature:
					encoder = new QuadratureEncoder(false);
					break;
				}
			}
		}
		else
		{
			reply.copy("Encoder type out of range");
			return GCodeResult::error;
		}
	}

	if (!seen)
	{
		reply.printf("Closed loop mode %s, encoder type %s", (closedLoopEnabled) ? "enabled" : "disabled", GetEncoderType().ToString());
	}
	return GCodeResult::ok;
}

void ClosedLoop::Diagnostics(const StringRef& reply) noexcept
{
	reply.printf("Encoder type %s", GetEncoderType().ToString());
	if (encoder != nullptr)
	{
		reply.catf(", position %" PRIi32, encoder->GetReading());
		encoder->AppendDiagnostics(reply);
	}
}

#endif

// End
