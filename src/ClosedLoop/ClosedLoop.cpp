/*
 * ClosedLoop.cpp
 *
 *  Created on: 9 Jun 2020
 *      Author: David
 */

#include "ClosedLoop.h"

#if SUPPORT_CLOSED_LOOP

#include <CanMessageGenericParser.h>

ClosedLoopDriverMode ClosedLoop::currentMode = ClosedLoopDriverMode::openLoop;

GCodeResult ClosedLoop::ProcessM569Point1(const CanMessageGeneric &msg, const StringRef &reply)
{
	CanMessageGenericParser parser(msg, M569Point1Params);
	uint8_t newMode;
	if (parser.GetUintParam('S', newMode))
	{
		//TODO
		return GCodeResult::errorNotSupported;
	}

	static constexpr const char * ModeNames[] = { "open loop", "rotary quadrature", "linear quadrature", "rotary AS5047", "rotary TLI5012" };
	const unsigned int index = (unsigned int)currentMode;
	reply.printf("Driver mode is %s", (index < ARRAY_SIZE(ModeNames)) ? ModeNames[index] : "undefined");
	return GCodeResult::ok;
}

#endif

// End
