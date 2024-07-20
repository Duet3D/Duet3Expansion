/*
 * InputShaper.cpp
 *
 *  Created on: 20 Feb 2021
 *      Author: David
 */

#include "AxisShaper.h"

#if SUPPORT_DRIVERS

#include <CanMessageFormats.h>

AxisShaper::AxisShaper() noexcept
	: numImpulses(1)
{
	coefficients[0] = 1.0;
	delays[0] = 0;
}

// Handle a request from the master board to set input shaping parameters
GCodeResult AxisShaper::EutSetInputShaping(const CanMessageSetInputShapingNew& msg, size_t dataLength, const StringRef& reply) noexcept
{
	if (msg.numImpulses <= MaxImpulses && dataLength >= msg.GetActualDataLength())
	{
		numImpulses = msg.numImpulses;
		for (size_t i = 0; i < numImpulses; ++i)
		{
			coefficients[i] = msg.impulses[i].coefficient;
			delays[i] = msg.impulses[i].delay;
		}
		return GCodeResult::ok;
	}
	return GCodeResult::error;
}

#endif

// End
