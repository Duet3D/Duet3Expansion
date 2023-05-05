/*
 * LedStrips.cpp
 *
 *  Created on: 5 May 2023
 *      Author: David
 */

#include "LedStrips.h"

// Configure an LED strip. If success and the strip does not require motion to be paused when sending data to the strip, set bit 0 of 'extra'.
GCodeResult LedStrips::HandleM950Led(const CanMessageGeneric &msg, const StringRef& reply, uint8_t &extra) noexcept
{
	reply.copy("LED strips not supported by this expansion board");
	return GCodeResult::error;
}

// Set the colours of a configured LED strip
GCodeResult LedStrips::HandleLedSetColours(const CanMessageGeneric &msg, const StringRef& reply) noexcept
{
	reply.copy("LED strips not supported by this expansion board");
	return GCodeResult::error;
}

// End
