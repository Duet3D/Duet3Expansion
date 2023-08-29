/*
 * LedStrips.cpp
 *
 *  Created on: 5 May 2023
 *      Author: David
 */

#include "LedStripManager.h"

#if SUPPORT_LED_STRIPS
# include "NeoPixelLedStrip.h"
# include <CAN/CanInterface.h>
# include <RTOSIface/RTOSIface.h>

namespace LedStripManager
{
	ReadWriteLock ledLock;
	LedStripBase *strips[MaxLedStrips] = { 0 };
}
#endif

// Configure an LED strip. If success and the strip does not require motion to be paused when sending data to the strip, set bit 0 of 'extra'.
GCodeResult LedStripManager::HandleM950Led(const CanMessageGeneric &msg, const StringRef& reply, uint8_t &extra) noexcept
{
#if SUPPORT_LED_STRIPS
	// Get and validate the port number
	CanMessageGenericParser parser(msg, M950LedParams);
	uint16_t stripNumber;
	if (!parser.GetUintParam('E', stripNumber))
	{
		reply.copy("Missing strip number parameter in M950Led message");
		return GCodeResult::remoteInternalError;
	}
	if (stripNumber >= MaxLedStrips)
	{
		reply.printf("LED strip number %u is too high for expansion board %u", stripNumber, CanInterface::GetCanAddress());
		return GCodeResult::error;
	}

	uint8_t rawStripType;
	const LedStripType t = (parser.GetUintParam('T', rawStripType)) ? (LedStripType)rawStripType : DefaultLedStripType;

	LedStripBase*& slot = strips[stripNumber];
	GCodeResult rslt;
	WriteLocker lock(ledLock);
	if (parser.HasParameter('C'))
	{
		// Configuring a new strip
		DeleteObject(slot);

		LedStripBase *newStrip = nullptr;
		switch (t.RawValue())
		{
		case LedStripType::NeoPixel_RGB:
			newStrip = new NeoPixelLedStrip(false);
			break;

		case LedStripType::NeoPixel_RGBW:
			newStrip = new NeoPixelLedStrip(true);
			break;

		default:
			reply.copy("Unsupported LED strip type");
			return GCodeResult::error;
		}

		rslt = newStrip->Configure(parser, reply, extra);
		if (rslt <= GCodeResult::warning)
		{
			slot = newStrip;
		}
		else
		{
			delete newStrip;
		}
	}
	else
	{
		// Reconfiguring or reporting on an existing strip
		rslt = slot->Configure(parser, reply, extra);
	}
	return rslt;
#else
	reply.copy("LED strips not supported by this expansion board");
	return GCodeResult::error;
#endif
}

// Set the colours of a configured LED strip
GCodeResult LedStripManager::HandleLedSetColours(const CanMessageGeneric &msg, const StringRef& reply) noexcept
{
#if SUPPORT_LED_STRIPS
	CanMessageGenericParser parser(msg, M150Params);
	uint16_t stripNumber = 0;								// strip number may be omitted, defaults to 0
	parser.GetUintParam('E', stripNumber);
	if (stripNumber >= MaxLedStrips)
	{
		reply.printf("LED strip number %u is too high for expansion board %u", stripNumber, CanInterface::GetCanAddress());
		return GCodeResult::error;
	}

	{
		ReadLocker locker(ledLock);
		LedStripBase *const strip = strips[stripNumber];
		if (strip != nullptr)
		{
			return strip->HandleM150(parser, reply);
		}
	}

	reply.printf("Board %u does not have LED strip #%u", CanInterface::GetCanAddress(), stripNumber);
	return GCodeResult::error;
#else
	reply.copy("LED strips not supported by this expansion board");
	return GCodeResult::error;
#endif
}

// End
