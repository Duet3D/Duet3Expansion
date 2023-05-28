/*
 * LedStrips.h
 *
 *  Created on: 5 May 2023
 *      Author: David
 */

#ifndef SRC_LEDSTRIPS_LEDSTRIPS_H_
#define SRC_LEDSTRIPS_LEDSTRIPS_H_

#include <RepRapFirmware.h>
#include <CanMessageFormats.h>
#include <CanMessageGenericTables.h>
#include <CanMessageGenericParser.h>

namespace LedStripManager
{
	GCodeResult HandleM950Led(const CanMessageGeneric &msg, const StringRef& reply, uint8_t &extra) noexcept;
	GCodeResult HandleLedSetColours(const CanMessageGeneric &msg, const StringRef& reply) noexcept;
}

#endif /* SRC_LEDSTRIPS_LEDSTRIPS_H_ */
