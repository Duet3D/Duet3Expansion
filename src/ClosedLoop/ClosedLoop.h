/*
 * ClosedLoop.h
 *
 *  Created on: 9 Jun 2020
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_CLOSEDLOOP_H_
#define SRC_CLOSEDLOOP_CLOSEDLOOP_H_

#include <RepRapFirmware.h>

#if SUPPORT_CLOSED_LOOP

#include <GCodes/GCodeResult.h>
#include <CanMessageFormats.h>
#include <General/NamedEnum.h>

class SpiEncoder;

namespace ClosedLoop
{
	extern bool closedLoopEnabled;
	extern SpiEncoder *encoder;

	void Init() noexcept;
	EncoderType GetEncoderType() noexcept;
	GCodeResult ProcessM569Point1(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	void Diagnostics(const StringRef& reply) noexcept;
}

#endif

#endif /* SRC_CLOSEDLOOP_CLOSEDLOOP_H_ */
