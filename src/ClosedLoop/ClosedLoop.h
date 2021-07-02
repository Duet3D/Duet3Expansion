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
#include <Hardware/SharedSpiDevice.h>
#include <CanMessageFormats.h>
#include <General/NamedEnum.h>

class SpiEncoder;

namespace ClosedLoop
{
	void Init() noexcept;
	EncoderType GetEncoderType() noexcept;
	int32_t GetEncoderReading() noexcept;
	GCodeResult ProcessM569Point1(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	void Diagnostics(const StringRef& reply) noexcept;

	void EnableEncodersSpi() noexcept;
	void DisableEncodersSpi() noexcept;
	void TurnAttinyOff() noexcept;

	void TakeStep() noexcept;
	void SetStepDirection(bool) noexcept;
	bool GetClosedLoopEnabled() noexcept;
	bool SetClosedLoopEnabled(bool, const StringRef&) noexcept;
}

#endif

#endif /* SRC_CLOSEDLOOP_CLOSEDLOOP_H_ */
