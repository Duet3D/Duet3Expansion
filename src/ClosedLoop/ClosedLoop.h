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

namespace ClosedLoop
{
	extern ClosedLoopDriverMode currentMode;

	inline ClosedLoopDriverMode GetCurrentMode() { return currentMode; }
	GCodeResult ProcessM569Point1(const CanMessageGeneric& msg, const StringRef& reply);
}

#endif

#endif /* SRC_CLOSEDLOOP_CLOSEDLOOP_H_ */
