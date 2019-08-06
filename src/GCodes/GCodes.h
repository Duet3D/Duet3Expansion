/*
 * GCodes.h
 *
 *  Created on: 9 Sep 2018
 *      Author: David
 */

#ifndef SRC_GCODES_H_
#define SRC_GCODES_H_

#include "RepRapFirmware.h"

// Type for specifying which endstops we want to check
typedef AxesBitmap EndstopChecks;						// must be large enough to hold a bitmap of drive numbers or ZProbeActive
const EndstopChecks ZProbeActive = 1 << 31;				// must be distinct from 1 << (any drive number)
const EndstopChecks HomeAxes = 1 << 30;					// must be distinct from 1 << (any drive number)
//const EndstopChecks LogProbeChanges = 1 << 29;			// must be distinct from 1 << (any drive number)
const EndstopChecks UseSpecialEndstop = 1 << 28;		// must be distinct from 1 << (any drive number)

namespace GCodes
{
	void Init();
	void Spin();

	inline bool IsPaused() { return false; }
	inline size_t GetTotalAxes() { return 3; }
	inline size_t GetVisibleAxes() { return 3; }
	inline const char *GetAxisLetters() { return "XYZ"; }
	inline void MoveStoppedByZProbe() {}
}

#endif /* SRC_GCODES_H_ */
