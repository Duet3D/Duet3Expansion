/*
 * GCodes.h
 *
 *  Created on: 9 Sep 2018
 *      Author: David
 */

#ifndef SRC_GCODES_H_
#define SRC_GCODES_H_

#include "ReprapFirmware.h"

// Type for specifying which endstops we want to check
typedef AxesBitmap EndstopChecks;						// must be large enough to hold a bitmap of drive numbers or ZProbeActive
const EndstopChecks ZProbeActive = 1 << 31;				// must be distinct from 1 << (any drive number)
const EndstopChecks HomeAxes = 1 << 30;					// must be distinct from 1 << (any drive number)
const EndstopChecks LogProbeChanges = 1 << 29;			// must be distinct from 1 << (any drive number)
const EndstopChecks UseSpecialEndstop = 1 << 28;		// must be distinct from 1 << (any drive number)

namespace GCodes
{
	struct RawMove
	{
		float coords[DRIVES];											// new positions for the axes, amount of movement for the extruders
		float initialCoords[MaxAxes];									// the initial positions of the axes
		float feedRate;													// feed rate of this move
		float virtualExtruderPosition;									// the virtual extruder position at the start of this move
//		FilePosition filePos;											// offset in the file being printed at the start of reading this move
		float proportionLeft;											// what proportion of the entire move remains after this segment
		AxesBitmap xAxes;												// axes that X is mapped to
		AxesBitmap yAxes;												// axes that Y is mapped to
		EndstopChecks endStopsToCheck;									// endstops to check
#if SUPPORT_LASER || SUPPORT_IOBITS
		LaserPwmOrIoBits laserPwmOrIoBits;								// the laser PWM or port bit settings required
#endif
		uint8_t moveType;												// the S parameter from the G0 or G1 command, 0 for a normal move

		uint8_t isFirmwareRetraction : 1;								// true if this is a firmware retraction/un-retraction move
		uint8_t usePressureAdvance : 1;									// true if we want to us extruder pressure advance, if there is any extrusion
		uint8_t canPauseAfter : 1;										// true if we can pause just after this move and successfully restart
		uint8_t hasExtrusion : 1;										// true if the move includes extrusion - only valid if the move was set up by SetupMove
		uint8_t isCoordinated : 1;										// true if this is a coordinates move
		uint8_t usingStandardFeedrate : 1;								// true if this move uses the standard feed rate

		void SetDefaults();												// set up default values
	};

	void Init();
	void Spin();
	bool ReadMove(RawMove& m);											// Called by the Move class to get a movement set by the last G Code

	inline bool IsPaused() { return false; }
	inline size_t GetTotalAxes() { return 3; }
	inline size_t GetVisibleAxes() { return 3; }
	inline const char *GetAxisLetters() { return "XYZ"; }
	inline void MoveStoppedByZProbe() {}
}

#endif /* SRC_GCODES_H_ */
