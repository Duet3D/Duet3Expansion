/*
 * EndstopDefs.h
 *
 *  Created on: 5 Apr 2019
 *      Author: David
 */

#ifndef SRC_ENDSTOPS_ENDSTOPDEFS_H_
#define SRC_ENDSTOPS_ENDSTOPDEFS_H_

// Forward declarations
class EndstopOrZProbe;
class Endstop;
class ZProbe;

// Type of a driver identifier
struct DriverId
{
	uint8_t localDriver;

	void SetFromBinary(uint32_t val)
	{
		localDriver = (uint8_t)val;
	}

	void SetLocal(unsigned int driver)
	{
		localDriver = (uint8_t)driver;
	}

	void Clear() { localDriver = 0; }

	bool IsLocal() const { return true; }
	bool IsRemote() const { return false; }
};

// Actions to take when an endstop is triggered. Note, these values are ordered!
enum class EndstopHitAction : uint8_t
{
	none = 0,						// don't stop anything
	reduceSpeed = 1,				// reduce speed because an endstop or Z probe is close to triggering
	stopDriver = 2,					// stop a single motor driver
	stopAxis = 3,					// stop all drivers for an axis
	stopAll = 4						// stop movement completely
};

// Struct to return info about what endstop has been triggered and what to do about it
struct EndstopHitDetails
{
	EndstopHitDetails() : action((uint32_t)EndstopHitAction::none), internalUse(0), axis(0), setAxisLow(false), setAxisHigh(false)
	{
		driver.Clear();
	}

	void SetAction(EndstopHitAction a) { action = (uint32_t)a; }
	EndstopHitAction GetAction() const { return (EndstopHitAction)action; }

	uint16_t action : 4,			// an EndstopHitAction
			 internalUse : 4,		// used to pass data between CheckTriggered() and Acknowledge()
			 axis : 4,				// which axis to stop if the action is stopAxis, and which axis to set the position of if setAxisPos is true
			 setAxisLow : 1,		// whether or not to set the axis position to its min
			 setAxisHigh : 1,		// whether or not to set the axis position to its max
			 isZProbe : 1;			// whether this is a Z probe
	DriverId driver;
};

// The values of the following enumeration must tally with the X,Y,... parameters for the M574 command
enum class EndStopPosition : unsigned int
{
	noEndStop = 0,
	lowEndStop = 1,
	highEndStop = 2,
	numPositions = 3
};

// Type of an endstop input - values must tally with the M574 command S parameter
enum class EndStopInputType : unsigned int
{
	activeLow = 0,
	activeHigh = 1,
	zProbeAsEndstop = 2,
	motorStallAny = 3,
	motorStallIndividual = 4,
	numInputTypes = 5
};

// This is used as the return type of function Stopped.
// Note the ordering: we need more-stopped-value > less-stopped-value
enum class EndStopHit
{
  noStop = 0,		// no endstop hit
  nearStop = 1,		// approaching Z-probe threshold
  atStop = 2
};

enum class ZProbeType : uint8_t
{
	none = 0,
	analog = 1,
	dumbModulated = 2,
	alternateAnalog = 3,
	endstopSwitch_obsolete = 4,
	digital = 5,
	e1Switch_obsolete = 6,
	zSwitch_obsolete = 7,
	unfilteredDigital = 8,
	blTouch = 9,
	zMotorStall = 10,
	numTypes = 11					// must be 1 higher than the last type
};

#endif /* SRC_ENDSTOPS_ENDSTOPDEFS_H_ */
