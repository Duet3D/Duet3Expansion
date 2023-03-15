/*
 * DriveMovement.h
 *
 *  Created on: 17 Jan 2015
 *      Author: David
 */

#ifndef DRIVEMOVEMENT_H_
#define DRIVEMOVEMENT_H_

#include "RepRapFirmware.h"

#if SUPPORT_DRIVERS

#include <Platform/Tasks.h>
#include "MoveSegment.h"

#if SUPPORT_CLOSED_LOOP

# include <ClosedLoop/ClosedLoop.h>

// Struct to pass data back to the ClosedLoop module
struct MotionParameters
{
	float position;
	float speed;
	float acceleration;
};

#endif

class LinearDeltaKinematics;
class PrepParams;

enum class DMState : uint8_t
{
	idle = 0,
	stepError,

	// All higher values are various states of motion
	firstMotionState,
	cartAccel = firstMotionState,					// linear accelerating motion
	cartLinear,										// linear steady speed
	cartDecelNoReverse,
	cartDecelForwardsReversing,						// linear decelerating motion, expect reversal
	cartDecelReverse,								// linear decelerating motion, reversed

#if SUPPORT_DELTA_MOVEMENT
	deltaNormal,									// moving forwards without reversing in this segment, or in reverse
	deltaForwardsReversing,							// moving forwards to start with, reversing before the end of this segment
#endif
};

// This class describes a single movement of one drive
class DriveMovement
{
public:
	friend class DDA;

	DriveMovement() noexcept { };

	void* operator new(size_t count) noexcept { return Tasks::AllocPermanent(count); }
	void* operator new(size_t count, std::align_val_t align) noexcept { return Tasks::AllocPermanent(count, align); }
	void operator delete(void* ptr) noexcept {}
	void operator delete(void* ptr, std::align_val_t align) noexcept {}

	bool CalcNextStepTime(const DDA &dda) noexcept SPEED_CRITICAL;
	bool PrepareCartesianAxis(const DDA& dda, const PrepParams& params) noexcept SPEED_CRITICAL;
#if SUPPORT_DELTA_MOVEMENT
	bool PrepareDeltaAxis(const DDA& dda, const PrepParams& params) noexcept SPEED_CRITICAL;
#endif
	bool PrepareExtruder(const DDA& dda, const PrepParams& params, float signedEffStepsPerMm) noexcept SPEED_CRITICAL;

	void DebugPrint() const noexcept;
	int32_t GetNetStepsTaken() const noexcept;

#if HAS_SMART_DRIVERS
	uint32_t GetStepInterval(uint32_t msShift) const noexcept;	// Get the current full step interval for this axis or extruder
#endif

#if SUPPORT_CLOSED_LOOP
	void AdjustNetSteps(float proportion) noexcept;

	// Get the current position relative to the start of this move, speed and acceleration. Units are microsteps and step clocks.
	// Interrupts are disabled on entry and must remain disabled. Segments are advanced as necessary.
	void GetCurrentMotion(float topSpeed, uint32_t ticksSinceStart, MotionParameters& mParams) noexcept;

	// This is like getCurrentMotion but it just returns the distance and doesn't start new segments
	int32_t GetNetStepsTakenClosedLoop(float topSpeed, int32_t ticksSinceStart) const noexcept;
#endif

private:
	bool CalcNextStepTimeFull(const DDA &dda) noexcept SPEED_CRITICAL;
	bool NewCartesianSegment() noexcept SPEED_CRITICAL;
	bool NewExtruderSegment() noexcept SPEED_CRITICAL;
#if SUPPORT_DELTA_MOVEMENT
	bool NewDeltaSegment(const DDA& dda) noexcept SPEED_CRITICAL;
#endif

	void CheckDirection(bool reversed) noexcept;

	static DriveMovement *freeList;
	static int numFree;
	static int minFree;

	// Parameters common to Cartesian, delta and extruder moves

#if !SINGLE_DRIVER
	DriveMovement *nextDM;								// link to next DM that needs a step
#endif

	const MoveSegment *currentSegment;

	DMState state;										// whether this is active or not
	uint8_t drive;										// the drive that this DM controls
	uint8_t direction : 1,								// true=forwards, false=backwards
			directionChanged : 1,						// set by CalcNextStepTime if the direction is changed
			directionReversed : 1,						// true if we have reversed the requested motion direction because of pressure advance
			isDelta : 1,								// true if this motor is executing a delta tower move
			isExtruder : 1,								// true if this DM is for an extruder (only matters if !isDelta)
					: 1,								// padding to make the next field last
			stepsTakenThisSegment : 2;					// how many steps we have taken this phase, counts from 0 to 2. Last field in the byte so that we can increment it efficiently.
	uint8_t stepsTillRecalc;							// how soon we need to recalculate

	int32_t totalSteps;									// total number of steps for this move
#if SUPPORT_CLOSED_LOOP
	int32_t netSteps;
#endif

	// These values change as the step is executed, except for reverseStartStep
	int32_t nextStep;									// number of steps already done
	int32_t segmentStepLimit;							// the first step number of the next phase, or the reverse start step if smaller
	int32_t reverseStartStep;							// the step number for which we need to reverse direction due to pressure advance or delta movement
	uint32_t nextStepTime;								// how many clocks after the start of this move the next step is due
	uint32_t stepInterval;								// how many clocks between steps

	float distanceSoFar;								// the accumulated distance at the end of the current move segment
	float timeSoFar;									// the accumulated taken for this current DDA at the end of the current move segment
	float pA, pB, pC;									// the move parameters for the current move segment. pA is not used when performing a move at constant speed.

	// Parameters unique to a style of move (Cartesian, delta or extruder). Currently, extruders and Cartesian moves use the same parameters.
	union
	{
#if SUPPORT_DELTA_MOVEMENT
		struct DeltaParameters							// Parameters for delta movement
		{
			// The following don't depend on how the move is executed, so they could be set up in Init() if we use fixed acceleration/deceleration
			float fTwoA;
			float fTwoB;
			float h0MinusZ0;							// the height subtended by the rod at the start of the move
			float fDSquaredMinusAsquaredMinusBsquaredTimesSsquared;
			float fHmz0s;								// the starting height less the starting Z height, multiplied by the Z movement fraction (can go negative)
			float fMinusAaPlusBbTimesS;
			float reverseStartDistance;					// the overall move distance at which movement reversal occurs
		} delta;
#endif

		struct CartesianParameters						// Parameters for Cartesian and extruder movement, including extruder pressure advance
		{
			float pressureAdvanceK;						// how much pressure advance is applied to this move
			float effectiveStepsPerMm;					// the steps/mm multiplied by the movement fraction
			float effectiveMmPerStep;					// reciprocal of [the steps/mm multiplied by the movement fraction]
			float extrusionBroughtForwards;				// the amount of extrusion brought forwards from previous moves. Only needed for debug output.
		} cart;
	} mp;
};

// Calculate and store the time since the start of the move when the next step for the specified DriveMovement is due.
// Return true if there are more steps to do. When finished, leave nextStep == totalSteps + 1 and state == DMState::idle.
// This is also used for extruders on delta machines.
// We inline this part to speed things up when we are doing double/quad/octal stepping.
inline bool DriveMovement::CalcNextStepTime(const DDA &dda) noexcept
{
	++nextStep;
	if (nextStep <= totalSteps || isExtruder)
	{
		if (stepsTillRecalc != 0)
		{
			--stepsTillRecalc;			// we are doing double/quad/octal stepping
			nextStepTime += stepInterval;
			return true;
		}
		if (CalcNextStepTimeFull(dda))
		{
			return true;
		}
	}

	state = DMState::idle;
	return false;
}

// Return the number of net steps already taken for the move in the forwards direction.
// We have already taken nextSteps - 1 steps, unless nextStep is zero.
// This is called only when the driver is in open loop mode.
inline int32_t DriveMovement::GetNetStepsTaken() const noexcept
{
	int32_t netStepsTaken;
	if (nextStep < reverseStartStep || reverseStartStep > totalSteps)				// if no reverse phase, or not started it yet
	{
		netStepsTaken = (nextStep == 0) ? 0 : (int32_t)nextStep - 1;
	}
	else
	{
		netStepsTaken = (int32_t)nextStep - (int32_t)(2 * reverseStartStep) + 1;	// allowing for direction having changed
	}
	return (direction) ? netStepsTaken : -netStepsTaken;
}

inline void DriveMovement::CheckDirection(bool reversed) noexcept
{
	if (reversed != directionReversed)
	{
		directionReversed = !directionReversed;										// this can be done by an xor so hopefully more efficient than assignment
		direction = !direction;
		directionChanged = true;
	}
}

#if HAS_SMART_DRIVERS

// Get the current full step interval for this axis or extruder
inline uint32_t DriveMovement::GetStepInterval(uint32_t microstepShift) const noexcept
{
	return ((nextStep >> microstepShift) != 0)		// if at least 1 full step done
		? stepInterval << microstepShift			// return the interval between steps converted to full steps
			: 0;
}

#endif

#if SUPPORT_CLOSED_LOOP

inline void DriveMovement::AdjustNetSteps(float proportion) noexcept
{
	netSteps = lrintf((float)netSteps * proportion);
}

// Get the current position relative to the start of this move, speed and acceleration. Units are microsteps and step clocks.
// Interrupts are disabled on entry and must remain disabled.
inline void DriveMovement::GetCurrentMotion(float topSpeed, uint32_t ticksSinceStart, MotionParameters& mParams) noexcept
{
	const MoveSegment *ms = currentSegment;
	if (ms == nullptr)
	{
		// Drive was not commanded to move
		mParams.position = mParams.speed = mParams.acceleration = 0.0;
		return;
	}

	for (;;)
	{
		const float segTimeRemaining = timeSoFar - (float)ticksSinceStart;
		if (segTimeRemaining >= 0.0 || ms->GetNext() == nullptr)
		{
			// The current move segment is still in progress, or it is the last move segment and it has only just finished
			const float multiplier = (direction != directionReversed) ? mp.cart.effectiveStepsPerMm : -mp.cart.effectiveStepsPerMm;
			if (ms->IsLinear())
			{
				mParams.position = (distanceSoFar - segTimeRemaining * topSpeed) * multiplier;
				mParams.speed = topSpeed * mp.cart.effectiveStepsPerMm;
				mParams.acceleration = 0.0;
			}
			else
			{
				//TODO are the following correct for extruders with pressure advance?
				const float segmentEndSpeed = ms->GetNonlinearEndSpeed(mp.cart.pressureAdvanceK);
				const float speedIncreaseRemaining = ms->GetAcceleration() * segTimeRemaining;
				mParams.position = (distanceSoFar - ((segmentEndSpeed - 0.5 * speedIncreaseRemaining) * segTimeRemaining)) * multiplier;
				mParams.speed = (segmentEndSpeed - speedIncreaseRemaining) * multiplier;
				mParams.acceleration = ms->GetAcceleration() * multiplier;
			}
			return;
		}
		currentSegment = ms = ms->GetNext();
		if (isExtruder)
		{
			(void)NewExtruderSegment();
		}
		else
		{
			(void)NewCartesianSegment();
		}
	}
}

// This is like getCurrentMotion but it just returns the distance and doesn't start new segments
inline int32_t DriveMovement::GetNetStepsTakenClosedLoop(float topSpeed, int32_t ticksSinceStart) const noexcept
{
	const MoveSegment *const ms = currentSegment;
	float ret;
	if (ms == nullptr)
	{
		ret = distanceSoFar;
	}
	else
	{
		const float segTimeRemaining = timeSoFar - (float)ticksSinceStart;
		if (segTimeRemaining <= 0.0)
		{
			ret = distanceSoFar;
		}
		else if (ms->IsLinear())
		{
			ret = distanceSoFar - (segTimeRemaining * topSpeed);
		}
		else
		{
			//TODO are the following correct for extruders with pressure advance?
			const float segmentEndSpeed = ms->GetNonlinearEndSpeed(mp.cart.pressureAdvanceK);
			const float speedIncreaseRemaining = ms->GetAcceleration() * segTimeRemaining;
			ret = distanceSoFar - ((segmentEndSpeed - 0.5 * speedIncreaseRemaining) * segTimeRemaining);
		}
	}
	const float multiplier = (direction != directionReversed) ? mp.cart.effectiveStepsPerMm : -mp.cart.effectiveStepsPerMm;
	return lrintf(ret * multiplier);
}

#endif	// SUPPORT_CLOSED_LOOP

#endif	// SUPPORT_DRIVERS

#endif /* DRIVEMOVEMENT_H_ */
