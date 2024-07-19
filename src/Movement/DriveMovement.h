/*
 * DriveMovement.h
 *
 *  Created on: 17 Jan 2015
 *      Author: David
 */

#ifndef DRIVEMOVEMENT_H_
#define DRIVEMOVEMENT_H_

#include <RepRapFirmware.h>

#if SUPPORT_DRIVERS

#include <Platform/Tasks.h>
#include "MoveSegment.h"
#include "ExtruderShaper.h"

#if SUPPORT_CLOSED_LOOP
# include <ClosedLoop/ClosedLoop.h>
#endif

class LinearDeltaKinematics;
class PrepParams;

enum class DMState : uint8_t
{
	idle = 0,
	stepError,

	// All higher values are various states of motion and require interrupts to be generated
	firstMotionState,
	starting = firstMotionState,					// interrupt scheduled for when the move should start
	ending,											// interrupt scheduled for when the move should end
	cartAccel,										// linear accelerating motion
	cartLinear,										// linear steady speed
	cartDecelNoReverse,
	cartDecelForwardsReversing,						// linear decelerating motion, expect reversal
	cartDecelReverse,								// linear decelerating motion, reversed
};

// This class describes a single movement of one drive
class DriveMovement
{
public:
	friend class Move;

	DriveMovement() noexcept { }
	void Init(size_t drv) noexcept;

	bool CalcNextStepTime(uint32_t now) noexcept SPEED_CRITICAL;

	void DebugPrint() const noexcept;
	int32_t GetCurrentMotorPosition() const noexcept { return currentMotorPosition; }
	void StopDriverFromRemote() noexcept;
	int32_t GetNetStepsTaken() const noexcept;							// return the number of steps taken in the current segment
	void SetMotorPosition(int32_t pos) noexcept;
	bool MotionPending() const noexcept { return segments != nullptr; }
	bool IsPrintingExtruderMovement() const noexcept;					// returns true if this is an extruder executing a printing move
	bool CheckingEndstops() const noexcept;								// returns true when executing a move that checks endstops or Z probe

	void AddSegment(uint32_t startTime, uint32_t duration, motioncalc_t distance, motioncalc_t a, MovementFlags moveFlags) noexcept;
	void SetAsExtruder(bool p_isExtruder) noexcept { isExtruder = p_isExtruder; }

#if HAS_SMART_DRIVERS
	uint32_t GetStepInterval(uint32_t microstepShift) const noexcept;	// Get the current full step interval for this axis or extruder
#endif

#if SUPPORT_CLOSED_LOOP
	// Get the current position relative to the start of this move, speed and acceleration. Units are microsteps and step clocks.
	// Interrupts are disabled on entry and must remain disabled. Segments are advanced as necessary.
	void GetCurrentMotion(const DDA& dda, uint32_t ticksSinceStart, MotionParameters& mParams) noexcept;

	// This is like getCurrentMotion but it just returns the distance and doesn't start new segments
	int32_t GetNetStepsTakenClosedLoop(float topSpeed, int32_t ticksSinceStart) const noexcept;
#endif

	static int32_t GetAndClearMaxStepsLate() noexcept;

private:
	bool CalcNextStepTimeFull(uint32_t now) noexcept SPEED_CRITICAL;
	MoveSegment *NewSegment(uint32_t now) noexcept SPEED_CRITICAL;
	bool ScheduleFirstSegment() noexcept;

	void ReleaseSegments() noexcept;					// release the list of segments and set it to nullptr
	bool LogStepError(uint8_t type) noexcept;			// tell the Move class that we had a step error

	static int32_t maxStepsLate;

	// Parameters common to Cartesian, delta and extruder moves

#if !SINGLE_DRIVER
	DriveMovement *nextDM;								// link to next DM that needs a step
#endif

	MoveSegment *volatile segments;						// pointer to the segment list for this driver
	ExtruderShaper extruderShaper;						// pressure advance control

	DMState state;										// whether this is active or not
	uint8_t drive;										// the drive that this DM controls
	uint8_t direction : 1,								// true=forwards, false=backwards
			directionChanged : 1,						// set by CalcNextStepTime if the direction is changed
			isExtruder : 1,								// true if this DM is for an extruder (only matters if !isDelta)
			stepErrorType : 3,							// records what type of step error we had
			stepsTakenThisSegment : 2;					// how many steps we have taken this phase, counts from 0 to 2. Last field in the byte so that we can increment it efficiently.
	uint8_t stepsTillRecalc;							// how soon we need to recalculate

	int32_t netStepsThisSegment;						// the (signed) net number of steps in the current segment
	int32_t segmentStepLimit;							// the first step number of the next phase, or the reverse start step if smaller
	int32_t reverseStartStep;							// the step number for which we need to reverse direction due to pressure advance or delta movement
	motioncalc_t q, t0, p;								// the movement parameters of the current segment
	MovementFlags segmentFlags;							// whether this segment checks endstops etc.
	motioncalc_t distanceCarriedForwards;				// the residual distance in microsteps (less than one) that was pending at the end of the previous segment

	int32_t currentMotorPosition;						// the current motor position in microsteps
	int32_t positionAtSegmentStart;						// the value of currentMotorPosition at the start of the current segment

	// These values change as the step is executed, except for reverseStartStep
	int32_t nextStep;									// number of steps already done. For extruders this gets reset to the net steps already done at the start of each segment, so it can go negative.
	uint32_t nextStepTime;								// how many clocks after the start of this move the next step is due
	uint32_t stepInterval;								// how many clocks between steps

	std::atomic<int32_t> movementAccumulator;			// the accumulated movement in microsteps since GetAccumulatedMovement was last called. Only used for extruders.
	uint32_t extruderPrintingSince;						// the millis ticks when this extruder started doing printing moves

	bool extruderPrinting;								// true if this is an extruder and the most recent segment started was a printing move
};

// Calculate and store the time since the start of the move when the next step for the specified DriveMovement is due.
// Return true if there are more steps to do. When finished, leave nextStep == totalSteps + 1 and state == DMState::idle.
// We inline this part to speed things up when we are doing double/quad/octal stepping.
inline bool DriveMovement::CalcNextStepTime(uint32_t now) noexcept
{
	// We have just taken a step, so update the current motor position
	const int32_t adjustment = (int32_t)(direction << 1) - 1;	// to avoid a conditional jump, calculate +1 or -1 according to direction
	currentMotorPosition += adjustment;					// adjust the current position

	++nextStep;
	if (stepsTillRecalc != 0)
	{
		--stepsTillRecalc;								// we are doing double/quad/octal stepping
		nextStepTime += stepInterval;
#ifdef DUET3_MB6HC										// we need to increase the minimum step pulse length to be long enough for the TMC5160
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
		asm volatile("nop");
#endif
		return true;
	}
	return CalcNextStepTimeFull(now);
}

// Return true if this is an extruder executing a printing move
// Call must disable interrupts before calling this
inline bool DriveMovement::IsPrintingExtruderMovement() const noexcept
{
	return !segmentFlags.nonPrintingMove;
}

inline int32_t DriveMovement::GetAndClearMaxStepsLate() noexcept
{
	const int32_t ret = maxStepsLate;
	maxStepsLate = 0;
	return ret;
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
		const float timeSinceMoveStart = (float)ticksSinceStart;
		const float segTimeRemaining = timeSoFar - timeSinceMoveStart;
		if (segTimeRemaining <= 0.0)
		{
			ret = distanceSoFar;
		}
		else if (ms->IsLinear())
		{
			ret = (timeSinceMoveStart - pB) * topSpeed;
		}
		else
		{
			ret = 0.5 * ms->GetAcceleration() * (fsquare(timeSinceMoveStart - pB) - pA);
		}
	}
	const float multiplier = (direction != directionReversed) ? mp.cart.effectiveStepsPerMm : -mp.cart.effectiveStepsPerMm;
	return lrintf(ret * multiplier);
}

#endif	// SUPPORT_CLOSED_LOOP

#endif	// SUPPORT_DRIVERS

#endif /* DRIVEMOVEMENT_H_ */
