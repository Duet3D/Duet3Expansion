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
class DDA;

#define DM_USE_FPU			(__FPU_USED)
#define ROUND_TO_NEAREST	(0)			// 1 for round to nearest (as used in 1.20beta10), 0 for round down (as used prior to 1.20beta10)

// Rounding functions, to improve code clarity. Also allows a quick switch between round-to-nearest and round down in the movement code.
inline uint32_t roundU32(float f) noexcept
{
#if ROUND_TO_NEAREST
	return (uint32_t)lrintf(f);
#else
	return (uint32_t)f;
#endif
}

inline uint32_t roundU32(double d) noexcept
{
#if ROUND_TO_NEAREST
	return lrint(d);
#else
	return (uint32_t)d;
#endif
}

inline int32_t roundS32(float f) noexcept
{
#if ROUND_TO_NEAREST
	return lrintf(f);
#else
	return (int32_t)f;
#endif
}

inline int32_t roundS32(double d) noexcept
{
#if ROUND_TO_NEAREST
	return lrint(d);
#else
	return (int32_t)d;
#endif
}

inline uint64_t roundU64(float f) noexcept
{
#if ROUND_TO_NEAREST
	return (uint64_t)llrintf(f);
#else
	return (uint64_t)f;
#endif
}

inline uint64_t roundU64(double d) noexcept
{
#if ROUND_TO_NEAREST
	return (uint64_t)llrint(d);
#else
	return (uint64_t)d;
#endif
}

inline int64_t roundS64(float f) noexcept
{
#if ROUND_TO_NEAREST
	return llrintf(f);
#else
	return (int64_t)f;
#endif
}

inline int64_t roundS64(double d) noexcept
{
#if ROUND_TO_NEAREST
	return llrint(d);
#else
	return (int64_t)d;
#endif
}

enum class DMState : uint8_t
{
	idle = 0,
	moving = 1,
	stepError = 2
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
	void PrepareCartesianAxis(const DDA& dda, const PrepParams& params) noexcept SPEED_CRITICAL;
	void PrepareDeltaAxis(const DDA& dda, const PrepParams& params) noexcept SPEED_CRITICAL;
	void PrepareExtruder(const DDA& dda, const PrepParams& params, float speedChange) noexcept SPEED_CRITICAL;
	void DebugPrint(char c) const noexcept;
	int32_t GetNetStepsTaken() const noexcept;
	bool IsDeltaMovement() const noexcept { return isDeltaMovement; }

#if HAS_SMART_DRIVERS
	uint32_t GetStepInterval(uint32_t msShift) const noexcept;	// Get the current full step interval for this axis or extruder
#endif

#if SUPPORT_CLOSED_LOOP
	void AdjustNetSteps(float proportion) noexcept;
#endif

private:
	bool CalcNextStepTimeCartesianFull(const DDA &dda) noexcept SPEED_CRITICAL;
#if SUPPORT_DELTA_MOVEMENT
	bool CalcNextStepTimeDeltaFull(const DDA &dda) noexcept SPEED_CRITICAL;
#endif

	static DriveMovement *freeList;
	static int numFree;
	static int minFree;

	// Parameters common to Cartesian, delta and extruder moves

#if !SINGLE_DRIVER
	DriveMovement *nextDM;								// link to next DM that needs a step
#endif

	DMState state;										// whether this is active or not
	uint8_t drive;										// the drive that this DM controls
	uint8_t direction : 1,								// true=forwards, false=backwards
			directionChanged : 1,						// set by CalcNextStepTime if the direction is changed
			isDeltaMovement : 1;						// true if this motor is executing a delta tower move
	uint8_t stepsTillRecalc;							// how soon we need to recalculate

	uint32_t totalSteps;								// total number of steps for this move
#if SUPPORT_CLOSED_LOOP
	int32_t netSteps;
#endif

	// These values change as the step is executed, except for reverseStartStep
	uint32_t nextStep;									// number of steps already done
	uint32_t reverseStartStep;							// the step number for which we need to reverse direction due to pressure advance or delta movement
	uint32_t nextStepTime;								// how many clocks after the start of this move the next step is due
	uint32_t stepInterval;								// how many clocks between steps

#if DM_USE_FPU
	float fMmPerStepTimesCdivtopSpeed;
#else
	uint32_t mmPerStepTimesCKdivtopSpeed;
#endif

	// At this point we are 64-bit aligned
	// The following only needs to be stored per-drive if we are supporting pressure advance
#if DM_USE_FPU
	float fTwoDistanceToStopTimesCsquaredDivD;
#else
	uint64_t twoDistanceToStopTimesCsquaredDivD;
#endif

#if DM_USE_FPU
	float fTwoCsquaredTimesMmPerStepDivA;				// 2 * clock^2 * mmPerStepInHyperCuboidSpace / acceleration
	float fTwoCsquaredTimesMmPerStepDivD;				// 2 * clock^2 * mmPerStepInHyperCuboidSpace / deceleration
#else
	uint64_t twoCsquaredTimesMmPerStepDivA;				// 2 * clock^2 * mmPerStepInHyperCuboidSpace / acceleration
	uint64_t twoCsquaredTimesMmPerStepDivD;				// 2 * clock^2 * mmPerStepInHyperCuboidSpace / deceleration
#endif

	// Parameters unique to a style of move (Cartesian, delta or extruder). Currently, extruders and Cartesian moves use the same parameters.
	union MoveParams
	{
		struct CartesianParameters						// Parameters for Cartesian and extruder movement, including extruder pressure advance
		{
			// The following depend on how the move is executed, so they must be set up in Prepare()
#if DM_USE_FPU
			float fFourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD;
#else
			int64_t fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD;		// this one can be negative
#endif
			uint32_t accelStopStep;						// the first step number at which we are no longer accelerating
			uint32_t decelStartStep;					// the first step number at which we are decelerating
			uint32_t compensationClocks;				// the pressure advance time in clocks
			uint32_t accelCompensationClocks;			// compensationClocks * (1 - startSpeed/topSpeed)
		} cart;

#if SUPPORT_DELTA_MOVEMENT
		struct DeltaParameters							// Parameters for delta movement
		{
# if DM_USE_FPU
			// The following don't depend on how the move is executed, so they could be set up in Init() if we use fixed acceleration/deceleration
			float fDSquaredMinusAsquaredMinusBsquaredTimesSsquared;
			float fHmz0s;								// the starting step position less the starting Z height, multiplied by the Z movement fraction and K (can go negative)
			float fMinusAaPlusBbTimesS;

			// The following depend on how the move is executed, so they must be set up in Prepare()
			float fAccelStopDs;
			float fDecelStartDs;
# else
			// The following don't depend on how the move is executed, so they could be set up in Init() if we use fixed acceleration/deceleration
			int64_t dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared;
			int32_t hmz0sK;								// the starting step position less the starting Z height, multiplied by the Z movement fraction and K (can go negative)
			int32_t minusAaPlusBbTimesKs;

			// The following depend on how the move is executed, so they must be set up in Prepare()
			uint32_t accelStopDsK;
			uint32_t decelStartDsK;
# endif
		} delta;
#endif
	} mp;

	static constexpr uint32_t NoStepTime = 0xFFFFFFFF;	// value to indicate that no further steps are needed when calculating the next step time

#if !DM_USE_FPU
	static constexpr uint32_t K1 = 1024;				// a power of 2 used to multiply the value mmPerStepTimesCdivtopSpeed to reduce rounding errors
	static constexpr uint32_t K2 = 512;					// a power of 2 used in delta calculations to reduce rounding errors (but too large makes things worse)
	static constexpr int32_t Kc = 1024 * 1024;			// a power of 2 for scaling the Z movement fraction
#endif
};

// Calculate and store the time since the start of the move when the next step for the specified DriveMovement is due.
// Return true if there are more steps to do. When finished, leave nextStep == totalSteps + 1 and state == DMState::idle.
// This is also used for extruders on delta machines.
// We inline this part to speed things up when we are doing double/quad/octal stepping.
inline bool DriveMovement::CalcNextStepTime(const DDA &dda) noexcept
{
	++nextStep;
	if (nextStep <= totalSteps)
	{
		if (stepsTillRecalc != 0)
		{
			--stepsTillRecalc;			// we are doing double/quad/octal stepping
#if USE_EVEN_STEPS
			nextStepTime += stepInterval;
#endif
			return true;
		}
#if SUPPORT_DELTA_MOVEMENT
		return (IsDeltaMovement()) ? CalcNextStepTimeDeltaFull(dda) : CalcNextStepTimeCartesianFull(dda);
#else
		return CalcNextStepTimeCartesianFull(dda);
#endif
	}

	state = DMState::idle;
	return false;
}

// Return the number of net steps already taken for the move in the forwards direction.
// We have already taken nextSteps - 1 steps, unless nextStep is zero.
inline int32_t DriveMovement::GetNetStepsTaken() const noexcept
{
#if SUPPORT_CLOSED_LOOP
	if (ClosedLoop::GetClosedLoopEnabled(drive))
	{
		return netSteps;
	}
#endif

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

#endif

#endif	// SUPPORT_DRIVERS

#endif /* DRIVEMOVEMENT_H_ */
