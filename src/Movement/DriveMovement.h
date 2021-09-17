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

#if SUPPORT_CLOSED_LOOP

// Struct to pass data back to the ClosedLoop module
struct MotionParameters
{
	float position;
	float speed;
	float acceleration;
};

#endif

class LinearDeltaKinematics;
class DDA;

#define DM_USE_FPU			(__FPU_USED)
#define ROUND_TO_NEAREST	(0)			// 1 for round to nearest (as used in 1.20beta10), 0 for round down (as used prior to 1.20beta10)

// Rounding functions, to improve code clarity. Also allows a quick switch between round-to-nearest and round down in the movement code.
inline uint32_t roundU32(float f)
{
#if ROUND_TO_NEAREST
	return (uint32_t)lrintf(f);
#else
	return (uint32_t)f;
#endif
}

inline uint32_t roundU32(double d)
{
#if ROUND_TO_NEAREST
	return lrint(d);
#else
	return (uint32_t)d;
#endif
}

inline int32_t roundS32(float f)
{
#if ROUND_TO_NEAREST
	return lrintf(f);
#else
	return (int32_t)f;
#endif
}

inline int32_t roundS32(double d)
{
#if ROUND_TO_NEAREST
	return lrint(d);
#else
	return (int32_t)d;
#endif
}

inline uint64_t roundU64(float f)
{
#if ROUND_TO_NEAREST
	return (uint64_t)llrintf(f);
#else
	return (uint64_t)f;
#endif
}

inline uint64_t roundU64(double d)
{
#if ROUND_TO_NEAREST
	return (uint64_t)llrint(d);
#else
	return (uint64_t)d;
#endif
}

inline int64_t roundS64(float f)
{
#if ROUND_TO_NEAREST
	return llrintf(f);
#else
	return (int64_t)f;
#endif
}

inline int64_t roundS64(double d)
{
#if ROUND_TO_NEAREST
	return llrint(d);
#else
	return (int64_t)d;
#endif
}

// Struct for passing parameters to the DriveMovement Prepare methods
struct PrepParams
{
	// Parameters used for all types of motion
	float totalDistance;
	float acceleration;
	float deceleration;
	float decelStartDistance;
#if DM_USE_FPU
	float fTopSpeedTimesCdivD;
#else
	uint32_t topSpeedTimesCdivD;
#endif

	// Parameters used only for delta moves
	float initialX;
	float initialY;
	const LinearDeltaKinematics *dparams;
	float a2plusb2;								// sum of the squares of the X and Y movement fractions
	float dvecX, dvecY, dvecZ;
};

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

	DriveMovement() { };

	bool CalcNextStepTime(const DDA &dda) SPEED_CRITICAL;
	void PrepareCartesianAxis(const DDA& dda, const PrepParams& params) SPEED_CRITICAL;
	void PrepareDeltaAxis(const DDA& dda, const PrepParams& params) SPEED_CRITICAL;
	void PrepareExtruder(const DDA& dda, const PrepParams& params, float speedChange) SPEED_CRITICAL;
	void DebugPrint(char c) const;
	int32_t GetNetStepsLeft() const;
	int32_t GetNetStepsTaken() const;
	bool IsDeltaMovement() const { return isDeltaMovement; }

#if HAS_SMART_DRIVERS
	uint32_t GetStepInterval(uint32_t msShift) const;	// Get the current full step interval for this axis or extruder
#endif

#if SUPPORT_CLOSED_LOOP
	void GetCurrentMotion(MotionParameters& mParams, int32_t netMicrostepsTaken, int microstepShift) const noexcept;
#endif

private:
	bool CalcNextStepTimeCartesianFull(const DDA &dda) SPEED_CRITICAL;
#if SUPPORT_DELTA_MOVEMENT
	bool CalcNextStepTimeDeltaFull(const DDA &dda) SPEED_CRITICAL;
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
inline bool DriveMovement::CalcNextStepTime(const DDA &dda)
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

// Return the number of net steps left for the move in the forwards direction.
// We have already taken nextSteps - 1 steps, unless nextStep is zero.
inline int32_t DriveMovement::GetNetStepsLeft() const
{
	int32_t netStepsLeft;
	if (reverseStartStep > totalSteps)		// if no reverse phase
	{
		netStepsLeft = (nextStep == 0) ? (int32_t)totalSteps : (int32_t)totalSteps - (int32_t)nextStep + 1;
	}
	else if (nextStep >= reverseStartStep)
	{
		netStepsLeft = (int32_t)totalSteps - (int32_t)nextStep + 1;
	}
	else
	{
		const int32_t totalNetSteps = (int32_t)(2 * reverseStartStep) - (int32_t)totalSteps - 2;
		netStepsLeft = (nextStep == 0) ? totalNetSteps : totalNetSteps - (int32_t)nextStep + 1;
	}
	return (direction) ? netStepsLeft : -netStepsLeft;
}

// Return the number of net steps already taken for the move in the forwards direction.
// We have already taken nextSteps - 1 steps, unless nextStep is zero.
inline int32_t DriveMovement::GetNetStepsTaken() const
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

#if HAS_SMART_DRIVERS

// Get the current full step interval for this axis or extruder
inline uint32_t DriveMovement::GetStepInterval(uint32_t microstepShift) const
{
	return ((nextStep >> microstepShift) != 0)		// if at least 1 full step done
		? stepInterval << microstepShift			// return the interval between steps converted to full steps
			: 0;
}

#endif

#if SUPPORT_CLOSED_LOOP

// This is called on the current DDA with interrupts disabled, to report the current position, speed and acceleration
// netMicrostepsTaken is the position in microsteps at the start of this move
inline void DriveMovement::GetCurrentMotion(MotionParameters& mParams, int32_t netMicrostepsTaken, int microstepShift) const noexcept
{
	// When we switch to a segment-based approach we will calculate the position directly.
	mParams.position = ldexp(netMicrostepsTaken + GetNetStepsTaken(), microstepShift);
	mParams.speed = mParams.acceleration = 0;
}

#endif

	#endif	// SUPPORT_DRIVERS

#endif /* DRIVEMOVEMENT_H_ */
