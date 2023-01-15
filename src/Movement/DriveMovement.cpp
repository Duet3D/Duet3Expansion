/*
 * DriveMovement.cpp
 *
 *  Created on: 17 Jan 2015
 *      Author: David
 */

#include "DriveMovement.h"

#if SUPPORT_DRIVERS

#include "DDA.h"
#include "Move.h"
#include "Math/Isqrt.h"

#if SUPPORT_DELTA_MOVEMENT
# include "Kinematics/LinearDeltaKinematics.h"
#endif

#include "StepTimer.h"
#include <Platform/Platform.h>

// Prepare this DM for a Cartesian axis move
void DriveMovement::PrepareCartesianAxis(const DDA& dda, const PrepParams& params) noexcept
{
	isDeltaMovement = false;

#if DM_USE_FPU
	fTwoCsquaredTimesMmPerStepDivA = (float)((double)2.0/((double)totalSteps * (double)dda.acceleration));
	fTwoCsquaredTimesMmPerStepDivD = (float)((double)2.0/((double)totalSteps * (double)dda.deceleration));
#else
	twoCsquaredTimesMmPerStepDivA = roundU64((double)2.0/((double)totalSteps * (double)dda.acceleration));
	twoCsquaredTimesMmPerStepDivD = roundU64((double)2.0/((double)totalSteps * (double)dda.deceleration));
#endif

	// Acceleration phase parameters
	mp.cart.accelStopStep = (uint32_t)(dda.accelDistance * totalSteps) + 1;
	mp.cart.compensationClocks = mp.cart.accelCompensationClocks = 0;

	// Constant speed phase parameters
#if DM_USE_FPU
	fMmPerStepTimesCdivtopSpeed = 1.0/(totalSteps * dda.topSpeed);
#else
	mmPerStepTimesCKdivtopSpeed = roundU32(((float)K1)/(totalSteps * dda.topSpeed));
#endif

	// Deceleration phase parameters
	// First check whether there is any deceleration at all, otherwise we may get strange results because of rounding errors
	if (dda.decelDistance * totalSteps < 0.5)
	{
		mp.cart.decelStartStep = totalSteps + 1;
#if DM_USE_FPU
		fTwoDistanceToStopTimesCsquaredDivD = 0.0;
#else
		twoDistanceToStopTimesCsquaredDivD = 0;
#endif
	}
	else
	{
		mp.cart.decelStartStep = (uint32_t)(params.decelStartDistance * totalSteps) + 1;
#if DM_USE_FPU
		fTwoDistanceToStopTimesCsquaredDivD = fsquare(params.fTopSpeedTimesCdivD) + (params.decelStartDistance * 2)/dda.deceleration;
#else
		twoDistanceToStopTimesCsquaredDivD = isquare64(params.topSpeedTimesCdivD) + roundU64((params.decelStartDistance * 2)/dda.deceleration);
#endif
	}

	// No reverse phase
	reverseStartStep = totalSteps + 1;
#if DM_USE_FPU
	mp.cart.fFourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD = 0.0;
#else
	mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD = 0;
#endif
}

#if SUPPORT_DELTA_MOVEMENT

// Prepare this DM for a Delta axis move
//TODO convert this to normalised coordinates like we did for Cartesian drives
void DriveMovement::PrepareDeltaAxis(const DDA& dda, const PrepParams& params) noexcept
{
	isDeltaMovement = true;

	const float stepsPerMm = Platform::DriveStepsPerUnit(drive);
	const float A = params.initialX - params.dparams->GetTowerX(drive);
	const float B = params.initialY - params.dparams->GetTowerY(drive);
	const float aAplusbB = A * params.dvecX + B * params.dvecY;
	const float dSquaredMinusAsquaredMinusBsquared = params.dparams->GetDiagonalSquared(drive) - fsquare(A) - fsquare(B);
	const float h0MinusZ0 = fastSqrtf(dSquaredMinusAsquaredMinusBsquared);
#if DM_USE_FPU
	mp.delta.fHmz0s = h0MinusZ0 * stepsPerMm;
	mp.delta.fMinusAaPlusBbTimesS = -(aAplusbB * stepsPerMm);
	mp.delta.fDSquaredMinusAsquaredMinusBsquaredTimesSsquared = dSquaredMinusAsquaredMinusBsquared * fsquare(stepsPerMm);
	fTwoCsquaredTimesMmPerStepDivA = (float)((double)(2 * StepTimer::StepClockRateSquared)/((double)stepsPerMm * (double)dda.acceleration));
	fTwoCsquaredTimesMmPerStepDivD = (float)((double)(2 * StepTimer::StepClockRateSquared)/((double)stepsPerMm * (double)dda.deceleration));
#else
	mp.delta.hmz0sK = roundS32(h0MinusZ0 * stepsPerMm * DriveMovement::K2);
	mp.delta.minusAaPlusBbTimesKs = -roundS32(aAplusbB * stepsPerMm * DriveMovement::K2);
	mp.delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared = roundS64(dSquaredMinusAsquaredMinusBsquared * fsquare(stepsPerMm * DriveMovement::K2));
	twoCsquaredTimesMmPerStepDivA = roundU64((double)(2 * StepTimer::StepClockRateSquared)/((double)stepsPerMm * (double)dda.acceleration));
	twoCsquaredTimesMmPerStepDivD = roundU64((double)(2 * StepTimer::StepClockRateSquared)/((double)stepsPerMm * (double)dda.deceleration));
#endif

	// Calculate the distance at which we need to reverse direction.
	if (params.a2plusb2 <= 0.0)
	{
		// Pure Z movement. We can't use the main calculation because it divides by a2plusb2.
		direction = (params.dvecZ >= 0.0);
		reverseStartStep = totalSteps + 1;
	}
	else
	{
		// The distance to reversal is the solution to a quadratic equation. One root corresponds to the carriages being below the bed,
		// the other root corresponds to the carriages being above the bed.
		const float drev = ((params.dvecZ * fastSqrtf(params.a2plusb2 * params.dparams->GetDiagonalSquared(drive) - fsquare(A * params.dvecY - B * params.dvecX)))
							- aAplusbB)/params.a2plusb2;
		if (drev > 0.0 && drev < 1.0)		// if the reversal point is within range
		{
			// Calculate how many steps we need to move up before reversing
			const float hrev = params.dvecZ * drev + fastSqrtf(dSquaredMinusAsquaredMinusBsquared - 2 * drev * aAplusbB - params.a2plusb2 * fsquare(drev));
			const int32_t numStepsUp = (int32_t)((hrev - h0MinusZ0) * stepsPerMm);

			// We may be almost at the peak height already, in which case we don't really have a reversal.
			if (numStepsUp < 1 || (direction && (uint32_t)numStepsUp <= totalSteps))
			{
				reverseStartStep = totalSteps + 1;
			}
			else
			{
				reverseStartStep = (uint32_t)numStepsUp + 1;

				// Correct the initial direction and the total number of steps
				if (direction)
				{
					// Net movement is up, so we will go up a bit and then down by a lesser amount
					totalSteps = (2 * numStepsUp) - totalSteps;
				}
				else
				{
					// Net movement is down, so we will go up first and then down by a greater amount
					direction = true;
					totalSteps = (2 * numStepsUp) + totalSteps;
				}
			}
		}
		else
		{
			reverseStartStep = totalSteps + 1;
		}
	}

	// Acceleration phase parameters
#if DM_USE_FPU
	mp.delta.fAccelStopDs = dda.accelDistance * stepsPerMm;
#else
	mp.delta.accelStopDsK = roundU32(dda.accelDistance * stepsPerMm * K2);
#endif

	// Constant speed phase parameters
#if DM_USE_FPU
	fMmPerStepTimesCdivtopSpeed = 1.0/(stepsPerMm * dda.topSpeed);
#else
	mmPerStepTimesCKdivtopSpeed = roundU32((float)K1/(stepsPerMm * dda.topSpeed));
#endif

	// Deceleration phase parameters
	// First check whether there is any deceleration at all, otherwise we may get strange results because of rounding errors
	if (dda.decelDistance * stepsPerMm < 0.5)
	{
#if DM_USE_FPU
		mp.delta.fDecelStartDs = std::numeric_limits<float>::max();
		fTwoDistanceToStopTimesCsquaredDivD = 0.0;
#else
		mp.delta.decelStartDsK = 0xFFFFFFFF;
		twoDistanceToStopTimesCsquaredDivD = 0;
#endif
	}
	else
	{
#if DM_USE_FPU
		mp.delta.fDecelStartDs = params.decelStartDistance * stepsPerMm;
		fTwoDistanceToStopTimesCsquaredDivD = fsquare(params.fTopSpeedTimesCdivD) + (params.decelStartDistance * 2)/dda.deceleration;
#else
		mp.delta.decelStartDsK = roundU32(params.decelStartDistance * stepsPerMm * K2);
		twoDistanceToStopTimesCsquaredDivD = isquare64(params.topSpeedTimesCdivD) + roundU64((params.decelStartDistance * 2)/dda.deceleration);
#endif
	}
}

#endif

// Prepare this DM for an extruder move. The caller has already checked that pressure advance is enabled.
void DriveMovement::PrepareExtruder(const DDA& dda, const PrepParams& params, float speedChange) noexcept
{
	// Calculate the pressure advance parameters
	const float compensationClocks = Platform::GetPressureAdvanceClocks(drive);
	if (compensationClocks < 1.0)
	{
		return PrepareCartesianAxis(dda, params);			// no compensation active, so use the simpler calculation
	}

	isDeltaMovement = false;

	mp.cart.compensationClocks = roundU32(compensationClocks);

	// Recalculate the net total step count to allow for compensation. It may be negative.
	const int32_t extraSteps = (dda.endSpeed - dda.startSpeed) * compensationClocks * totalSteps;
	int32_t netStepsNeeded = (int32_t)totalSteps + extraSteps;

	// Calculate the acceleration phase parameters
	const float accelCompensationDistance = compensationClocks * (dda.topSpeed - dda.startSpeed);
	mp.cart.accelCompensationClocks = roundU32(accelCompensationDistance/dda.topSpeed);
	mp.cart.accelStopStep = (uint32_t)((dda.accelDistance + accelCompensationDistance) * totalSteps) + 1;

#if DM_USE_FPU
	fTwoCsquaredTimesMmPerStepDivA = (double)2.0/((double)totalSteps * (double)dda.acceleration);
	fTwoCsquaredTimesMmPerStepDivD = (double)2.0/((double)totalSteps * (double)dda.deceleration);
#else
	twoCsquaredTimesMmPerStepDivA = roundU64((double)2.0/((double)totalSteps * (double)dda.acceleration));
	twoCsquaredTimesMmPerStepDivD = roundU64((double)2.0/((double)totalSteps * (double)dda.deceleration));
#endif

	// Constant speed phase parameters
#if DM_USE_FPU
	fMmPerStepTimesCdivtopSpeed = 1.0/(totalSteps * dda.topSpeed);
#else
	mmPerStepTimesCKdivtopSpeed = (uint32_t)((float)K1/(totalSteps * dda.topSpeed));
#endif

	uint32_t newTotalSteps;

	// Calculate the deceleration and reverse phase parameters and update totalSteps
	// First check whether there is any deceleration at all, otherwise we may get strange results because of rounding errors
	if (dda.decelDistance * totalSteps < 0.5)		// if less than 1 deceleration step
	{
		newTotalSteps = (uint32_t)max<int32_t>(netStepsNeeded, 0);
		mp.cart.decelStartStep = reverseStartStep = newTotalSteps + 1;
#if DM_USE_FPU
		mp.cart.fFourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD = 0.0;
		fTwoDistanceToStopTimesCsquaredDivD = 0.0;
#else
		mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD = 0;
		twoDistanceToStopTimesCsquaredDivD = 0;
#endif
	}
	else
	{
		mp.cart.decelStartStep = (uint32_t)((params.decelStartDistance + accelCompensationDistance) * totalSteps) + 1;
#if DM_USE_FPU
		const float initialDecelSpeedTimesCdivD = params.fTopSpeedTimesCdivD - (float)mp.cart.compensationClocks;
		const float initialDecelSpeedTimesCdivDSquared = fsquare(initialDecelSpeedTimesCdivD);
		fTwoDistanceToStopTimesCsquaredDivD =
			initialDecelSpeedTimesCdivDSquared + ((params.decelStartDistance + accelCompensationDistance) * 2)/dda.deceleration;
#else
		const int32_t initialDecelSpeedTimesCdivD = (int32_t)params.topSpeedTimesCdivD - (int32_t)mp.cart.compensationClocks;	// signed because it may be negative and we square it
		const uint64_t initialDecelSpeedTimesCdivDSquared = isquare64(initialDecelSpeedTimesCdivD);
		twoDistanceToStopTimesCsquaredDivD =
			initialDecelSpeedTimesCdivDSquared + roundU64(((params.decelStartDistance + accelCompensationDistance) * 2)/dda.deceleration);
#endif

		// See whether there is a reverse phase
		const float compensationSpeedChange = dda.deceleration * compensationClocks;
		const uint32_t stepsBeforeReverse = (compensationSpeedChange > dda.topSpeed)
											? mp.cart.decelStartStep - 1
#if DM_USE_FPU
											: (uint32_t)(fTwoDistanceToStopTimesCsquaredDivD/fTwoCsquaredTimesMmPerStepDivD);
#else
											: twoDistanceToStopTimesCsquaredDivD/twoCsquaredTimesMmPerStepDivD;
#endif
		if (dda.endSpeed < compensationSpeedChange && (int32_t)stepsBeforeReverse > netStepsNeeded)
		{
			reverseStartStep = stepsBeforeReverse + 1;
			newTotalSteps = (uint32_t)((int32_t)(2 * stepsBeforeReverse) - netStepsNeeded);
#if DM_USE_FPU
			mp.cart.fFourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD = (2 * stepsBeforeReverse) * fTwoCsquaredTimesMmPerStepDivD - fTwoDistanceToStopTimesCsquaredDivD;
#else
			mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD =
					(int64_t)((2 * stepsBeforeReverse) * twoCsquaredTimesMmPerStepDivD) - (int64_t)twoDistanceToStopTimesCsquaredDivD;
#endif
		}
		else
		{
			// There is no reverse phase. Check that we can actually do the last step requested.
			if (netStepsNeeded > (int32_t)stepsBeforeReverse)
			{
				netStepsNeeded = (int32_t)stepsBeforeReverse;
			}
			newTotalSteps = (uint32_t)max<int32_t>(netStepsNeeded, 0);
			reverseStartStep = newTotalSteps + 1;
#if DM_USE_FPU
			mp.cart.fFourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD = 0.0;
#else
			mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD = 0;
#endif
		}
	}
	DDA::stepsRequested[drive] += newTotalSteps - totalSteps;
	totalSteps = newTotalSteps;
}

void DriveMovement::DebugPrint(char c) const
{
	if (state != DMState::idle)
	{
#if DM_USE_FPU
		debugPrintf("DM%c%s dir=%c steps=%" PRIu32 " next=%" PRIu32 " rev=%" PRIu32 " interval=%" PRIu32
					" 2dtstc2diva=%.2f\n",
					c, (state == DMState::stepError) ? " ERR:" : ":", (direction) ? 'F' : 'B', totalSteps, nextStep, reverseStartStep, stepInterval,
					(double)fTwoDistanceToStopTimesCsquaredDivD);
#else
		debugPrintf("DM%c%s dir=%c steps=%" PRIu32 " next=%" PRIu32 " rev=%" PRIu32 " interval=%" PRIu32
					" 2dtstc2divd=%" PRIu64 "\n",
					c, (state == DMState::stepError) ? " ERR:" : ":", (direction) ? 'F' : 'B', totalSteps, nextStep, reverseStartStep, stepInterval,
					twoDistanceToStopTimesCsquaredDivD);
#endif

#if SUPPORT_DELTA_MOVEMENT
		if (isDeltaMovement)
		{
# if DM_USE_FPU
			debugPrintf("hmz0s=%.2f minusAaPlusBbTimesS=%.2f dSquaredMinusAsquaredMinusBsquared=%.2f\n"
						"2c2mmsda=%.2f 2c2mmsdd=%.2f asds=%.2f dsds=%.2f mmstcdts=%.2f\n",
						(double)mp.delta.fHmz0s, (double)mp.delta.fMinusAaPlusBbTimesS, (double)mp.delta.fDSquaredMinusAsquaredMinusBsquaredTimesSsquared,
						(double)fTwoCsquaredTimesMmPerStepDivA, (double)fTwoCsquaredTimesMmPerStepDivD, (double)mp.delta.fAccelStopDs, (double)mp.delta.fDecelStartDs, (double)fMmPerStepTimesCdivtopSpeed
						);
# else
			debugPrintf("hmz0sK=%" PRIi32 " minusAaPlusBbTimesKs=%" PRIi32 " dSquaredMinusAsquaredMinusBsquared=%" PRId64 "\n"
						"2c2mmsda=%" PRIu64 "2c2mmsdd=%" PRIu64 " asdsk=%" PRIu32 " dsdsk=%" PRIu32 " mmstcdts=%" PRIu32 "\n",
						mp.delta.hmz0sK, mp.delta.minusAaPlusBbTimesKs, mp.delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared,
						twoCsquaredTimesMmPerStepDivA, twoCsquaredTimesMmPerStepDivD, mp.delta.accelStopDsK, mp.delta.decelStartDsK, mmPerStepTimesCKdivtopSpeed
						);
# endif
		}
		else
#endif
		{
#if DM_USE_FPU
			debugPrintf("accelStopStep=%" PRIu32 " decelStartStep=%" PRIu32 " 2c2mmsda=%.2f 2c2mmsdd=%.2f\n"
						"mmPerStepTimesCdivtopSpeed=%.2f fmsdmtstdca2=%.2f cc=%" PRIu32 " acc=%" PRIu32 "\n",
						mp.cart.accelStopStep, mp.cart.decelStartStep, (double)fTwoCsquaredTimesMmPerStepDivA, (double)fTwoCsquaredTimesMmPerStepDivD,
						(double)fMmPerStepTimesCdivtopSpeed, (double)mp.cart.fFourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD, mp.cart.compensationClocks, mp.cart.accelCompensationClocks
						);
#else
			debugPrintf("accelStopStep=%" PRIu32 " decelStartStep=%" PRIu32 " 2c2mmsda=%" PRIu64 " 2c2mmsdd=%" PRIu64 "\n"
						"mmPerStepTimesCdivtopSpeed=%" PRIu32 " fmsdmtstdca2=%" PRId64 " cc=%" PRIu32 " acc=%" PRIu32 "\n",
						mp.cart.accelStopStep, mp.cart.decelStartStep, twoCsquaredTimesMmPerStepDivA, twoCsquaredTimesMmPerStepDivD,
						mmPerStepTimesCKdivtopSpeed, mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD, mp.cart.compensationClocks, mp.cart.accelCompensationClocks
						);
#endif
		}
	}
	else
	{
		debugPrintf("DM%c: not moving\n", c);
	}
}

// Calculate and store the time since the start of the move when the next step for the specified DriveMovement is due.
// Return true if there are more steps to do.
// This is also used for extruders on delta machines.
bool DriveMovement::CalcNextStepTimeCartesianFull(const DDA &dda) noexcept
pre(nextStep < totalSteps; stepsTillRecalc == 0)
{
	// Work out how many steps to calculate at a time.
	// The last step before reverseStartStep must be single stepped to make sure that we don't reverse the direction too soon.
	uint32_t shiftFactor = 0;		// assume single stepping
	if (stepInterval < DDA::MinCalcIntervalCartesian)
	{
		const uint32_t stepsToLimit = ((nextStep <= reverseStartStep && reverseStartStep <= totalSteps)
										? reverseStartStep
										: totalSteps
									  ) - nextStep;
		if (stepInterval < DDA::MinCalcIntervalCartesian/4 && stepsToLimit > 8)
		{
			shiftFactor = 3;		// octal stepping
		}
		else if (stepInterval < DDA::MinCalcIntervalCartesian/2 && stepsToLimit > 4)
		{
			shiftFactor = 2;		// quad stepping
		}
		else if (stepsToLimit > 2)
		{
			shiftFactor = 1;		// double stepping
		}
	}

	stepsTillRecalc = (1u << shiftFactor) - 1u;					// store number of additional steps to generate

	const uint32_t nextCalcStep = nextStep + stepsTillRecalc;
	uint32_t nextCalcStepTime;
	if (nextCalcStep < mp.cart.accelStopStep)
	{
		// acceleration phase
#if DM_USE_FPU
		const float adjustedStartSpeedTimesCdivA = (float)(dda.afterPrepare.startSpeedTimesCdivA + mp.cart.compensationClocks);
		nextCalcStepTime = (uint32_t)(fastSqrtf(fsquare(adjustedStartSpeedTimesCdivA) + (fTwoCsquaredTimesMmPerStepDivA * nextCalcStep)) - adjustedStartSpeedTimesCdivA);
#else
		const uint32_t adjustedStartSpeedTimesCdivA = dda.afterPrepare.startSpeedTimesCdivA + mp.cart.compensationClocks;
		nextCalcStepTime = isqrt64(isquare64(adjustedStartSpeedTimesCdivA) + (twoCsquaredTimesMmPerStepDivA * nextCalcStep)) - adjustedStartSpeedTimesCdivA;
#endif
	}
	else if (nextCalcStep < mp.cart.decelStartStep)
	{
		// steady speed phase
		nextCalcStepTime =
#if DM_USE_FPU
					(uint32_t)(  (int32_t)(fMmPerStepTimesCdivtopSpeed * nextCalcStep)
							   + dda.afterPrepare.extraAccelerationClocks
							   - (int32_t)mp.cart.accelCompensationClocks
							  );
#else
					(uint32_t)(  (int32_t)(((uint64_t)mmPerStepTimesCKdivtopSpeed * nextCalcStep)/K1)
							   + dda.afterPrepare.extraAccelerationClocks
							   - (int32_t)mp.cart.accelCompensationClocks
							  );
#endif
	}
	else if (nextCalcStep < reverseStartStep)
	{
		// deceleration phase, not reversed yet
		const uint32_t adjustedTopSpeedTimesCdivDPlusDecelStartClocks = dda.afterPrepare.topSpeedTimesCdivDPlusDecelStartClocks - mp.cart.compensationClocks;
#if DM_USE_FPU
		const float temp = fTwoCsquaredTimesMmPerStepDivD * nextCalcStep;
		// Allow for possible rounding error when the end speed is zero or very small
		nextCalcStepTime = (temp < fTwoDistanceToStopTimesCsquaredDivD)
						? adjustedTopSpeedTimesCdivDPlusDecelStartClocks - (uint32_t)(fastSqrtf(fTwoDistanceToStopTimesCsquaredDivD - temp))
						: adjustedTopSpeedTimesCdivDPlusDecelStartClocks;
#else
		const uint64_t temp = twoCsquaredTimesMmPerStepDivD * nextCalcStep;
		// Allow for possible rounding error when the end speed is zero or very small
		nextCalcStepTime = (temp < twoDistanceToStopTimesCsquaredDivD)
						? adjustedTopSpeedTimesCdivDPlusDecelStartClocks - isqrt64(twoDistanceToStopTimesCsquaredDivD - temp)
						: adjustedTopSpeedTimesCdivDPlusDecelStartClocks;
#endif
	}
	else
	{
		// deceleration phase, reversing or already reversed
		if (nextCalcStep == reverseStartStep)
		{
			direction = !direction;
			directionChanged = true;
		}
		const uint32_t adjustedTopSpeedTimesCdivDPlusDecelStartClocks = dda.afterPrepare.topSpeedTimesCdivDPlusDecelStartClocks - mp.cart.compensationClocks;
		nextCalcStepTime = adjustedTopSpeedTimesCdivDPlusDecelStartClocks
#if DM_USE_FPU
							+ (uint32_t)(fastSqrtf((fTwoCsquaredTimesMmPerStepDivD * nextCalcStep) - mp.cart.fFourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD));
#else
							+ isqrt64((int64_t)(twoCsquaredTimesMmPerStepDivD * nextCalcStep) - mp.cart.fourMaxStepDistanceMinusTwoDistanceToStopTimesCsquaredDivD);
#endif
	}

	// When crossing between movement phases with high microstepping, due to rounding errors the next step may appear to be due before the last one
	stepInterval = (nextCalcStepTime > nextStepTime)
					? (nextCalcStepTime - nextStepTime) >> shiftFactor	// calculate the time per step, ready for next time
					: 0;
#if USE_EVEN_STEPS
	nextStepTime = nextCalcStepTime - (stepsTillRecalc * stepInterval);
#else
	nextStepTime = nextCalcStepTime;
#endif

	if (nextCalcStepTime > dda.clocksNeeded)
	{
		// The calculation makes this step late.
		// When the end speed is very low, calculating the time of the last step is very sensitive to rounding error.
		// So if this is the last step and it is late, bring it forward to the expected finish time.
		// Very rarely on a delta, the penultimate step may also be calculated late. Allow for that here in case it affects Cartesian axes too.
		if (nextStep + 1 >= totalSteps)
		{
			nextStepTime = dda.clocksNeeded;
		}
		else
		{
			// We don't expect any step except the last to be late
			state = DMState::stepError;
			stepInterval = 10000000 + nextStepTime;				// so we can tell what happened in the debug print
			DDA::RecordStepError();
			return false;
		}
	}
	return true;
}

#if SUPPORT_DELTA_MOVEMENT

// Calculate the time since the start of the move when the next step for the specified DriveMovement is due
// Return true if there are more steps to do
bool DriveMovement::CalcNextStepTimeDeltaFull(const DDA &dda) noexcept
pre(nextStep < totalSteps; stepsTillRecalc == 0)
{
	// Work out how many steps to calculate at a time.
	// The last step before reverseStartStep must be single stepped to make sure that we don't reverse the direction too soon.
	// The simulator suggests that at 200steps/mm, the minimum step pulse interval for 400mm/sec movement is 4.5us
	uint32_t shiftFactor = 0;		// assume single stepping
	if (stepInterval < DDA::MinCalcIntervalDelta)
	{
		const uint32_t stepsToLimit = ((nextStep < reverseStartStep && reverseStartStep <= totalSteps)
										? reverseStartStep
										: totalSteps
									  ) - nextStep;
		if (stepInterval < DDA::MinCalcIntervalDelta/8 && stepsToLimit > 16)
		{
			shiftFactor = 4;		// hexadecimal stepping
		}
		else if (stepInterval < DDA::MinCalcIntervalDelta/4 && stepsToLimit > 8)
		{
			shiftFactor = 3;		// octal stepping
		}
		else if (stepInterval < DDA::MinCalcIntervalDelta/2 && stepsToLimit > 4)
		{
			shiftFactor = 2;		// quad stepping
		}
		else if (stepsToLimit > 2)
		{
			shiftFactor = 1;		// double stepping
		}
	}

	stepsTillRecalc = (1u << shiftFactor) - 1;					// store number of additional steps to generate

	if (nextStep == reverseStartStep)
	{
		direction = false;
		directionChanged = true;
	}

	// Calculate d*s*K as an integer, where d = distance the head has travelled, s = steps/mm for this drive, K = a power of 2 to reduce the rounding errors
	{
#if DM_USE_FPU
		float steps = float(1u << shiftFactor);
		if (!direction)
		{
			steps = -steps;
		}
		mp.delta.fHmz0s += steps;								// get new carriage height above Z in steps
#else
		int32_t shiftedK2 = (int32_t)(K2 << shiftFactor);
		if (!direction)
		{
			shiftedK2 = -shiftedK2;
		}
		mp.delta.hmz0sK += shiftedK2;
#endif
	}

#if DM_USE_FPU
	const float hmz0sc = mp.delta.fHmz0s * dda.afterPrepare.zFraction;
	const float t1 = mp.delta.fMinusAaPlusBbTimesS + hmz0sc;
	// Due to rounding error we can end up trying to take the square root of a negative number if we do not take precautions here
	const float t2a = mp.delta.fDSquaredMinusAsquaredMinusBsquaredTimesSsquared - fsquare(mp.delta.fHmz0s) + fsquare(t1);
	const float t2 = (t2a > 0.0) ? fastSqrtf(t2a) : 0.0;
	const float ds = (direction) ? t1 - t2 : t1 + t2;
#else
	const int32_t hmz0scK = (int32_t)(((int64_t)mp.delta.hmz0sK * dda.afterPrepare.cKc)/Kc);
	const int32_t t1 = mp.delta.minusAaPlusBbTimesKs + hmz0scK;
	// Due to rounding error we can end up trying to take the square root of a negative number if we do not take precautions here
	const int64_t t2a = mp.delta.dSquaredMinusAsquaredMinusBsquaredTimesKsquaredSsquared - (int64_t)isquare64(mp.delta.hmz0sK) + (int64_t)isquare64(t1);
	const int32_t t2 = (t2a > 0) ? isqrt64(t2a) : 0;
	const int32_t dsK = (direction) ? t1 - t2 : t1 + t2;
#endif

	// Now feed dsK into a modified version of the step algorithm for Cartesian motion without elasticity compensation
#if DM_USE_FPU
	if (ds < 0.0)
#else
	if (dsK < 0)
#endif
	{
		state = DMState::stepError;
		nextStep += 1000000;						// so that we can tell what happened in the debug print
		return false;
	}

	uint32_t nextCalcStepTime;
#if DM_USE_FPU
	if (ds < mp.delta.fAccelStopDs)
	{
		// Acceleration phase
		nextCalcStepTime = (uint32_t)(fastSqrtf(fsquare((float)dda.afterPrepare.startSpeedTimesCdivA) + (fTwoCsquaredTimesMmPerStepDivA * ds))) - dda.afterPrepare.startSpeedTimesCdivA;
	}
	else if (ds < mp.delta.fDecelStartDs)
	{
		// Steady speed phase
		nextCalcStepTime = (uint32_t)(fMmPerStepTimesCdivtopSpeed * ds) + dda.afterPrepare.extraAccelerationClocks;
	}
	else
	{
		const float temp = fTwoCsquaredTimesMmPerStepDivD * ds;
		// Because of possible rounding error when the end speed is zero or very small, we need to check that the square root will work OK
		nextCalcStepTime = (temp < fTwoDistanceToStopTimesCsquaredDivD)
						? dda.afterPrepare.topSpeedTimesCdivDPlusDecelStartClocks - lrintf(fastSqrtf(fTwoDistanceToStopTimesCsquaredDivD - temp))
						: dda.afterPrepare.topSpeedTimesCdivDPlusDecelStartClocks;
	}
#else
	if ((uint32_t)dsK < mp.delta.accelStopDsK)
	{
		// Acceleration phase
		nextCalcStepTime = isqrt64(isquare64(dda.afterPrepare.startSpeedTimesCdivA) + (twoCsquaredTimesMmPerStepDivA * (uint32_t)dsK)/K2) - dda.afterPrepare.startSpeedTimesCdivA;
	}
	else if ((uint32_t)dsK < mp.delta.decelStartDsK)
	{
		// Steady speed phase
		nextCalcStepTime = (uint32_t)(  (int32_t)(((uint64_t)mmPerStepTimesCKdivtopSpeed * (uint32_t)dsK)/(K1 * K2))
								  + dda.afterPrepare.extraAccelerationClocks
								 );
	}
	else
	{
		const uint64_t temp = (twoCsquaredTimesMmPerStepDivD * (uint32_t)dsK)/K2;
		// Because of possible rounding error when the end speed is zero or very small, we need to check that the square root will work OK
		nextCalcStepTime = (temp < twoDistanceToStopTimesCsquaredDivD)
						? dda.afterPrepare.topSpeedTimesCdivDPlusDecelStartClocks - isqrt64(twoDistanceToStopTimesCsquaredDivD - temp)
						: dda.afterPrepare.topSpeedTimesCdivDPlusDecelStartClocks;
	}
#endif

	// When crossing between movement phases with high microstepping, due to rounding errors the next step may appear to be due before the last one.
	stepInterval = (nextCalcStepTime > nextStepTime)
					? (nextCalcStepTime - nextStepTime) >> shiftFactor	// calculate the time per step, ready for next time
					: 0;
#if USE_EVEN_STEPS
	nextStepTime = nextCalcStepTime - (stepsTillRecalc * stepInterval);
#else
	nextStepTime = nextCalcStepTime;
#endif

	if (nextCalcStepTime > dda.clocksNeeded)
	{
		// The calculation makes this step late.
		// When the end speed is very low, calculating the time of the last step is very sensitive to rounding error.
		// So if this is the last step and it is late, bring it forward to the expected finish time.
		// Very rarely, the penultimate step may be calculated late, so allow for that too.
		if (nextStep + 1 >= totalSteps)
		{
			nextStepTime = dda.clocksNeeded;
		}
		else
		{
			// We don't expect any steps except the last two to be late
			state = DMState::stepError;
			stepInterval = 10000000 + nextStepTime;		// so we can tell what happened in the debug print
			return false;
		}
	}
	return true;
}

#endif

#endif	// SUPPORT_DRIVERS

// End
