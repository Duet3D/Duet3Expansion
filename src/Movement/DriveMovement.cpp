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
#include "StepTimer.h"
#include <Math/Isqrt.h>
#include <Platform/Platform.h>

int32_t DriveMovement::maxStepsLate = 0;

void DriveMovement::DebugPrint() const noexcept
{
	const char c = drive + '0';
	if (state != DMState::idle)
	{
		const char *const errText = (state == DMState::stepError1) ? " ERR1:"
									: (state == DMState::stepError2) ? " ERR2:"
										: (state == DMState::stepError3) ? " ERR3:"
											: (state == DMState::stepError4) ? " ERR4:"
												: (state == DMState::stepError5) ? " ERR5:"
													: ":";
		debugPrintf("DM%c%s dir=%c steps=%" PRIu32 " next=%" PRIu32 " rev=%" PRIu32 " interval=%" PRIu32 " ssl=%" PRIu32 " A=%.4e B=%.4e C=%.4e dsf=%.4e tsf=%.1f\n",
						c, errText, (direction) ? 'F' : 'B', totalSteps, nextStep, reverseStartStep, stepInterval, segmentStepLimit,
							(double)pA, (double)pB, (double)pC, (double)distanceSoFar, (double)timeSoFar);
	}
	else
	{
		debugPrintf("DM%c: not moving\n", c);
	}
}

// This is called when currentSegment has just been changed to a new segment. Return true if there is a new segment to execute.
#if RP2040
__attribute__((section(".time_critical")))
#elif SAMC21
__attribute__((aligned(8)))			// insufficient RAM to put it there, so align it on a flash cache line boundary
#endif
bool DriveMovement::NewCartesianSegment() noexcept
{
	while (true)
	{
		if (currentSegment == nullptr)
		{
			return false;
		}

		// Work out the movement limit in steps
		pC = currentSegment->CalcCFromMmPerStep(mp.cart.effectiveMmPerStep);
		if (currentSegment->IsLinear())
		{
			// Set up pB, pC such that for forward motion, time = pB + pC * stepNumber
			pA = 0.0;																							// clear this to make debugging easier
			pB = currentSegment->CalcLinearB(distanceSoFar, timeSoFar);
			state = DMState::cartLinear;
		}
		else
		{
			// Set up pA, pB, pC such that for forward motion, time = pB + sqrt(pA + pC * stepNumber)
			pA = currentSegment->CalcNonlinearA(distanceSoFar);
			pB = currentSegment->CalcNonlinearB(timeSoFar);
			state = (currentSegment->IsAccelerating()) ? DMState::cartAccel : DMState::cartDecelNoReverse;
		}

		distanceSoFar += currentSegment->GetSegmentLength();
		timeSoFar += currentSegment->GetSegmentTime();

		segmentStepLimit = (currentSegment->GetNext() == nullptr) ? totalSteps + 1 : (uint32_t)(distanceSoFar * mp.cart.effectiveStepsPerMm) + 1;

#if 0	//DEBUG
		if (__get_BASEPRI() == 0)
		{
			debugPrintf("New cart seg: state %u A=%.4e B=%.4e C=%.4e ns=%" PRIu32 " ssl=%" PRIu32 "\n",
						(unsigned int)state, (double)pA, (double)pB, (double)pC, nextStep, segmentStepLimit);
		}
#endif
		if (nextStep < segmentStepLimit)
		{
			reverseStartStep = segmentStepLimit;						// need to set this so that CalcNextStepTime works properly
			return true;
		}

		currentSegment = currentSegment->GetNext();						// skip this segment
	}
}

// This is called for an extruder driver when currentSegment has just been changed to a new segment. Return true if there is a new segment to execute.
#if RP2040
__attribute__((section(".time_critical")))
#elif SAMC21
__attribute__((aligned(8)))			// insufficient RAM to put it there, so align it on a flash cache line boundary
#endif
bool DriveMovement::NewExtruderSegment() noexcept
{
	while (true)
	{
		if (currentSegment == nullptr)
		{
			return false;
		}

		const float startDistance = distanceSoFar;
		const float startTime = timeSoFar;

		distanceSoFar += currentSegment->GetSegmentLength();
		timeSoFar += currentSegment->GetSegmentTime();
		pC = currentSegment->CalcCFromMmPerStep(mp.cart.effectiveMmPerStep);
		if (currentSegment->IsLinear())
		{
			// Set up pB, pC such that for forward motion, time = pB + pC * stepNumber
			pA = 0.0;																							// clear this to make debugging easier
			pB = currentSegment->CalcLinearB(startDistance, startTime);
			state = DMState::cartLinear;
			reverseStartStep = segmentStepLimit = (int32_t)(distanceSoFar * mp.cart.effectiveStepsPerMm) + 1;
		}
		else
		{
			// Set up pA, pB, pC such that for forward motion, time = pB + sqrt(pA + pC * stepNumber)
			pA = currentSegment->CalcNonlinearA(startDistance, mp.cart.pressureAdvanceK);
			pB = currentSegment->CalcNonlinearB(startTime, mp.cart.pressureAdvanceK);
			distanceSoFar += currentSegment->GetNonlinearSpeedChange() * mp.cart.pressureAdvanceK;				// add the extra extrusion due to pressure advance to the extrusion done at the end of this move
			const int32_t netStepsAtSegmentEnd = (int32_t)floorf(distanceSoFar * mp.cart.effectiveStepsPerMm);	// we must round towards minus infinity because distanceSoFar may be negative
			const float endSpeed = currentSegment->GetNonlinearEndSpeed(mp.cart.pressureAdvanceK);
			if (currentSegment->IsAccelerating())
			{
				state = DMState::cartAccel;
				reverseStartStep = segmentStepLimit = netStepsAtSegmentEnd + 1;
			}
			else
			{
				// This is a decelerating segment. If it includes pressure advance then it may include reversal.
				if (endSpeed >= 0.0)
				{
					state = DMState::cartDecelNoReverse;					// this segment is forwards throughout
					reverseStartStep = segmentStepLimit = netStepsAtSegmentEnd + 1;
					CheckDirection(false);
				}
				else
				{
					const float startSpeed = currentSegment->GetNonlinearStartSpeed(mp.cart.pressureAdvanceK);
					if (startSpeed <= 0.0)
					{
						state = DMState::cartDecelReverse;					// this segment is reverse throughout
						reverseStartStep = nextStep;
						CheckDirection(true);
					}
					else
					{
						// This segment starts forwards and then reverses. Either or both of the forward and reverse segments may be small enough to need no steps.
						const float distanceToReverse = currentSegment->GetDistanceToReverse(startSpeed) + startDistance;
						const int32_t netStepsToReverse = (int32_t)(distanceToReverse * mp.cart.effectiveStepsPerMm);
						if (nextStep <= netStepsToReverse)
						{
							// There is at least one step before we reverse
							reverseStartStep = netStepsToReverse + 1;
							state = DMState::cartDecelForwardsReversing;
							CheckDirection(false);
						}
						else
						{
							// There is no significant forward phase, so start in reverse
							reverseStartStep = nextStep;					// they are probably equal anyway, but just in case...
							state = DMState::cartDecelReverse;
							CheckDirection(true);
						}
					}
					segmentStepLimit = (2 * reverseStartStep) - netStepsAtSegmentEnd - 1;
				}
			}
		}

#if 0	//DEBUG
		if (__get_BASEPRI() == 0)
		{
			debugPrintf("New ex seg: state %u A=%.4e B=%.4e C=%.4e ns=%" PRIi32 " ssl=%" PRIi32 " rss=%" PRIi32 "\n",
						(unsigned int)state, (double)pA, (double)pB, (double)pC, nextStep, segmentStepLimit, reverseStartStep);
		}
#endif
		if (nextStep < segmentStepLimit)
		{
			return true;
		}

		currentSegment = currentSegment->GetNext();							// skip this segment
	}
}

// Prepare this DM for a Cartesian axis move, returning true if there are steps to do
bool DriveMovement::PrepareCartesianAxis(const DDA& dda) noexcept
{
	distanceSoFar = 0.0;
	timeSoFar = 0.0;
	mp.cart.pressureAdvanceK = 0.0;
	// We can't use directionVector here because those values relate to Cartesian space, whereas we may be CoreXY etc.
	mp.cart.effectiveStepsPerMm = (float)totalSteps;	// because totalDistance = 1.0
	mp.cart.effectiveMmPerStep = 1.0/mp.cart.effectiveStepsPerMm;
	isDelta = false;
	isExtruder = false;
	currentSegment = dda.segments;
	nextStep = 1;									// must do this before calling NewCartesianSegment
	directionChanged = directionReversed = false;	// must clear these before we call NewCartesianSegment

	if (!NewCartesianSegment())
	{
		return false;
	}

	// Prepare for the first step
	nextStepTime = 0;
	stepsTakenThisSegment = 0;						// no steps taken yet since the start of the segment
	stepInterval = 0;								// to keep the debug output deterministic
	return CalcNextStepTimeFull(dda);				// calculate the scheduled time of the first step
}

// Prepare this DM for an extruder move, returning true if there are steps to do
// If there are no steps to do, set nextStep = 0 so that DDARing::CurrentMoveCompleted doesn't add any steps to the movement accumulator
// We have already generated the extruder segments and we know that there are some
// effStepsPerMm is the number of extruder steps needed per mm of totalDistance before we apply pressure advance
void DriveMovement::PrepareExtruder(const DDA& dda, float signedEffStepsPerMm) noexcept
{
	const float effStepsPerMm = fabsf(signedEffStepsPerMm);
	mp.cart.effectiveStepsPerMm = effStepsPerMm;
	mp.cart.effectiveMmPerStep = 1.0/effStepsPerMm;

	timeSoFar = 0.0;
	currentSegment = dda.segments;
	isDelta = false;
	isExtruder = true;
	nextStep = 1;									// must do this before calling NewExtruderSegment
	totalSteps = 0;									// we don't use totalSteps but set it to 0 to avoid random values being printed by DebugPrint
	directionChanged = directionReversed = false;	// must clear these before we call NewExtruderSegment

	// Prepare for the first step
	nextStepTime = 0;
	stepsTakenThisSegment = 0;						// no steps taken yet since the start of the segment
	stepInterval = 0;								// to keep the debug output deterministic

	// The remainder of the preparation can't be done until we start the move, because until then we don't know how much extrusion is pending
	state = DMState::extruderPendingPreparation;
}

// Finish preparing this DM for execution, returning true if there are any steps to do.
// A note on accumulating partial extruder steps:
// We must only accumulate partial steps when the extrusion is forwards. If we try to accumulate partial steps on reverse extrusion too,
// things go horribly wrong under particular circumstances. We use the pressure advance flag as a proxy for forward extrusion.
// This means that partial extruder steps don't get accumulated on a reprime move, but that is probably a good thing because it will
// behave in a similar way to a retraction move.
bool DriveMovement::LatePrepareExtruder(const DDA& dda) noexcept
{
	ExtruderShaper& shaper = moveInstance->GetExtruderShaper(drive);

	// distanceSoFar will accumulate the equivalent amount of totalDistance that the extruder moves forwards.
	// It would be equal to totalDistance if there was no pressure advance and no extrusion pending.
	if (dda.flags.usePressureAdvance)
	{
		const float extrusionPending = shaper.GetExtrusionPending();
		moveInstance->UpdateExtrusionPendingLimits(extrusionPending);
		distanceSoFar = extrusionPending * mp.cart.effectiveMmPerStep;
		mp.cart.pressureAdvanceK = shaper.GetKclocks();
	}
	else
	{
		mp.cart.pressureAdvanceK = 0.0;
		distanceSoFar =	0.0;
	}

	if (!NewExtruderSegment())						// if no steps to do
	{
		if (dda.flags.usePressureAdvance)
		{
			shaper.SetExtrusionPending(distanceSoFar * mp.cart.effectiveStepsPerMm);
		}
		state = DMState::idle;
		return false;								// quit if no steps to do
	}

	return CalcNextStepTimeFull(dda);				// calculate the scheduled time of the first step
}

// Version of fastSqrtf that allows for slightly negative operands caused by rounding error
static inline float fastLimSqrtf(float f) noexcept
{
	return (f <= 0.0) ? 0.0 : fastSqrtf(f);
}

// Calculate and store the time since the start of the move when the next step for the specified DriveMovement is due.
// We have already incremented nextStep and checked that it does not exceed totalSteps, so at least one more step is due
// Return true if all OK, false to abort this move because the calculation has gone wrong
#if SAMC21 || RP2040
__attribute__((section(".time_critical")))
#endif
bool DriveMovement::CalcNextStepTimeFull(const DDA &dda) noexcept
pre(nextStep <= totalSteps; stepsTillRecalc == 0)
{
	uint32_t shiftFactor = 0;										// assume single stepping
	{
		int32_t stepsToLimit = segmentStepLimit - nextStep;
		// If there are no more steps left in this segment, skip to the next segment and use single stepping
		if (stepsToLimit == 0)
		{
			currentSegment = currentSegment->GetNext();
			if (isExtruder)
			{
				{
					AtomicCriticalSectionLocker lock;										// avoid a race with GetNetStepsTaken called by filament monitor code
					nextStep = nextStep - 2 * (segmentStepLimit - reverseStartStep);		// set nextStep to the net steps taken in the original direction (this may make nextStep negative)
					CheckDirection(false);													// so that GetNetStepsTaken returns the correct value
				}
#if 0	//DEBUG
				DMState oldState = state;
#endif
				if (!NewExtruderSegment())
				{
					if (dda.flags.usePressureAdvance)
					{
						ExtruderShaper& shaper = moveInstance->GetExtruderShaper(drive);
						const int32_t netStepsDone = nextStep - 1;
						const float stepsCarriedForward = (distanceSoFar - (float)netStepsDone * mp.cart.effectiveMmPerStep) * mp.cart.effectiveStepsPerMm;
						shaper.SetExtrusionPending(stepsCarriedForward);
#if 0	//DEBUG
						if (stepsCarriedForward > 1.0 || stepsCarriedForward < -1.0)
						{
							debugPrintf("calc xpend=%.2f s=%u\n", (double)stepsCarriedForward, (unsigned int)oldState);
						}
#endif
					}
					return false;
				}
			}
			else
			{
				const bool more = NewCartesianSegment();
				if (!more)
				{
					state = DMState::stepError1;
					return false;
				}
			}
			// Leave shiftFactor set to 0 so that we compute a single step time, because the interval will have changed
			stepsTakenThisSegment = 1;								// this will be the first step in this segment
		}
		else if (stepsTakenThisSegment < 2)
		{
			// Reasons why we always use single stepping until we are on the third step in a segment:
			// 1. On the very first step of a move we don't know what the step interval is, so we must use single stepping for the first step.
			// 2. For extruders the step interval calculated for the very first step may be very small because of overdue extrusion,
			//    so we don't have a reliable step interval until we have calculated 2 steps.
			// 3. When starting a subsequent segment there may be a discontinuity due to rounding error,
			//    so the step interval calculated after the first step in a subsequent phase is not reliable.
			++stepsTakenThisSegment;
		}
		else
		{
			if (reverseStartStep < segmentStepLimit && nextStep < reverseStartStep)
			{
				stepsToLimit = reverseStartStep - nextStep;
			}

			if (stepsToLimit > 1 && stepInterval < DDA::MinCalcInterval)
			{
				if (stepInterval < DDA::MinCalcInterval/4 && stepsToLimit > 8)
				{
					shiftFactor = 3;							// octal stepping
				}
				else if (stepInterval < DDA::MinCalcInterval/2 && stepsToLimit > 4)
				{
					shiftFactor = 2;							// quad stepping
				}
				else if (stepsToLimit > 2)
				{
					shiftFactor = 1;							// double stepping
				}
			}
		}
	}

	stepsTillRecalc = (1u << shiftFactor) - 1u;					// store number of additional steps to generate

	float nextCalcStepTime;

	// Work out the time of the step
	switch (state)
	{
	case DMState::cartLinear:									// linear steady speed
		nextCalcStepTime = pB + (float)(nextStep + stepsTillRecalc) * pC;
		break;

	case DMState::cartAccel:									// Cartesian accelerating
		nextCalcStepTime = pB + fastLimSqrtf(pA + pC * (float)(nextStep + stepsTillRecalc));
		break;

	case DMState::cartDecelForwardsReversing:
		if (nextStep + stepsTillRecalc < reverseStartStep)
		{
			nextCalcStepTime = pB - fastLimSqrtf(pA + pC * (float)(nextStep + stepsTillRecalc));
			break;
		}

		CheckDirection(true);
		state = DMState::cartDecelReverse;
		// no break
	case DMState::cartDecelReverse:								// Cartesian decelerating, reverse motion. Convert the steps to int32_t because the net steps may be negative.
		{
			const int32_t netSteps = 2 * reverseStartStep - nextStep - 1;
			nextCalcStepTime = pB + fastLimSqrtf(pA + pC * (float)(netSteps - (int32_t)stepsTillRecalc));
		}
		break;

	case DMState::cartDecelNoReverse:							// Cartesian decelerating with no reversal
		nextCalcStepTime = pB - fastLimSqrtf(pA + pC * (float)(nextStep + stepsTillRecalc));
		break;

	default:
		return false;
	}

#if 0	//DEBUG
	if (std::isnan(nextCalcStepTime) || nextCalcStepTime < 0.0)
	{
		state = DMState::stepError5;
		distanceSoFar = nextCalcStepTime;					//DEBUG
		return false;
	}
#endif

	uint32_t iNextCalcStepTime = (uint32_t)nextCalcStepTime;

	if (iNextCalcStepTime > dda.clocksNeeded)
	{
		// The calculation makes this step late.
		// When the end speed is very low, calculating the time of the last step is very sensitive to rounding error.
		// So if this is the last step and it is late, bring it forward to the expected finish time.
		// Very rarely on a delta, the penultimate step may also be calculated late. Allow for that here in case it affects Cartesian axes too.
		// Don't use totalSteps here because it isn't valid for extruders
		if (currentSegment->GetNext() == nullptr)
		{
			iNextCalcStepTime = dda.clocksNeeded;
			const int32_t nextCalcStep = nextStep + (int32_t)stepsTillRecalc;
			const int32_t stepsLate = segmentStepLimit - nextCalcStep;
			if (stepsLate > maxStepsLate) { maxStepsLate = stepsLate; }
		}
		else
		{
			// We don't expect any segment except the last to have late steps
			state = DMState::stepError3;
			stepInterval = iNextCalcStepTime;				//DEBUG
			return false;
		}
	}

	// When crossing between movement phases with high microstepping, due to rounding errors the next step may appear to be due before the last one
	stepInterval = (iNextCalcStepTime > nextStepTime)
					? (iNextCalcStepTime - nextStepTime) >> shiftFactor	// calculate the time per step, ready for next time
					: 0;

#if 0	//DEBUG
	if (isExtruder && stepInterval < 20 /*&& nextStep + stepsTillRecalc + 1 < totalSteps*/)
	{
		state = DMState::stepError4;
		return false;
	}
#endif

	nextStepTime = iNextCalcStepTime - (stepsTillRecalc * stepInterval);
	return true;
}

#if SUPPORT_CLOSED_LOOP

// Get the current position relative to the start of this move, speed and acceleration. Units are microsteps and step clocks.
// Interrupts are disabled on entry and must remain disabled.
void DriveMovement::GetCurrentMotion(const DDA& dda, uint32_t ticksSinceStart, MotionParameters& mParams) noexcept
{
	const MoveSegment *ms = currentSegment;
	if (ms == nullptr)
	{
		// Drive was not commanded to move
		mParams.position = mParams.speed = mParams.acceleration = 0.0;
		return;
	}

	const float timeSinceMoveStart = (float)ticksSinceStart;
	for (;;)
	{
		const float segTimeRemaining = timeSoFar - timeSinceMoveStart;
		if (segTimeRemaining >= 0.0 || ms->GetNext() == nullptr)
		{
			// The current move segment is still in progress, or it is the last move segment and it has only just finished
			const float multiplier = (direction != directionReversed) ? mp.cart.effectiveStepsPerMm : -mp.cart.effectiveStepsPerMm;
			if (ms->IsLinear())
			{
				const float effectiveSpeed = dda.topSpeed * multiplier;
				mParams.position = (timeSinceMoveStart - pB) * effectiveSpeed;
				mParams.speed = effectiveSpeed;
				mParams.acceleration = 0.0;
			}
			else
			{
				const float effectiveAcceleration = ms->GetAcceleration() * multiplier;
				mParams.position = 0.5 * effectiveAcceleration * (fsquare(timeSinceMoveStart - pB) - pA);
				mParams.speed = effectiveAcceleration * (timeSinceMoveStart - pB);
				mParams.acceleration = effectiveAcceleration;
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

#endif

#endif	// SUPPORT_DRIVERS

// End
