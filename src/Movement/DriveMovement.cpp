/*
 * DriveMovement.cpp
 *
 *  Created on: 17 Jan 2015
 *      Author: David
 */

#include "DriveMovement.h"

#if SUPPORT_DRIVERS

#include "Move.h"
#include "StepTimer.h"
#include "MoveTiming.h"
#include <Math/Isqrt.h>
#include <Platform/Platform.h>

int32_t DriveMovement::maxStepsLate = 0;

void DriveMovement::Init(size_t drv) noexcept
{
	drive = (uint8_t)drv;
	state = DMState::idle;
	stepErrorType = 0;
	distanceCarriedForwards = 0.0;
	currentMotorPosition = positionAtSegmentStart = positionAtMoveStart = 0;
	movementAccumulator = 0;
	extruderPrinting = false;
	driversNormallyUsed = driversCurrentlyUsed = 0;
#if !SINGLE_DRIVER
	nextDM = nullptr;
#endif
	segments = nullptr;
	segmentFlags.InitNonPrinting();
#if SUPPORT_CLOSED_LOOP
	closedLoopControl.InitInstance();
#endif
}

void DriveMovement::DebugPrint() const noexcept
{
	const char c = drive + '0';
	if (state != DMState::idle)
	{
		debugPrintf("DM%c state=%u err=%u dir=%c next=%" PRIi32 " rev=%" PRIi32 " ssl=%" PRIi32 " sns=%" PRIi32 " interval=%" PRIu32 " q=%.4e t0=%.4e p=%.4e dcf=%.2f\n",
						c, (unsigned int)state, (unsigned int)stepErrorType, (direction) ? 'F' : 'B',
							nextStep, reverseStartStep, segmentStepLimit, netStepsThisSegment, stepInterval,
								(double)q, (double)t0, (double)p, (double)distanceCarriedForwards);
	}
	else
	{
		debugPrintf("DM%c: not moving\n", c);
	}
}

// Set up to schedule the first segment, returning true if an interrupt for this DM is needed. This is called only if the state is 'idle'.
bool DriveMovement::ScheduleFirstSegment() noexcept
{
	directionChanged = true;									// force the direction to be set up - this could be the first move or we may have switched between bed tramming and normal Z moves
	const uint32_t now = StepTimer::GetMovementTimerTicks();
	if (NewSegment(now) != nullptr)
	{
		if (state == DMState::starting)
		{
			return true;
		}
#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
		if (state == DMState::phaseStepping)
		{
			return false;
		}
#endif
		return CalcNextStepTimeFull(now);
	}
	return false;
}

#if USE_FIXED_STEP_TIMING

// Convert the movement parameters of a stepping constant-speed segment to fixed point for the per-step
// calculations in CalcNextStepTimeFull.
// Per step we evaluate (n - 1) * pFix + t0Fix where t0Fix holds the time of the first step,
// p + t0, computed in floating point (one add per segment). Using the first step time as the
// base instead of t0 matters: for a segment carrying a tiny distance combined with |dcf| close
// to 1, p and t0 are huge nearly-cancelling values (t0 = -dcf * p) whose fixed point images
// would both saturate, destroying their difference, while p + t0 = (1 - dcf) * p is an ordinary
// step time that always converts exactly. For any segment taking a second step, n * p <= duration
// implies p < 2^32, so (n - 1) * pFix cannot saturate for real segments; the saturation limit
// 2^satBits with satBits = 61 - floor(log2(ssl)) still bounds |(n - 1) * pFix| + |t0Fix| < 2^63
// even for garbage coefficients (infinities etc).
// Deliberately noinline and without the .time_critical attribute: this runs once per stepping segment,
// and executing it from flash keeps the RAM-resident part of NewSegment substantially smaller.
/*static*/ __attribute__((noinline)) void DriveMovement::CalcLinearFixCoeffs(motioncalc_t p, motioncalc_t t0, int32_t stepLimit, FixedStepCoeffs& out) noexcept
{
	uint32_t satBits = 61 - FloorLog2((uint32_t)stepLimit);
	if (satBits < 32) { satBits = 32; }				// keep the helper's precondition for absurdly high step counts
	out.pFix = FastMotionCalcToFix(p, StepTimeFracBits, satBits);
	out.t0Fix = FastMotionCalcToFix(p + t0, StepTimeFracBits, satBits);
}

// Convert the movement parameters of a stepping accelerating or decelerating segment to fixed point for the
// per-step calculations in CalcNextStepTimeFull.
// Per step we evaluate t0Fix +/- sqrt(qFix + (n * pFix >> pShift)) * 2^-sqrtRShift where the step
// number argument n satisfies |n| <= segmentStepLimit + 2 * reverseStartStep + 1 < 4 * max(ssl, rss)
// <= 2^lgN. Choosing sqrtScale = 59 - max(expQ, expP + lgN) makes |qFix| < 2^60 and the shifted
// product < 2^61, hence their sum < 2^63: no overflow. The scale is rounded down to an even value so
// that the square root maps 2^sqrtScale to 2^(sqrtScale/2) exactly; |t0| <= 2^26 here
// (NormaliseAndCheckLinear converts segments with larger t0 to linear ones), so t0Fix cannot saturate.
// pFix is stored with pShift extra scale bits, chosen so that p keeps at least 28 significant bits
// even when |q| >> |p * n| forces a coarse sqrtScale: with pScale = 61 - expP - lgN the product
// n * pFix stays below 2^63 while the truncation error of p amplified by the largest n stays below
// 2^(2 * lgN - 61) relative to p * n, i.e. far below the 2^-24 relative error of the float path.
// Deliberately noinline and without the .time_critical attribute, like CalcLinearFixCoeffs above.
/*static*/ __attribute__((noinline)) void DriveMovement::CalcAccelDecelFixCoeffs(motioncalc_t q, motioncalc_t p, motioncalc_t t0, int32_t stepLimit, int32_t revStartStep, FixedStepCoeffs& out) noexcept
{
	const uint32_t lgN = FloorLog2((uint32_t)max<int32_t>(stepLimit, revStartStep)) + 3;
	const int32_t expQ = (int32_t)MotionCalcBiasedExponent(q) - 127;
	const int32_t expP = (int32_t)MotionCalcBiasedExponent(p) - 127;
	int32_t sqrtScale = (59 - max<int32_t>(expQ, expP + (int32_t)lgN)) & ~1;
	if (sqrtScale > 46) { sqrtScale = 46; }			// no need for more fractional bits than this
	else if (sqrtScale < -64) { sqrtScale = -64; }	// only reachable if q or p is infinity or garbage
	int32_t pShiftBits = (61 - expP - (int32_t)lgN) - sqrtScale;	// >= 2 unless a clamp above was hit
	if (pShiftBits > 63) { pShiftBits = 63; }		// beyond this the p * n term is less than one ulp of qFix anyway
	else if (pShiftBits < 1) { pShiftBits = 1; }	// only reachable together with the -64 clamp, i.e. garbage coefficients; keep ShiftRight64's precondition
	out.qFix = FastMotionCalcToFix(q, sqrtScale, 62);
	// Saturating |pFix| at 2^(62 - lgN) keeps |n * pFix| < 2^63 even when the scales had to be clamped
	// above; with unclamped scales pFix stays below 2^(62 - lgN) anyway
	out.pFix = FastMotionCalcToFix(p, sqrtScale + pShiftBits, max<uint32_t>(62 - lgN, 32));
	out.t0Fix = FastMotionCalcToFix(t0, StepTimeFracBits, 62);
	out.sqrtRShift = (uint8_t)(StepTimeFracBits - (sqrtScale >> 1));
	out.pShift = (uint8_t)pShiftBits;
}

#endif	// USE_FIXED_STEP_TIMING

// This is called when we need to examine the segment list and prepare the head segment (if there is one) for execution.
// If there is no segment to execute, set our state to 'idle' and return nullptr.
// If there is a segment to execute but it isn't due to start for a while, set our state to 'starting', set nextStepTime to when the move is due to start or shortly before,
// set driversCurrentlyUsed to 0 to suppress the step pulse, and return the segment.
// If there is a segment ready to execute and it has steps, set up our movement parameters, copy the flags over, set the 'executing' flag in the segment, and return the segment.
// If there is a segment ready to execute but it involves zero steps, skip and free it and start again.
// This is called when currentSegment has just been changed to a new segment. Return true if there is a new segment to execute.
#if RP2040 || SAMC21
__attribute__((section(".time_critical")))
#endif
MoveSegment *DriveMovement::NewSegment(uint32_t now) noexcept
{
	positionAtSegmentStart = currentMotorPosition;

	while (true)
	{
		MoveSegment *seg = segments;				// capture volatile variable
		if (seg == nullptr)
		{
			segmentFlags.InitNonPrinting();
			state = DMState::idle;					// if we have been round this loop already then we will have changed the state, so reset it to idle
			return nullptr;
		}

		segmentFlags = seg->GetFlags();				// assume we are going to execute this segment, or at least generate an interrupt when it is due to begin

		if ((int32_t)(seg->GetStartTime() - now) > (int32_t)MoveTiming::MaximumMoveStartAdvanceClocks)
		{
			state = DMState::starting;				// the segment is not due to start for a while. To allow it to be changed meanwhile, generate an interrupt when it is due to start.
			driversCurrentlyUsed = 0;				// don't generate a step on that interrupt
			nextStepTime = seg->GetStartTime();		// this is when we want the interrupt
			return seg;
		}

		seg->SetExecuting();

		// Calculate the movement parameters
		netStepsThisSegment = (int32_t)(seg->GetLength() + distanceCarriedForwards);

#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
		if (closedLoopControl.IsClosedLoopEnabled())
		{
			u = seg->CalcU();
			state = DMState::phaseStepping;
			return seg;
		}
#endif

		bool newDirection;
		int32_t multiplier;
		motioncalc_t rawP;

		if (seg->NormaliseAndCheckLinear(distanceCarriedForwards, t0))
		{
			// Segment is linear
			rawP = seg->CalcLinearRecipU();
			newDirection = !std::signbit(seg->GetLength());
			multiplier = 2 * (int32_t)newDirection - 1;			// +1 or -1
			reverseStartStep = segmentStepLimit = 1 + netStepsThisSegment * multiplier;
			q = (motioncalc_t)0.0;								// to make the debug output consistent
			state = DMState::cartLinear;
		}
		else
		{
			// Segment has acceleration or deceleration
			// n = distanceCarriedForwards + u * t + 0.5 * a * t^2
			// Therefore 0.5 * t^2 + u * t/a + (distanceCarriedForwards - n)/a = 0
			// Therefore t = -u/a +/- sqrt((u/a)^2 - 2 * (distanceCarriedForwards - n)/a)
			// Calculate the t0, p and q coefficients for an accelerating or decelerating move such that t = t0 + sqrt(p*n + q) and set up the initial direction
			newDirection = !std::signbit(seg->GetA());			// assume accelerating motion
			multiplier = 2 * (int32_t)newDirection - 1;			// +1 or -1
			if (!IsPositive(t0))								// use IsPositive here, on Cortex-M0+ it's faster than a floating point compare
			{
				// The direction reversal is in the past so the initial direction is the direction of the acceleration
				segmentStepLimit = reverseStartStep = 1 + netStepsThisSegment * multiplier;
				state = DMState::cartAccel;
			}
			else
			{
				// The initial direction is opposite to the acceleration
				newDirection = !newDirection;
				multiplier = -multiplier;
				const int32_t netStepsInInitialDirection = netStepsThisSegment * multiplier;

				if (t0 < (motioncalc_t)seg->GetDuration())
				{
					// Reversal is potentially in this segment, but it may be before the first step, or may be beyond the last step we are going to take
					// It can also happen that the target end speed is zero but due to FP rounding error, distanceToReverse was just below netStepsInInitialDirection and got rounded down
					// Note, t0 = -u/a therefore u = a*t0 therefore u*t0^2 + 0.5*a*t0^2 = -a*t0^2 + 0.5*a*t0^2 = -0.5*a*t0^2
					const motioncalc_t rawDistanceToReverse = (motioncalc_t)-0.5 * seg->GetA() * msquare(t0) + distanceCarriedForwards;
#if SAMC21 || RP2040							// avoid floating point multiplication
					const motioncalc_t distanceToReverse = (newDirection) ? rawDistanceToReverse : -rawDistanceToReverse;
#else
					const motioncalc_t distanceToReverse = rawDistanceToReverse * multiplier;
#endif
					const int32_t stepsBeforeReverse = (int32_t)(distanceToReverse - (motioncalc_t)0.2);			// don't step and immediately step back again
					// Note, stepsBeforeReverse may be negative at this point
					if (stepsBeforeReverse <= netStepsInInitialDirection && netStepsInInitialDirection >= 0)
					{
						segmentStepLimit = reverseStartStep = netStepsInInitialDirection + 1;
						state = DMState::cartDecelNoReverse;
					}
					else if (stepsBeforeReverse <= 0)
					{
						// Reversal happens immediately
						newDirection = !newDirection;
#if !(SAMC21 || RP2040)															// we've finished with 'multiplier' on these processors
						multiplier = -multiplier;
#endif
						segmentStepLimit = reverseStartStep = 1 - netStepsInInitialDirection;
						state = DMState::cartAccel;
					}
					else
					{
						reverseStartStep = stepsBeforeReverse + 1;
						segmentStepLimit = 2 * reverseStartStep - netStepsInInitialDirection - 1;
						state = DMState::cartDecelForwardsReversing;
					}
				}
				else
				{
					// Reversal doesn't occur until after the end of this segment
					segmentStepLimit = reverseStartStep = netStepsInInitialDirection + 1;
					state = DMState::cartDecelNoReverse;
				}
			}
			rawP = (motioncalc_t)2.0/seg->GetA();
			q = msquare(t0) - rawP * distanceCarriedForwards;
#if 0
			if (std::isinf(q))
			{
				debugPrintf("t0=%.1f mult=%.1f dcf=%.3e a=%.4e\n", (double)t0, (double)multiplier, (double)distanceCarriedForwards, (double)seg->GetA());
			}
#endif
		}

#if SAMC21 || RP2040							// avoid floating point multiplication
		p = (newDirection) ? rawP : -rawP;
#else
		p = rawP * multiplier;
#endif

		nextStep = 1;
		if (nextStep < segmentStepLimit)
		{
			if (newDirection != direction)
			{
				directionChanged = true;
				direction = newDirection;
			}

			// Re-enable all drivers for this axis
			driversCurrentlyUsed = driversNormallyUsed;

#if USE_FIXED_STEP_TIMING
			// Convert the movement parameters to fixed point for the per-step calculations in CalcNextStepTimeFull.
			// Only segments that generate steps get here, so zero-step segments don't pay for the conversions,
			// and every stepping segment saves much more than they cost on its first step alone
			// (~150/~170 clocks per step, see CalcNextStepTimeFull). The conversion functions deliberately
			// live in flash: see the comments on their definitions.
			if (state == DMState::cartLinear)
			{
				FixedStepCoeffs fc;
				CalcLinearFixCoeffs(p, t0, segmentStepLimit, fc);
				t0Fix = fc.t0Fix;
				pFix = fc.pFix;
			}
			else
			{
				FixedStepCoeffs fc;
				CalcAccelDecelFixCoeffs(q, p, t0, segmentStepLimit, reverseStartStep, fc);
				t0Fix = fc.t0Fix;
				pFix = fc.pFix;
				qFix = fc.qFix;
				sqrtRShift = fc.sqrtRShift;
				pShift = fc.pShift;
			}
#endif

			// Update variables used by filament monitoring
			if (segmentFlags.isExtruder)
			{
				if (segmentFlags.nonPrintingMove)
				{
					extruderPrinting = false;
				}
				else if (!extruderPrinting)
				{
					extruderPrintingSince = millis();
					extruderPrinting = true;
				}
			}

#if 0	//DEBUG
			debugPrintf("New cart seg: state %u q=%.4e t0=%.4e p=%.4e ns=%" PRIi32 " ssl=%" PRIi32 "\n",
							(unsigned int)state, (double)q, (double)t0, (double)p, nextStep, segmentStepLimit);
#endif
			return seg;
		}

#if 0
		if (netStepsThisSegment != 0)
		{
			debugPrintf("Calc error! dcf=%.2f seg: ", (double)distanceCarriedForwards);
			seg->DebugPrint();
		}
#endif

#if 0
		debugPrintf("skipping seg: state %u q=%.4e t0=%.4e p=%.4e ns=%" PRIi32 " ssl=%" PRIi32 "\n",
						(unsigned int)state, (double)q, (double)t0, (double)p, nextStep, segmentStepLimit);
		seg->DebugPrint();
#endif
		motioncalc_t newDcf = distanceCarriedForwards + seg->GetLength();
		if (fabsm(newDcf) > 1.0)
		{
			LogStepError(7, (float)newDcf, seg);
			newDcf = constrain<motioncalc_t>(newDcf, -1.0, 1.0);	// to prevent the next segment erroring out
		}
		distanceCarriedForwards = newDcf;
		MoveSegment *oldSeg = seg;
		segments = seg = seg->GetNext();						// skip this segment
		MoveSegment::Release(oldSeg);
	}
}

// Version of fastSqrt that allows for slightly negative operands caused by rounding error
static inline motioncalc_t fastLimSqrtm(motioncalc_t f) noexcept
{
#if USE_DOUBLE_MOTIONCALC
	return (f >= (motioncalc_t)0.0) ? fastSqrtd(f) : (motioncalc_t)0.0;
#else
	return IsPositive(f) ? fastSqrtf(f) : 0.0;
#endif
}

#if USE_FIXED_STEP_TIMING

// Fixed point counterpart of fastLimSqrtm. Takes the operand as s * 2^sqrtScale (may be slightly negative due
// to rounding error, like the float version) and returns sqrt(s) * 2^StepTimeFracBits, i.e. Q40.24 step clocks.
// The square root itself is Qfplib's qfp_fsqrt (~67 clocks, RAM-resident, reached via fastSqrtf), which is much
// faster than a 64-bit integer square root on these cores; the conversions on either side cost ~30 + ~22 clocks
// against the qfp_fmul + two qfp_fadds (~200 clocks) that the float path spends around its identical sqrt call.
// Deliberately noinline and in flash to save RAM: executing the conversions from flash and reaching the
// RAM-resident square root through a veneer costs of the order of 100 clocks per accelerating/decelerating
// step calculation, which the per-step budget can afford; explicit noinline keeps the placement predictable
// (left inline, GCC partial-inlines just the sFix <= 0 guard and out-lines the body anyway).
__attribute__((noinline))
static int64_t FixLimSqrt(int64_t sFix, uint32_t rShift) noexcept
{
	if (sFix <= 0)
	{
		return 0;
	}
	return FastMotionCalcToFix(fastSqrtf(FastUint64ToMotionCalc((uint64_t)sFix)), (int32_t)rShift, 62);
}

#endif	// USE_FIXED_STEP_TIMING

// Tell the Move class that we had a step error. This always returns false so that CalcNextStepTimeFull can tail-chain to it.
bool DriveMovement::LogStepError(uint8_t type, float info, const MoveSegment *seg) noexcept
{
	state = DMState::stepError;
	stepErrorType = type;
	debugPrintf("Code %u move error: info=%.3g, seg: ", type, (double)info);
	if (seg != nullptr)
	{
		seg->DebugPrint();
	}
	debugPrintf("\n");
	moveInstance->LogStepError(type);
	return false;
}

// Calculate and store the time since the start of the move when the next step for the specified DriveMovement is due.
// We have already incremented nextStep and checked that it does not exceed totalSteps, so at least one more step is due
// Return true if all OK, false to abort this move because the calculation has gone wrong
#if SAMC21 || RP2040
__attribute__((section(".time_critical")))
#endif
bool DriveMovement::CalcNextStepTimeFull(uint32_t now) noexcept
pre(stepsTillRecalc == 0; segments != nullptr)
{
	MoveSegment *currentSegment = segments;							// capture volatile variable
	uint32_t shiftFactor = 0;										// assume single stepping
	{
		int32_t stepsToLimit = segmentStepLimit - nextStep;
		if (stepsToLimit == 1 && currentSegment->GetNext() == nullptr && !currentSegment->GetFlags().isExtruder && reverseStartStep != nextStep)
		{
			// It's an axis and we are soon to stop movement, so we should end on an exact microstep.
			// Check whether taking the last step would end up going a little too far or not quite far enough
			const motioncalc_t provisionalDistanceCarriedForwards = distanceCarriedForwards + currentSegment->GetLength() - (motioncalc_t)netStepsThisSegment;
			if (fabsm(provisionalDistanceCarriedForwards) < 0.05)
			{
				currentSegment->AdjustLength(-provisionalDistanceCarriedForwards);				// just correct the segment length
			}
			else if (provisionalDistanceCarriedForwards > (motioncalc_t)0.95)
			{
				currentSegment->AdjustLength((motioncalc_t)1.0 - provisionalDistanceCarriedForwards);	// adjust the segment length slightly
				if (direction)
				{
					// Take 1 more step
					++netStepsThisSegment;														// add 1 step to it
					const int32_t oldSsl = segmentStepLimit;
					segmentStepLimit = oldSsl + 1;												// increase the number of steps due
					if (reverseStartStep == oldSsl) { reverseStartStep = oldSsl + 1; }			// if we didn't reverse already, make sure we don't reverse when we take the extra step
					++stepsToLimit;																// we can take 1 more step
				}
				else
				{
					// Take 1 less step
					--segmentStepLimit;
					--netStepsThisSegment;
					stepsToLimit = 0;
				}
			}
			else if (provisionalDistanceCarriedForwards < -(motioncalc_t)0.95)
			{
				currentSegment->AdjustLength(-(motioncalc_t)1.0 - provisionalDistanceCarriedForwards);	// adjust the segment length slightly
				if (direction)
				{
					// Take 1 less step
					--segmentStepLimit;
					--netStepsThisSegment;
					stepsToLimit = 0;
				}
				else
				{
					--netStepsThisSegment;														// add 1 step to it in the backwards direction
					const int32_t oldSsl = segmentStepLimit;
					segmentStepLimit = oldSsl + 1;												// increase the number of steps due
					if (reverseStartStep == oldSsl) { reverseStartStep = oldSsl + 1; }			// if we didn't reverse already, make sure we don't reverse now
					++stepsToLimit;
				}
			}
		}

		// If there are no more steps left in this segment, skip to the next segment and use single stepping
		if (stepsToLimit <= 0)
		{
			distanceCarriedForwards += currentSegment->GetLength() - (motioncalc_t)netStepsThisSegment;
#if !(SAMC21 || RP2040)												// this check is expensive on these processors
			if (fabsm(distanceCarriedForwards) > (motioncalc_t)1.0)
			{
				return LogStepError(5, (float)distanceCarriedForwards, currentSegment);
			}
#endif
			if (currentMotorPosition - positionAtSegmentStart != netStepsThisSegment)
			{
				return LogStepError(6, 0.0, currentSegment);
			}

			movementAccumulator += netStepsThisSegment;				// update the amount of extrusion for filament monitors
			segments = currentSegment->GetNext();
			const uint32_t prevEndTime = currentSegment->GetStartTime() + currentSegment->GetDuration();
			MoveSegment::Release(currentSegment);
			currentSegment = NewSegment(now);
			if (currentSegment == nullptr)
			{
				return false;										// the call to NewSegment has already set the state to idle
			}

			if (state == DMState::starting)
			{
				return true;										// the call to NewSegment has already set up the interrupt time
			}

			if (unlikely((int32_t)(currentSegment->GetStartTime() - prevEndTime) < -10))
			{
				return LogStepError(1, (float)(int32_t)(currentSegment->GetStartTime() - prevEndTime), currentSegment);
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
			if (reverseStartStep < segmentStepLimit && nextStep <= reverseStartStep)
			{
				stepsToLimit = reverseStartStep - nextStep;
			}

			if (stepsToLimit > 1 && stepInterval < MoveTiming::MinCalcInterval)
			{
				if (stepInterval < MoveTiming::MinCalcInterval/4 && stepsToLimit > 8)
				{
					shiftFactor = 3;							// octal stepping
				}
				else if (stepInterval < MoveTiming::MinCalcInterval/2 && stepsToLimit > 4)
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

	// Work out the time of the step. The state machine below is shared between the fixed point and the floating
	// point implementations: only the arithmetic kernels differ, selected here, so that changes to the state
	// handling need to be made only once. In the fixed point case the linear kernel takes n - 1 because t0Fix
	// holds the time of the first step rather than t0 (see CalcLinearFixCoeffs).
#if USE_FIXED_STEP_TIMING
	typedef int64_t steptime_t;
# define STEP_TIME_LINEAR(n)	MulStepByCoeff((n) - 1, pFix)
# define STEP_TIME_SQRT(n)		FixLimSqrt(qFix + ShiftRight64(MulStepByCoeff((n), pFix), pShift), sqrtRShift)
# define STEP_TIME_BASE			t0Fix
#else
	typedef motioncalc_t steptime_t;
# define STEP_TIME_LINEAR(n)	((motioncalc_t)(n) * p)
# define STEP_TIME_SQRT(n)		fastLimSqrtm(q + p * (motioncalc_t)(n))
# define STEP_TIME_BASE			t0
#endif

	steptime_t tCalc;

	switch (state)
	{
	case DMState::cartLinear:									// linear steady speed
		tCalc = STEP_TIME_LINEAR(nextStep + (int32_t)stepsTillRecalc);
		break;

	case DMState::cartAccel:									// Cartesian accelerating
		tCalc = STEP_TIME_SQRT(nextStep + (int32_t)stepsTillRecalc);
		break;

	case DMState::cartDecelForwardsReversing:
		if (nextStep + (int32_t)stepsTillRecalc < reverseStartStep)
		{
			tCalc = -STEP_TIME_SQRT(nextStep + (int32_t)stepsTillRecalc);
			break;
		}

		direction = !direction;
		directionChanged = true;
		state = DMState::cartDecelReverse;
		// no break
	case DMState::cartDecelReverse:								// Cartesian decelerating, reverse motion. Convert the steps to int32_t because the net steps may be negative.
		{
			const int32_t netSteps = 2 * reverseStartStep - nextStep - 1;
			tCalc = STEP_TIME_SQRT(netSteps - (int32_t)stepsTillRecalc);
		}
		break;

	case DMState::cartDecelNoReverse:							// Cartesian decelerating with no reversal
		tCalc = -STEP_TIME_SQRT(nextStep + (int32_t)stepsTillRecalc);
		break;

	default:
#if SEGMENT_DEBUG
		debugPrintf("DMstate %u, quitting\n", (unsigned int)state);
#endif
		return LogStepError(4, (float)state, currentSegment);
	}

	tCalc += STEP_TIME_BASE;

#undef STEP_TIME_LINEAR
#undef STEP_TIME_SQRT
#undef STEP_TIME_BASE

	uint32_t iNextCalcStepTime;

	// Check that the next step time is reasonable
#if USE_FIXED_STEP_TIMING
	if (unlikely(tCalc < 0))
	{
		// If we are carrying almost a whole step forward to this segment so that the first step is due almost immediately,
		// then due to rounding error we can get a slightly negative value here. As in the float path we allow
		// two step clocks of error, plus an allowance that scales with the magnitude of the base time: near a speed
		// reversal the float32 rounding error of q = t0^2 - p * dcf propagated through the square root is of the
		// order of |t0| * 2^-24 clocks, which exceeds 2 clocks when |t0| is large. (The float path is subject
		// to the same coefficient error but partially cancels it with its own per-step rounding, so its fixed
		// 2-clock allowance rarely trips; without this scaling term the more accurate fixed point value could land
		// just beyond -2 where the float value landed just inside, aborting a move the float path would execute.)
		const int64_t allowance = ((int64_t)2 << StepTimeFracBits) + (((t0Fix < 0) ? -t0Fix : t0Fix) >> 22);
		if (tCalc < -allowance)
		{
			return LogStepError(2, (float)(int32_t)(tCalc >> StepTimeFracBits), currentSegment);
		}
		iNextCalcStepTime = 0;
	}
	else
	{
		// A value of 2^32 or more cannot be a valid step time; saturate instead of letting the cast wrap, so that
		// the late-step check below clamps it to the segment duration
		iNextCalcStepTime = (tCalc >= ((int64_t)1 << (32 + StepTimeFracBits))) ? 0xFFFFFFFFu
								: (uint32_t)(uint64_t)(tCalc >> StepTimeFracBits);
	}
#else
# if (SAMC21 || RP2040) && !USE_DOUBLE_MOTIONCALC
	// The FP library we use on Cortext-M0+ MCUs doesn't support NaNs so there is no point in testing for them
	if (unlikely(std::signbit(tCalc)))
# else
	if (unlikely(std::isnan(tCalc)))
	{
		return LogStepError(2, (float)tCalc, currentSegment);
	}

	if (unlikely(tCalc < (motioncalc_t)0.0))
# endif
	{
		// If we are carrying almost a whole step forward to this segment so that the first step is due almost immediately,
		// then due to floating point rounding error we can get a slightly negative value here for the step time.
		// The only value we have had reported so far is -0.00195 but we now allow up to two step clocks of error.
		if (tCalc < -(motioncalc_t)2.0)
		{
			return LogStepError(2, (float)tCalc, currentSegment);
		}
		iNextCalcStepTime = 0;
	}
	else
	{
		iNextCalcStepTime = (uint32_t)tCalc;
	}
#endif	// USE_FIXED_STEP_TIMING

	if (iNextCalcStepTime > currentSegment->GetDuration())
	{
		// The calculation makes this step late.
		// When the end speed is very low, calculating the time of the last step is very sensitive to rounding error.
		// So if this is the last step and it is late, bring it forward to the expected finish time.
		// 2023-12-06: we now allow any step to be late but we record the maximum number.
		// 2024-040-5: we now allow steps to be late on any segment, not just the last one, because a segment may be 0 or 1 step long and on deltas the last 2 steps may be calculated late.
		iNextCalcStepTime = currentSegment->GetDuration();
		const int32_t nextCalcStep = nextStep + (int32_t)stepsTillRecalc;
		const int32_t stepsLate = segmentStepLimit - nextCalcStep;
		if (stepsLate > maxStepsLate) { maxStepsLate = stepsLate; }
	}

	iNextCalcStepTime += currentSegment->GetStartTime();
	if (nextStep == 1)
	{
		nextStepTime = iNextCalcStepTime;
	}
	else
	{
		// When crossing between movement phases with high microstepping, due to rounding errors the next step may appear to be due before the last one
		const int32_t interval = (int32_t)(iNextCalcStepTime - nextStepTime);
		if (interval > 0)
		{
			stepInterval = (uint32_t)interval >> shiftFactor;		// calculate the time per step, ready for next time
#if 0	//debug
			if (interval < minStepInterval) { minStepInterval = interval; }
#endif
		}
		else
		{
			stepInterval = 0;
		}

#if 0	//DEBUG
		if (isExtruder && stepInterval < 20 /*&& nextStep + stepsTillRecalc + 1 < totalSteps*/)
		{
			state = DMState::stepError1;
			return LogStepError();
		}
#endif

		nextStepTime = iNextCalcStepTime - (stepsTillRecalc * stepInterval);
	}

	return true;
}

// If the driver is moving, stop it and release the segments. Caller will remote it from the active list and disable interrupts before calling this.
void DriveMovement::StopDriverFromRemote() noexcept
{
	if (state != DMState::idle)
	{
		state = DMState::idle;
		MoveSegment *seg = nullptr;
		std::swap(seg, const_cast<MoveSegment*&>(segments));
		MoveSegment::ReleaseAll(seg);
	}
}

#endif	// SUPPORT_DRIVERS

// End
