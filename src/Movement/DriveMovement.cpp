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

#if SHADOW_CACHE_DIAGNOSTICS
uint32_t DriveMovement::shadowCacheHits = 0;
uint32_t DriveMovement::shadowCacheMisses = 0;
uint32_t DriveMovement::maxCacheSkip = 0;
uint32_t DriveMovement::maxIsrSkip = 0;
#endif

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
	segmentsTail = nullptr;
	segHint = nullptr;
	segmentFlags.InitNonPrinting();
#if USE_SHADOW_SEGMENTS
	shadowHead = 0;
	shadowGen = 0;
	FlushShadows();
#endif
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

// The result of analysing one segment against the distance carried forwards that it will start with
struct SegAnalysis
{
	bool direction;										// the initial movement direction
	DMState state;										// the initial DM state
	int32_t stepLimit;									// the value for segmentStepLimit
	int32_t reverseStartStep;							// the value for reverseStartStep
	motioncalc_t p;										// the p coefficient, with the direction sign applied
	motioncalc_t q;										// the q coefficient (0.0 for linear segments, to make the debug output consistent)
};

// Analyse one segment: calculate the p and q coefficients (t = t0 + sqrt(p*n + q) for accelerating or decelerating
// segments), the step limits and the initial direction and state. This is the single copy of the analysis code, shared
// between NewSegment and PrepareShadowChunk (which works on a copy of the segment fields), so that both compute
// bit-identical results from the same inputs.
// segIsLinear and t0 are the results of MoveSegment::NormaliseAndCheckLinear/CheckLinearCore on the same values.
static inline void AnalyseSegment(bool segIsLinear, motioncalc_t t0, int32_t netSteps, motioncalc_t length, motioncalc_t a, uint32_t duration, motioncalc_t distanceCarriedForwards, SegAnalysis& out) noexcept
{
	bool newDirection;
	int32_t multiplier;
	motioncalc_t rawP;

	if (segIsLinear)
	{
		// Segment is linear
		rawP = (motioncalc_t)duration/length;					// as MoveSegment::CalcLinearRecipU
		newDirection = !std::signbit(length);
		multiplier = 2 * (int32_t)newDirection - 1;			// +1 or -1
		out.reverseStartStep = out.stepLimit = 1 + netSteps * multiplier;
		out.q = (motioncalc_t)0.0;								// to make the debug output consistent
		out.state = DMState::cartLinear;
	}
	else
	{
		// Segment has acceleration or deceleration
		// n = distanceCarriedForwards + u * t + 0.5 * a * t^2
		// Therefore 0.5 * t^2 + u * t/a + (distanceCarriedForwards - n)/a = 0
		// Therefore t = -u/a +/- sqrt((u/a)^2 - 2 * (distanceCarriedForwards - n)/a)
		// Calculate the t0, p and q coefficients for an accelerating or decelerating move such that t = t0 + sqrt(p*n + q) and set up the initial direction
		newDirection = !std::signbit(a);			// assume accelerating motion
		multiplier = 2 * (int32_t)newDirection - 1;			// +1 or -1
		if (!IsPositive(t0))								// use IsPositive here, on Cortex-M0+ it's faster than a floating point compare
		{
			// The direction reversal is in the past so the initial direction is the direction of the acceleration
			out.stepLimit = out.reverseStartStep = 1 + netSteps * multiplier;
			out.state = DMState::cartAccel;
		}
		else
		{
			// The initial direction is opposite to the acceleration
			newDirection = !newDirection;
			multiplier = -multiplier;
			const int32_t netStepsInInitialDirection = netSteps * multiplier;

			if (t0 < (motioncalc_t)duration)
			{
				// Reversal is potentially in this segment, but it may be before the first step, or may be beyond the last step we are going to take
				// It can also happen that the target end speed is zero but due to FP rounding error, distanceToReverse was just below netStepsInInitialDirection and got rounded down
				// Note, t0 = -u/a therefore u = a*t0 therefore u*t0^2 + 0.5*a*t0^2 = -a*t0^2 + 0.5*a*t0^2 = -0.5*a*t0^2
				const motioncalc_t rawDistanceToReverse = (motioncalc_t)-0.5 * a * msquare(t0) + distanceCarriedForwards;
#if SAMC21 || RP2040							// avoid floating point multiplication
				const motioncalc_t distanceToReverse = (newDirection) ? rawDistanceToReverse : -rawDistanceToReverse;
#else
				const motioncalc_t distanceToReverse = rawDistanceToReverse * multiplier;
#endif
				const int32_t stepsBeforeReverse = (int32_t)(distanceToReverse - (motioncalc_t)0.2);			// don't step and immediately step back again
				// Note, stepsBeforeReverse may be negative at this point
				if (stepsBeforeReverse <= netStepsInInitialDirection && netStepsInInitialDirection >= 0)
				{
					out.stepLimit = out.reverseStartStep = netStepsInInitialDirection + 1;
					out.state = DMState::cartDecelNoReverse;
				}
				else if (stepsBeforeReverse <= 0)
				{
					// Reversal happens immediately
					newDirection = !newDirection;
#if !(SAMC21 || RP2040)															// we've finished with 'multiplier' on these processors
					multiplier = -multiplier;
#endif
					out.stepLimit = out.reverseStartStep = 1 - netStepsInInitialDirection;
					out.state = DMState::cartAccel;
				}
				else
				{
					out.reverseStartStep = stepsBeforeReverse + 1;
					out.stepLimit = 2 * out.reverseStartStep - netStepsInInitialDirection - 1;
					out.state = DMState::cartDecelForwardsReversing;
				}
			}
			else
			{
				// Reversal doesn't occur until after the end of this segment
				out.stepLimit = out.reverseStartStep = netStepsInInitialDirection + 1;
				out.state = DMState::cartDecelNoReverse;
			}
		}
		rawP = (motioncalc_t)2.0/a;
		out.q = msquare(t0) - rawP * distanceCarriedForwards;
#if 0
		if (std::isinf(out.q))
		{
			debugPrintf("t0=%.1f mult=%.1f dcf=%.3e a=%.4e\n", (double)t0, (double)multiplier, (double)distanceCarriedForwards, (double)a);
		}
#endif
	}

#if SAMC21 || RP2040							// avoid floating point multiplication
	out.p = (newDirection) ? rawP : -rawP;
#else
	out.p = rawP * multiplier;
#endif
	out.direction = newDirection;
}

#if USE_SHADOW_SEGMENTS

// Prepare the movement parameters of one more upcoming stepping segment (and any run of zero-step segments preceding
// it) into a free shadow slot, taking the coefficient float maths for that segment boundary out of the step ISR.
// The preparation is possible because the distanceCarriedForwards each upcoming segment starts with is fully determined
// in advance: it changes only at segment boundaries, by dcf += length - netSteps with netSteps = int(length + dcf), so
// the whole chain follows from the executing segment's values (which cannot change while it has a successor). The chain
// below uses exactly the operations of the corresponding updates in NewSegment and CalcNextStepTimeFull, and the
// analysis is the same single copy of code that NewSegment uses (AnalyseSegment/CheckLinearCore), so all stored values
// are bit-identical to what the normal path would compute.
// This runs with interrupts enabled while the step ISR may release segments, so each segment is copied under a brief
// interrupt-disabled window, and the result is committed only if shadowGen is unchanged, i.e. no segment was released,
// no slot consumed or flushed, and no list modification made meanwhile. Correctness never depends on the preparation:
// any list change invalidates the affected slots and the boundary then falls back to the normal path.
__attribute__((noinline)) bool DriveMovement::PrepareShadowChunk() noexcept
{
	// Values captured about a segment with interrupts disabled, so that the float maths can run on a consistent copy
	struct SegSnapshot
	{
		uint32_t startTime;
		uint32_t duration;
		motioncalc_t distance;
		motioncalc_t a;
		const MoveSegment *next;
	};

	const MoveSegment *cursor;								// the first segment of the run we will prepare
	motioncalc_t dcf = (motioncalc_t)0.0;					// distanceCarriedForwards at the start of *cursor
	unsigned int slotIdx;
	uint32_t gen0;
	bool anchorIsExecutingHead = false;
	motioncalc_t anchorLen = (motioncalc_t)0.0;
	motioncalc_t anchorDcf = (motioncalc_t)0.0;
	int32_t anchorNetSteps = 0;

#if SUPPORT_PHASE_STEPPING || SUPPORT_CLOSED_LOOP
	if (closedLoopControl.IsClosedLoopEnabled())
	{
		return false;										// segments are not executed step-by-step in closed loop mode, and NewSegment takes the phase stepping exit before the slot check
	}
#endif

	{
		const uint32_t oldFlags = IrqSave();
		gen0 = shadowGen;

		// Find the first free slot in ring order; the last valid slot (if any) anchors the dcf chain
		const ShadowSlot *lastValid = nullptr;
		unsigned int k;
		for (k = 0; k < NumShadowSlots; ++k)
		{
			const ShadowSlot& s = shadowSlots[(shadowHead + k) % NumShadowSlots];
			if (s.seg == nullptr)
			{
				break;
			}
			lastValid = &s;
		}
		if (k == NumShadowSlots)
		{
			IrqRestore(oldFlags);
			return false;									// all slots are already prepared
		}
		slotIdx = (shadowHead + k) % NumShadowSlots;

		if (lastValid != nullptr)
		{
			cursor = lastValid->seg->GetNext();				// safe: a segment covered by a valid slot has not been released
			dcf = lastValid->dcfAfterSeg;
		}
		else
		{
			MoveSegment *const head = segments;
			if (head == nullptr)
			{
				IrqRestore(oldFlags);
				return false;
			}
			if (head->GetFlags().executing)
			{
				// The executing head's dcf update is float maths, so capture the inputs and do the arithmetic with interrupts re-enabled
				anchorIsExecutingHead = true;
				anchorDcf = distanceCarriedForwards;
				anchorLen = head->GetLength();
				anchorNetSteps = netStepsThisSegment;
				cursor = head->GetNext();
			}
			else
			{
				cursor = head;								// the head segment has not started executing yet (DM idle or starting)
				dcf = distanceCarriedForwards;
			}
		}
		IrqRestore(oldFlags);
	}

	if (cursor == nullptr)
	{
		return false;
	}
	if (anchorIsExecutingHead)
	{
		dcf = anchorDcf + (anchorLen - (motioncalc_t)anchorNetSteps);	// exactly the end-of-segment update in CalcNextStepTimeFull
	}

	ShadowSlot local;
	local.chainHead = cursor;
	unsigned int numSlivers = 0;
	uint32_t prevEndTime = 0;								// only used once numSlivers != 0
	for (;;)
	{
		SegSnapshot snap;
		{
			const uint32_t oldFlags = IrqSave();
			snap.startTime = cursor->GetStartTime();
			snap.duration = cursor->GetDuration();
			snap.distance = cursor->GetLength();
			snap.a = cursor->GetA();
			snap.next = cursor->GetNext();
			IrqRestore(oldFlags);
		}

		if (numSlivers != 0 && snap.startTime != prevEndTime)
		{
			return false;									// a gap after a sliver: the normal path waits for the start time then, so don't span it
		}

		const int32_t netSteps = (int32_t)(snap.distance + dcf);	// exactly the netStepsThisSegment calculation in NewSegment

		motioncalc_t sT0;
		const bool segIsLinear = (CheckLinearCore(snap.a, snap.duration, snap.distance, dcf, sT0) != 0);
		SegAnalysis an;
		AnalyseSegment(segIsLinear, sT0, netSteps, snap.distance, snap.a, snap.duration, dcf, an);

		if (an.stepLimit <= 1)
		{
			// A zero-step segment: fold it into the run as a sliver
			const motioncalc_t newDcf = dcf + snap.distance;	// exactly the update the skip path in NewSegment makes
			if (fabsm(newDcf) > 1.0)
			{
				return false;								// this would be a step error; leave it to the normal path to detect and report
			}
			dcf = newDcf;
			if (snap.next == nullptr || numSlivers >= 250)
			{
				return false;								// no stepping segment follows (yet); leave the run to the normal path
			}
			++numSlivers;
			prevEndTime = snap.startTime + snap.duration;
			cursor = snap.next;
			continue;
		}

		// A stepping segment: complete the slot
		local.seg = cursor;
		local.t0 = sT0;
		local.p = an.p;
		local.q = an.q;
		local.numSlivers = (uint8_t)numSlivers;
		local.state = an.state;
		local.direction = an.direction;
		local.netSteps = netSteps;
		local.stepLimit = an.stepLimit;
		local.reverseStartStep = an.reverseStartStep;
		local.dcfAtSeg = dcf;
		local.dcfAfterSeg = dcf + (snap.distance - (motioncalc_t)netSteps);	// exactly the end-of-segment update in CalcNextStepTimeFull
		break;
	}

	// Commit the slot, unless the ISR released segments or consumed/flushed slots while we were computing
	bool committed = false;
	{
		const uint32_t oldFlags = IrqSave();
		ShadowSlot& s = shadowSlots[slotIdx];
		if (shadowGen == gen0 && s.seg == nullptr)
		{
			s = local;										// interrupts are off, so the ISR cannot see a partially-written slot
			committed = true;
		}
		IrqRestore(oldFlags);
	}
	return committed;
}

// Flush all prepared slots unless every one of them covers only segments that end at or before startTime, which
// segments added from startTime onwards cannot affect. Caller must have the step interrupt shut out.
// shadowGen is bumped unconditionally so that a preparation in flight that walked segments the caller is about to modify discards at commit.
void DriveMovement::InvalidateShadowsFrom(uint32_t startTime) noexcept
{
	++shadowGen;
	for (unsigned int k = 0; k < NumShadowSlots; ++k)
	{
		const ShadowSlot& s = shadowSlots[(shadowHead + k) % NumShadowSlots];
		if (s.seg == nullptr)
		{
			break;
		}
		if ((int32_t)(startTime - (s.seg->GetStartTime() + s.seg->GetDuration())) < 0)
		{
			FlushShadows();
			break;
		}
	}
}

#endif	// USE_SHADOW_SEGMENTS

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

#if SHADOW_CACHE_DIAGNOSTICS
	unsigned int skipRun = 0;								// diagnostic: the number of zero-step segments this call has skipped without a prepared slot
#endif
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

#if USE_SHADOW_SEGMENTS
		{
			ShadowSlot& slot = shadowSlots[shadowHead];
			if (slot.seg != nullptr && seg == slot.chainHead)
			{
				if (slot.numSlivers != 0)
				{
					// A run of zero-step segments precedes the prepared stepping segment. Release them all without any per-segment
					// float maths: the preparation computed the resulting distanceCarriedForwards bit-identically and verified its error bound.
					unsigned int n = slot.numSlivers;
#if SHADOW_CACHE_DIAGNOSTICS
					if (n > maxCacheSkip) { maxCacheSkip = n; }
					shadowCacheHits += n;						// each sliver released from the slot counts as a hit
#endif
					do
					{
						MoveSegment *const oldSeg = seg;
						segments = seg = seg->GetNext();
						if (segHint == oldSeg) { segHint = nullptr; }	// (a stepping segment always follows the slivers, so the list cannot empty here)
						++shadowGen;
						MoveSegment::Release(oldSeg);
						--n;
					} while (n != 0 && seg != nullptr);
					distanceCarriedForwards = slot.dcfAtSeg;
					if (seg == slot.seg)
					{
						slot.chainHead = seg;					// the slivers are done with; next time round the loop the prepared parameters apply
						slot.numSlivers = 0;
					}
					else
					{
						FlushShadows();							// the list is not what the preparation saw; fall back to the normal path
					}
					continue;									// go round the loop again so that the start time check runs for the next segment
				}

				// seg is the prepared stepping segment: apply the prepared parameters instead of doing the float maths below;
				// distanceCarriedForwards already has exactly the value the preparation used
				netStepsThisSegment = slot.netSteps;
				segmentStepLimit = slot.stepLimit;
				reverseStartStep = slot.reverseStartStep;
				state = slot.state;
				t0 = slot.t0;
				p = slot.p;
				q = slot.q;
				nextStep = 1;
				if (slot.direction != direction)
				{
					directionChanged = true;
					direction = slot.direction;
				}
				driversCurrentlyUsed = driversNormallyUsed;		// re-enable all drivers for this axis
				slot.seg = nullptr;								// consume the slot; Move::Spin on the MAIN task refills it
				shadowHead = (shadowHead + 1u == NumShadowSlots) ? 0 : shadowHead + 1u;	// no % here: modulo by 3 would call the division function in flash
				++shadowGen;
#if SHADOW_CACHE_DIAGNOSTICS
				++shadowCacheHits;
#endif

				// Update variables used by filament monitoring, as the normal path below does
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
				return seg;
			}
		}
#endif

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

		const bool segIsLinear = seg->NormaliseAndCheckLinear(distanceCarriedForwards, t0);
		SegAnalysis an;
		AnalyseSegment(segIsLinear, t0, netStepsThisSegment, seg->GetLength(), seg->GetA(), seg->GetDuration(), distanceCarriedForwards, an);
		const bool newDirection = an.direction;
		segmentStepLimit = an.stepLimit;
		reverseStartStep = an.reverseStartStep;
		state = an.state;
		q = an.q;
		p = an.p;

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

#if SHADOW_CACHE_DIAGNOSTICS
			++shadowCacheMisses;						// this stepping segment was not served from a prepared slot
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
#if USE_SHADOW_SEGMENTS
		++shadowGen;											// tell PrepareShadowChunk that a segment has been released
		if (shadowSlots[shadowHead].seg != nullptr && (seg == shadowSlots[shadowHead].chainHead || seg == shadowSlots[shadowHead].seg))
		{
			FlushShadows();										// we are releasing a segment the preparation covers without consuming it, so the slots are stale
		}
#if SHADOW_CACHE_DIAGNOSTICS
		++shadowCacheMisses;									// this zero-step segment was skipped without a prepared slot
		++skipRun;
		if (skipRun > maxIsrSkip) { maxIsrSkip = skipRun; }
#endif
#endif
		MoveSegment *oldSeg = seg;
		segments = seg = seg->GetNext();						// skip this segment
		if (seg == nullptr) { segmentsTail = nullptr; }			// keep the tail cache consistent when the list empties
		if (segHint == oldSeg) { segHint = nullptr; }			// invalidate the insertion hint if we are releasing the segment it points to
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

#if USE_SHADOW_SEGMENTS
			++shadowGen;										// tell PrepareShadowChunk that a segment has been released
			if (shadowSlots[shadowHead].seg != nullptr && (currentSegment == shadowSlots[shadowHead].chainHead || currentSegment == shadowSlots[shadowHead].seg))
			{
				FlushShadows();									// can't happen (a prepared segment is consumed when it starts executing), but don't risk stale slots
			}
#endif
			movementAccumulator += netStepsThisSegment;				// update the amount of extrusion for filament monitors
			const uint32_t prevEndTime = currentSegment->GetStartTime() + currentSegment->GetDuration();
			MoveSegment *const nextSeg = currentSegment->GetNext();
			segments = nextSeg;
			if (nextSeg == nullptr) { segmentsTail = nullptr; }		// keep the tail cache consistent when the list empties
			if (segHint == currentSegment) { segHint = nullptr; }	// invalidate the insertion hint if we are releasing the segment it points to
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

	motioncalc_t nextCalcStepTime;

	// Work out the time of the step
	switch (state)
	{
	case DMState::cartLinear:									// linear steady speed
		nextCalcStepTime = (motioncalc_t)(nextStep + (int32_t)stepsTillRecalc) * p;
		break;

	case DMState::cartAccel:									// Cartesian accelerating
		nextCalcStepTime = fastLimSqrtm(q + p * (motioncalc_t)(nextStep + (int32_t)stepsTillRecalc));
		break;

	case DMState::cartDecelForwardsReversing:
		if (nextStep + (int32_t)stepsTillRecalc < reverseStartStep)
		{
			nextCalcStepTime = -fastLimSqrtm(q + p * (motioncalc_t)(nextStep + (int32_t)stepsTillRecalc));
			break;
		}

		direction = !direction;
		directionChanged = true;
		state = DMState::cartDecelReverse;
		// no break
	case DMState::cartDecelReverse:								// Cartesian decelerating, reverse motion. Convert the steps to int32_t because the net steps may be negative.
		{
			const int32_t netSteps = 2 * reverseStartStep - nextStep - 1;
			nextCalcStepTime = fastLimSqrtm(q + p * (motioncalc_t)(netSteps - (int32_t)stepsTillRecalc));
		}
		break;

	case DMState::cartDecelNoReverse:							// Cartesian decelerating with no reversal
		nextCalcStepTime = -fastLimSqrtm(q + p * (motioncalc_t)(nextStep + (int32_t)stepsTillRecalc));
		break;

	default:
#if SEGMENT_DEBUG
		debugPrintf("DMstate %u, quitting\n", (unsigned int)state);
#endif
		return LogStepError(4, (float)state, currentSegment);
	}

	nextCalcStepTime += t0;
	uint32_t iNextCalcStepTime;

	// Check that the next step time is reasonable
#if (SAMC21 || RP2040) && !USE_DOUBLE_MOTIONCALC
	// The FP library we use on Cortext-M0+ MCUs doesn't support NaNs so there is no point in testing for them
	if (unlikely(std::signbit(nextCalcStepTime)))
#else
	if (unlikely(std::isnan(nextCalcStepTime)))
	{
		return LogStepError(2, (float)nextCalcStepTime, currentSegment);
	}

	if (unlikely(nextCalcStepTime < (motioncalc_t)0.0))
#endif
	{
		// If we are carrying almost a whole step forward to this segment so that the first step is due almost immediately,
		// then due to floating point rounding error we can get a slightly negative value here for nextCalcStepTime.
		// The only value we have had reported so far is -0.00195 but we now allow up to two step clocks of error.
		if (nextCalcStepTime < -(motioncalc_t)2.0)
		{
			return LogStepError(2, (float)nextCalcStepTime, currentSegment);
		}
		iNextCalcStepTime = 0;
	}
	else
	{
		iNextCalcStepTime = (uint32_t)nextCalcStepTime;
	}

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
#if USE_SHADOW_SEGMENTS
		FlushShadows();										// the prepared slots refer to segments we are about to release
#endif
		MoveSegment *seg = nullptr;
		std::swap(seg, const_cast<MoveSegment*&>(segments));
		segmentsTail = nullptr;
		segHint = nullptr;
		MoveSegment::ReleaseAll(seg);
	}
}

#endif	// SUPPORT_DRIVERS

// End
