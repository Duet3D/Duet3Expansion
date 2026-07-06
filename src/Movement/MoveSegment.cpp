/*
 * MoveSegment.cpp
 *
 *  Created on: 26 Feb 2021
 *      Author: David
 */

#include "MoveSegment.h"

// Static members

MoveSegment *MoveSegment::freeList = nullptr;
unsigned int MoveSegment::numCreated = 0;

// Allocate a MoveSegment, from the freelist if possible, else create a new one
MoveSegment *MoveSegment::Allocate(MoveSegment *p_next) noexcept
{
	const auto iflags = IrqSave();
	MoveSegment * ms = freeList;
	if (ms != nullptr)
	{
		freeList = ms->GetNext();
		IrqRestore(iflags);
		ms->nextAndFlags = reinterpret_cast<uint32_t>(p_next);
	}
	else
	{
		++numCreated;
		IrqRestore(iflags);
		ms = new MoveSegment(p_next);
	}
	return ms;
}

// Release a MoveSegment
void MoveSegment::ReleaseAll(MoveSegment *item) noexcept
{
	while (item != nullptr)
	{
		MoveSegment *itemToRelease = item;
		item = item->GetNext();
		Release(itemToRelease);
	}
}

void MoveSegment::DebugPrint() const noexcept
{
	debugPrintf("s=%" PRIu32 " t=%" PRIu32 " d=%.2f u=%.4e a=%.4e f=%02" PRIx32 "\n", startTime, duration, (double)distance, (double)CalcU(), (double)a, GetFlags().all);
}

/*static*/ void MoveSegment::DebugPrintList(const MoveSegment *segs) noexcept
{
	if (segs == nullptr)
	{
		debugPrintf("null seg\n");
	}
	else
	{
		while (segs != nullptr)
		{
			segs->DebugPrint();
			segs = segs->GetNext();
		}
	}
}

#if SAMC21 && !USE_DOUBLE_MOTIONCALC && !defined(__ECV__)

// Out-of-line copies of some of the fast conversion helpers declared in MoveSegment.h, so that their bodies are
// not duplicated at every inlined call site in the RAM-resident step ISR code. They are used on segment
// preparation and fallback paths only, so they live in flash.

motioncalc_t FastUintToMotionCalc(uint32_t v) noexcept
{
	return FastUintToMotionCalcI(v);
}

// See the comment on the declaration in MoveSegment.h for what this computes
int32_t FastMotionCalcToInt(motioncalc_t f) noexcept
{
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-aliasing"
	const uint32_t b = *reinterpret_cast<const uint32_t*>(&f);
#pragma GCC diagnostic pop
	const uint32_t e = (b >> 23) & 0xFFu;							// biased exponent without the sign bit
	if (e < 127)
	{
		return 0;													// |f| < 1.0, including +/-0.0 and subnormals
	}
	const int32_t sh = (int32_t)e - (127 + 23);
	if (sh >= 8)
	{
		return ((int32_t)b < 0) ? INT32_MIN : INT32_MAX;			// |f| >= 2^31: saturate (unreachable at our call sites)
	}
	const uint32_t mant = (b & 0x7FFFFFu) | 0x800000u;
	const uint32_t mag = (sh >= 0) ? mant << sh : mant >> (uint32_t)-sh;
	return ((int32_t)b < 0) ? -(int32_t)mag : (int32_t)mag;
}

#endif

#if USE_FIXED_STEP_TIMING

// One shared copy of the step-number by coefficient multiply used by the per-step calculations in
// CalcNextStepTimeFull; deliberately out of line so that its body is not duplicated at the five call sites in the
// RAM-resident step ISR code, and in RAM because it is on the per-step hot path.
// See the comment on the declaration in MoveSegment.h for the precondition.
__attribute__((noinline, section(".time_critical")))
int64_t MulStepByCoeff(int32_t n, int64_t coeff) noexcept
{
	uint32_t un;
	bool neg;
	if (n < 0) { un = (uint32_t)-n; neg = true; } else { un = (uint32_t)n; neg = false; }
	uint64_t uc;
	if (coeff < 0) { uc = (uint64_t)-coeff; neg = !neg; } else { uc = (uint64_t)coeff; }
	const uint64_t prod = Mul32x32To64(un, (uint32_t)uc) + ((uint64_t)(un * (uint32_t)(uc >> 32)) << 32);
	return (neg) ? -(int64_t)prod : (int64_t)prod;
}

#endif	// USE_FIXED_STEP_TIMING

// End
