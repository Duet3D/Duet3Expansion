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
