/*
 * MoveSegment.h
 *
 *  Created on: 26 Feb 2021
 *      Author: David
 *
 * This class holds the parameters of a segment of a move with constant acceleration.
 * In order to handle input shaping we need to superimpose segments. This means we have to store the basic parameters.
 * The distance travelled when acceleration is a and initial speed is u is:
 *
 *		s = u*t + 0.5*a*t^2
 *
 * After n steps we want to achieve this distance plus any outstanding movement when the move started. So if q is the mm per step then:
 *
 * 		n*q = s0 + u*t + 0.5*a*t^2
 *
 * The segment parameters are therefore s0, u and a. We also store the start time t0 and the segment duration td.
 * We can superimpose two segments that start at the same times t0 by adding the s0, u and a parameters.
 * If the segments start and/or end at different times then we must split one or both into two or three segments so that we can superimpose segments with the same times.
 */

#ifndef SRC_MOVEMENT_MOVESEGMENT_H_
#define SRC_MOVEMENT_MOVESEGMENT_H_

#include <RepRapFirmware.h>
#include <Platform/Tasks.h>
#include <new>		// for align_val_t

#define SEGMENT_DEBUG	(0)
#define CHECK_SEGMENTS	(0)

// This bit field is used in multiple contexts so that we can copy them efficiently from one context to another. Not all flags are used in all contexts.
union MovementFlags
{
	// To conserve memory space on boards with less RAM such as TOOL1LC, we now combine the flags field in the MoveSegment with the next' field.
	// This means that we need to choose flag bits which are not valid bits in the address of a MoveSegment.
	// We can use the lowest 2 bits because a MoveSegment is always 4-byte aligned.
	// On the processors we use, addresses above 0x40000000 are used for peripherals and core registers, so we can use he top 2 bits as well.
	uint32_t all;												// this is to provide a means to clear all the flags in one go
	struct
	{
		uint32_t isExtruder			: 1,						// true if this segment is for an extruder
				 nonPrintingMove	: 1,						// true if the move that generated this segment does not have both forwards extrusion and associated axis movement; used for filament monitoring
				 	 	 	 	 	: 28,						// these bits might be valid in a RAM address
				 noShaping			: 1,						// true if input shaping should be disabled for this move
				 executing			: 1;						// normally clear, set in a MoveSegment when the move starts to be executed
	};

	static constexpr uint32_t FlagsMask = 0xC0000003;			// mask of all the flag bits. *** WARNING! this must be kept in step with flag bits allocation, see above. ***
	static constexpr uint32_t ExecutingBit = 0x80000000;		// just the 'executing' flag bit. *** WARNING! this must be kept in step with flag bits allocation, see above. ***

	constexpr MovementFlags() noexcept : all(0) { }
	constexpr MovementFlags(uint32_t f) noexcept : all(f) { }

	constexpr void InitNonPrinting() noexcept { all = 0; nonPrintingMove = true; }

	// This operator sets checkingEndstops if either of the segments to be combined checks endstops, and sets nonPrintingMove if either of them is a non printing move
	MovementFlags operator|(const MovementFlags other) const noexcept
	{
		return MovementFlags(all | other.all);
	}

	MovementFlags& operator|=(const MovementFlags other) noexcept
	{
		all |= other.all;
		return *this;
	}

	MovementFlags AddIsExtruder() const noexcept
	{
		MovementFlags ret(all);
		ret.isExtruder = true;
		return ret;
	}
};

// This class stores the characteristics of a segment of a move with constant acceleration.
// The characteristics stored are the start time in step clocks, the duration in step clocks, the distance moved in steps, the acceleration, and some flags.
// We no longer store the initial speed because it can be calculated from the duration, distance and acceleration.
class MoveSegment
{
public:
	void* operator new(size_t count) noexcept { return Tasks::AllocPermanent(count); }
	void* operator new(size_t count, std::align_val_t align) noexcept { return Tasks::AllocPermanent(count, align); }
	void operator delete(void* ptr) noexcept {}
	void operator delete(void* ptr, std::align_val_t align) noexcept {}

	// Read the values of the flag bits
	bool IsLinear() const noexcept { return a == 0; }		//TODO: should we ignore very small accelerations, to avoid rounding error in the calculation?
	MovementFlags GetFlags() const noexcept { return MovementFlags(nextAndFlags & MovementFlags::FlagsMask); }

#if 0 //SUPPORT_REMOTE_COMMANDS
	bool IsRemote() const noexcept { return isRemote; }
#endif

	// Given that this is not a constant-speed segment, test whether it is accelerating or decelerating
	bool IsAccelerating() const noexcept { return a > (motioncalc_t)0.0; }

	// Get the segment start time in step clocks
	uint32_t GetStartTime() const noexcept { return startTime; }

	// Get the segment duration in step clocks
	uint32_t GetDuration() const noexcept { return duration; }

	// Get the initial speed
	motioncalc_t CalcU() const noexcept { return distance/(motioncalc_t)duration - 0.5 * a * (motioncalc_t)duration; }

	// Get the initial speed assuming this move has no acceleration
	motioncalc_t CalcLinearU() const noexcept { return distance/(motioncalc_t)duration; }

	// Get the reciprocal of the initial speed assuming this move has no acceleration
	motioncalc_t CalcLinearRecipU() const noexcept pre(a == 0.0) { return (motioncalc_t)duration/distance; }

	// Get the acceleration
	motioncalc_t GetA() const noexcept { return a; }

	// Get the length
	motioncalc_t GetLength() const noexcept { return distance; }

	// Make a small correction to the length. Only ever called on the last segment in a list.
	void AdjustLength(motioncalc_t adjustment) noexcept { distance += adjustment; }

	// Set the parameters of this segment
	void SetParameters(uint32_t p_startTime, uint32_t p_duration, motioncalc_t p_distance, motioncalc_t p_a, MovementFlags p_flags) noexcept;

	// Split this segment in two, returning a pointer to the second part
	MoveSegment *Split(uint32_t firstDuration) noexcept pre(firstDuration < duration);

	// Merge the parameters for another segment with the same start time and duration into this one
	void Merge(motioncalc_t p_distance, motioncalc_t p_a, MovementFlags p_flags) noexcept;

	// Normalise this segment by removing very small accelerations that cause problems, update t0, return true if it is linear
	bool NormaliseAndCheckLinear(motioncalc_t distanceCarriedForwards, motioncalc_t& t0) noexcept;

	// Set the 'executing' bit in the flags
	void SetExecuting() noexcept { nextAndFlags |= MovementFlags::ExecutingBit; }

	// Get the next segment in this list
	MoveSegment *GetNext() const noexcept;

	// Set the next segment in this list
	void SetNext(MoveSegment *p_next) noexcept;

	// Print this segment to the debug channel
	void DebugPrint() const noexcept;

	// Print list of segments
	static void DebugPrintList(const MoveSegment *segs) noexcept;

	// Allocate a MoveSegment, clearing the flags
	static MoveSegment *Allocate(MoveSegment *p_next) noexcept;

	// Release a MoveSegment
	static void Release(MoveSegment *item) noexcept;

	// Release all MoveSegments in a chain
	static void ReleaseAll(MoveSegment *item) noexcept;

	// Return the number of MoveSegment objects that have been created
	static unsigned int NumCreated() noexcept { return numCreated; }

	static constexpr int32_t MinDuration = 10;				// the minimum duration in movement clock ticks that we consider sensible

protected:
	static MoveSegment *freeList;							// list of recycled segment objects
	static unsigned int numCreated;							// total number of segment objects created

	uint32_t nextAndFlags;									// pointer to the next segment, also holds the flag bits
	uint32_t startTime;										// when this segment should start, in movement clock ticks
	uint32_t duration;										// the duration of this segment in movement ticks
	motioncalc_t distance;									// the number of steps moved
	motioncalc_t a;											// the acceleration during this segment in steps per movement tick squared

private:
	MoveSegment(MoveSegment *p_next) noexcept;
};

// Create a new one, leaving the flags clear
inline MoveSegment::MoveSegment(MoveSegment *p_next) noexcept
	: nextAndFlags(reinterpret_cast<uint32_t>(p_next))
{
	// remaining fields are not initialised
}

// Test whether a floating point number is equal to positive or negative zero
static bool IsNonZero(motioncalc_t f) noexcept
{
#if (SAMC21 || RP2040) && !USE_DOUBLE_MOTIONCALC && !defined(__ECV__)
	// Nasty hack to avoid calling floating point subroutines on MCUs without a hardware FPU
	// ARM uses IEEE format, which uses the MSB as a sign bit and encodes zero as all zeros
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wstrict-aliasing"
	return (*reinterpret_cast<const uint32_t*>(&f) << 1) != 0;
# pragma GCC diagnostic pop
#else
	return f != (motioncalc_t)0.0;
#endif
}

// Test whether a floating point number is strictly greater than zero. The result is undefined if the operand is a NaN.
inline bool IsPositive(motioncalc_t f) noexcept
{
#if (SAMC21 || RP2040) && !USE_DOUBLE_MOTIONCALC && !defined(__ECV__)
	// Nasty hack to avoid calling floating point subroutines on MCUs without a hardware FPU
	// ARM uses IEEE format, which uses the MSB as a sign bit and encodes positive zero as all zeros. Therefore we can just test for > 0 when interpreted as an integer.
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wstrict-aliasing"
	return *reinterpret_cast<const int32_t*>(&f) > 0;
# pragma GCC diagnostic pop
#else
	return f > (motioncalc_t)0.0;
#endif
}

// Core of NormaliseAndCheckLinear below: decide whether a segment is to be treated as constant speed, and compute
// the corresponding t0, without modifying anything. Written once, so that NormaliseAndCheckLinear and
// DriveMovement::PrepareShadowChunk (which works on a copy of the segment fields, because the segment may still be
// modified before it executes) compute bit-identical results from a single copy of this code.
// Returns 0 if the segment has usable acceleration or deceleration, with t0 = time from start of segment at which the speed would have been/will be/would be zero;
//         1 if it is constant speed, with t0 = time from start of segment at which the distance would be/will be/would have been zero;
//         2 if it is to be treated as constant speed (t0 as for 1) because its tiny acceleration would cause calculation problems.
static inline unsigned int CheckLinearCore(motioncalc_t a, uint32_t duration, motioncalc_t distance, motioncalc_t distanceCarriedForwards, motioncalc_t& t0) noexcept
{
	unsigned int ret = 1;
	if (IsNonZero(a))
	{
		// The move has acceleration or deceleration, but it may be small enough to cause problems with the calculations.
		// The reason is that the step time is calculated as:
		//   time_from_segment_start = t0 +/- sqrt(q - p*n)
		// where q equals t0^2 or something very close to it. This gives rise to two issues:
		// 1. The maximum value that can be represented by a float is a little more than 3.4e38, so t0 values greater than about 1e19 cause trouble when we square them to calculate q.
		// 2. Rounding error may cause large errors in the step time, when t0 can't represented to within a small number of step clocks
		// Issue #2 causes problems when abs(t0) exceeds about 2^24 because then the number of step clocks can't be represented exactly.
		// Here are two possible ways round this:
		// 1. When t0 gets large we could use the Maclaurin expansion of sqrt(q - p*n) to give:
		//    time_from_segment_start ~= p*n/(2 * sqrt(q + p*n))
		// This is accurate to within about 1 clock on the last step N when (p*N)^4 < 8*(q + p*N)^3
		// so approximately when (p*N)^4 < 8*q^3, or very roughly when p*N << q
		// However, using the Maclaurin expansion requires an extra division in each step calculation, which we would prefer to avoid.
		// 2. We can convert the segment to a constant-speed segment, on the assumption that the speed won't change much during it. This is what we currently do.
		const motioncalc_t provisionalT0 = (motioncalc_t)0.5 * (motioncalc_t)duration - distance/(a * (motioncalc_t)duration);
		if (likely(fabsm(provisionalT0) <= 4 * (motioncalc_t)16777216.0))
		{
			t0 = provisionalT0;
			return 0;
		}
		ret = 2;												// the acceleration is small enough to cause calculation problems, so treat this as a linear move
	}

	// The move is constant speed
	t0 = -distanceCarriedForwards * (motioncalc_t)duration/distance;
	return ret;
}

// Normalise this segment by removing very small accelerations that cause problems, update t0, return true if it is linear
// Called only from DriveMovement::NewSegment. Speed critical, hence inline and the rather unusual behaviour.
// Returns:
//  true if the segment is constant speed, with t0 = time from start of segment at which the distance would be/will be/would have been zero
//  false if the segment has acceleration or deceleration, with t0 = time from start of segment at which the speed would have been/will be/would be zero
inline bool MoveSegment::NormaliseAndCheckLinear(motioncalc_t distanceCarriedForwards, motioncalc_t& t0) noexcept
{
	const unsigned int ret = CheckLinearCore(a, duration, distance, distanceCarriedForwards, t0);
	if (ret == 2)
	{
		a = (motioncalc_t)0.0;									// remove the tiny acceleration, so that the rest of the segment processing treats it as linear
	}
	return ret != 0;
}

// Release a MoveSegment
inline void MoveSegment::Release(MoveSegment *item) noexcept
{
	const auto iflags = IrqSave();
	item->nextAndFlags = reinterpret_cast<uint32_t>(freeList);
	freeList = item;
	IrqRestore(iflags);
}

inline MoveSegment *MoveSegment::GetNext() const noexcept
{
	return reinterpret_cast<MoveSegment*>(nextAndFlags & ~MovementFlags::FlagsMask);
}

inline void MoveSegment::SetNext(MoveSegment *p_next) noexcept
{
	nextAndFlags = (nextAndFlags & MovementFlags::FlagsMask) | reinterpret_cast<uint32_t>(p_next);
}

// Set the parameters of this segment. We assume the flags are clear initially.
inline void MoveSegment::SetParameters(uint32_t p_startTime, uint32_t p_duration, motioncalc_t p_distance, motioncalc_t p_a, MovementFlags p_flags) noexcept
{
	startTime = p_startTime;
	duration = p_duration;
	distance = p_distance;
	a = p_a;
	nextAndFlags |= p_flags.all & MovementFlags::FlagsMask;	
}

// Split this segment in two, returning a pointer to the new second part
inline MoveSegment *MoveSegment::Split(uint32_t firstDuration) noexcept
{
	MoveSegment *const secondSeg = Allocate(GetNext());
	const motioncalc_t firstDistance = (CalcU() + (motioncalc_t)0.5 * a * (motioncalc_t)firstDuration) * (motioncalc_t)firstDuration;
	secondSeg->SetParameters(startTime + firstDuration, duration - firstDuration, distance - firstDistance, a, GetFlags());
#if SEGMENT_DEBUG
	debugPrintf("split at %" PRIu32 ", fd=%.2f, sd=%.2f\n", firstDuration, (double)firstDistance, (double)(distance - firstDistance));
#endif
	duration = firstDuration;
	distance = firstDistance;
	SetNext(secondSeg);
	return secondSeg;
}

// Merge the parameters for another segment with the same start time and duration into this one
// s = u*t * 0.5*a*t^2 therefore s1+s2 = (u1+u2)*t + 0.5*(a1+a2)*t^2
inline void MoveSegment::Merge(motioncalc_t p_distance, motioncalc_t p_a, MovementFlags p_flags) noexcept
{
#if SEGMENT_DEBUG
	debugPrintf("merge d=%.2f a=%.4e into ", (double)p_distance, (double)p_a);
	DebugPrint();
#endif
	distance += p_distance;
	a += p_a;
	nextAndFlags |= (p_flags.all & MovementFlags::FlagsMask);
}

// Fixed-point step time calculation support.
// On the SAMC21 the remaining per-step cost in CalcNextStepTimeFull is the soft-float arithmetic itself:
// computing n*p + t0 (linear) or t0 +/- sqrt(q + p*n) (accelerating) costs one Qfplib multiply (62 clocks),
// one or two adds (76 each) and for the accelerating cases a square root (67), plus the conversions.
// The helpers below allow those expressions to be evaluated in 64-bit fixed point instead:
// - time-like quantities (t0, n*p for linear segments, and the square root result) are held as value * 2^24
//   step clocks in an int64_t (Q40.24), which covers the full uint32_t range of segment durations with more
//   fractional precision than a float mantissa provides;
// - the square root operand q + p*n is held as value * 2^sqrtScale where sqrtScale is chosen per segment from
//   the float exponents of q and p such that all intermediate values fit comfortably in an int64_t.
// The square root itself stays in floating point: Qfplib's qfp_fsqrt takes ~67 clocks, far less than a 64-bit
// integer square root needs on these cores (~300 clocks for the shift-subtract loop in Isqrt.cpp), so the
// operand is converted to float just for that one call.
// These helpers are used only on boards without a hardware FPU and with single-precision motioncalc_t.
// The RP2040 also has a Cortex-M0+ core, but the performance trade-offs have not been verified there
// (in particular its flash is QSPI XIP with a cache, so all the flash/RAM placement reasoning differs),
// so this is deliberately restricted to the SAMC21.
#if SAMC21 && !USE_DOUBLE_MOTIONCALC && !defined(__ECV__)
# define USE_FIXED_STEP_TIMING	(1)
#else
# define USE_FIXED_STEP_TIMING	(0)
#endif

#if USE_FIXED_STEP_TIMING

constexpr int32_t StepTimeFracBits = 24;						// time-like fixed point quantities are value * 2^24 step clocks in an int64_t

// Return floor(log2(v)) for v != 0 by loop-free binary search; these cores have no CLZ instruction and
// __builtin_clz would call __clzsi2 in flash.
static inline uint32_t FloorLog2(uint32_t v) noexcept
{
	uint32_t e = 0;
	if (v >= (1u << 16)) { v >>= 16; e = 16; }
	if (v >= (1u << 8))  { v >>= 8;  e += 8; }
	if (v >= (1u << 4))  { v >>= 4;  e += 4; }
	if (v >= (1u << 2))  { v >>= 2;  e += 2; }
	if (v >= (1u << 1))  {           e += 1; }
	return e;
}

// 32x32 -> 64 bit unsigned multiply from 16-bit partial products. The muls instruction is single-cycle on the
// SAMC21, so this is ~16 clocks inline; the plain C expression (uint64_t)a * b would be a call to
// __aeabi_lmul, which lives in flash and is reached from the RAM-resident step ISR through a veneer.
static inline uint64_t Mul32x32To64(uint32_t a, uint32_t b) noexcept
{
	const uint32_t a0 = a & 0xFFFFu, a1 = a >> 16;
	const uint32_t b0 = b & 0xFFFFu, b1 = b >> 16;
	const uint32_t p00 = a0 * b0;
	const uint32_t p01 = a0 * b1;
	const uint32_t mid = p01 + a1 * b0;							// (p01 + p10) mod 2^32; if this wraps, the lost bit is worth 2^16 in the high word
	const uint32_t hi = a1 * b1 + ((mid < p01) ? (1u << 16) : 0u);
	return (((uint64_t)hi << 32) | p00) + ((uint64_t)mid << 16);
}

// Multiply a step number by a 64-bit fixed point coefficient, exactly. The caller must guarantee
// |n * coeff| < 2^63; the conversions below saturate the coefficients so that this holds at all call sites.
// Out-of-line in RAM (MoveSegment.cpp): one shared copy instead of five inline copies in the per-step switch in
// CalcNextStepTimeFull. It stays in RAM because it is on the per-step hot path; the call overhead is a few clocks.
int64_t MulStepByCoeff(int32_t n, int64_t coeff) noexcept;

// Arithmetic right shift of an int64_t by a variable amount, pre(1 <= n <= 63). Composed from 32-bit shifts
// because the plain C expression would be a call to __aeabi_lasr in flash.
static inline int64_t ShiftRight64(int64_t v, uint32_t n) noexcept
{
	const int32_t hi = (int32_t)(uint32_t)((uint64_t)v >> 32);
	if (n >= 32)
	{
		return (int64_t)(hi >> (n - 32u));						// sign-extends; n - 32 is at most 31
	}
	const uint32_t lo = (uint32_t)v;
	return (int64_t)(((uint64_t)(uint32_t)(hi >> n) << 32) | ((lo >> n) | ((uint32_t)hi << (32u - n))));
}

// Return the biased exponent field of a motioncalc_t, i.e. 2^(exp - 127) <= |f| < 2^(exp - 126) for normal values
static inline uint32_t MotionCalcBiasedExponent(motioncalc_t f) noexcept
{
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wstrict-aliasing"
	return (*reinterpret_cast<const uint32_t*>(&f) >> 23) & 0xFFu;
# pragma GCC diagnostic pop
}

// Convert a motioncalc_t to fixed point value * 2^fracBits, truncating towards zero, saturating at +/-2^satBits
// (pre: 32 <= satBits <= 62). fracBits may be negative. Infinities and NaNs saturate too, so garbage coefficients
// produce bounded step times instead of undefined behaviour. The 64-bit shifts are composed from 32-bit shifts
// because a 64-bit shift by a variable amount would be a call to __aeabi_llsl/__aeabi_llsr in flash.
static inline int64_t FastMotionCalcToFix(motioncalc_t f, int32_t fracBits, uint32_t satBits) noexcept
{
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wstrict-aliasing"
	const uint32_t b = *reinterpret_cast<const uint32_t*>(&f);
# pragma GCC diagnostic pop
	const uint32_t biasedExp = (b >> 23) & 0xFFu;
	if (biasedExp == 0)
	{
		return 0;												// zero (denormals cannot occur in the motion calculations)
	}
	const int32_t sh = (int32_t)biasedExp - (127 + 23) + fracBits;	// f * 2^fracBits = +/- mant * 2^sh
	const uint32_t mant = (b & 0x7FFFFFu) | 0x800000u;
	uint64_t mag;
	if (sh <= -24)
	{
		mag = 0;
	}
	else if (sh < 0)
	{
		mag = mant >> (uint32_t)-sh;
	}
	else if ((uint32_t)sh >= satBits - 23)						// the top bit of mant would land at bit 23 + sh
	{
		mag = (uint64_t)(1u << (satBits - 32)) << 32;			// saturate to exactly 2^satBits
	}
	else if (sh < 9)
	{
		mag = mant << (uint32_t)sh;								// fits in 32 bits (23 + 8 = 31)
	}
	else if (sh < 32)
	{
		mag = ((uint64_t)(mant >> (32u - (uint32_t)sh)) << 32) | (uint32_t)(mant << (uint32_t)sh);
	}
	else
	{
		mag = (uint64_t)(mant << ((uint32_t)sh - 32u)) << 32;	// sh <= satBits - 24 <= 38, so mant shifts left by at most 6 bits here
	}
	return ((int32_t)b < 0) ? -(int64_t)mag : (int64_t)mag;
}

// Convert a uint64_t to motioncalc_t, truncating towards zero. Used only to feed the square root operand to
// qfp_fsqrt, so the truncation error is at most one ulp of the result.
static inline motioncalc_t FastUint64ToMotionCalc(uint64_t v) noexcept
{
	const uint32_t hi = (uint32_t)(v >> 32);
	uint32_t e, top;
	if (hi == 0)
	{
		const uint32_t lo = (uint32_t)v;
		if (lo == 0)
		{
			return (motioncalc_t)0.0;
		}
		e = FloorLog2(lo);
		top = (e <= 23) ? lo << (23u - e) : lo >> (e - 23u);
	}
	else
	{
		e = FloorLog2(hi) + 32u;
		const uint32_t sh = e - 23u;							// 9..40 because hi != 0
		top = (sh < 32u) ? ((uint32_t)v >> sh) | (hi << (32u - sh)) : hi >> (sh - 32u);
	}
	const uint32_t bits = ((e + 127u) << 23) + (top & 0x7FFFFFu);
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wstrict-aliasing"
	return *reinterpret_cast<const float*>(&bits);
# pragma GCC diagnostic pop
}

#endif	// USE_FIXED_STEP_TIMING

#endif /* SRC_MOVEMENT_MOVESEGMENT_H_ */
