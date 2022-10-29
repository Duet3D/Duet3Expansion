/*
 * RelativeEncoder.cpp
 *
 *  Created on: 20 Oct 2022
 *      Author: David
 */

#include "RelativeEncoder.h"

#if SUPPORT_CLOSED_LOOP

constexpr float MaxSlopeMismatch = 0.1;				// we want the forward and reverse measured CPS each to be within 10% of the average
constexpr float MinimumSlope = 7.5/1024;			// we require at least 8 transitions (8 pulses) per step, but we may measure slightly lower than the true value

// Take a reading and store currentCount, currentPhasePosition. Return true if success.
bool RelativeEncoder::TakeReading() noexcept
{
	bool err;
	const int32_t pos = GetRelativePosition(err);
	if (!err)
	{
		currentCount = pos * reversePolarityMultiplier;
		int32_t currentAngle = pos % (int32_t)countsPerRev;
		if (currentAngle < 0) { currentAngle += countsPerRev; }
		currentPhasePosition = (uint32_t)lrintf((currentAngle * stepsPerCount * 1024u) + zeroCountPhasePosition) % 4095u;
	}
	return err;
}

// Tell the encoder what the step phase is at a particular count, so that if currentCount was 'count' then currentPhasePosition would be calculated as 'phase'
void RelativeEncoder::SetKnownPhaseAtCount(uint32_t phase, int32_t count) noexcept
{
	count %= (int32_t)countsPerRev;
	if (count < 0) { count += countsPerRev; }
	const uint32_t relativePhasePosition = (uint32_t)lrintf(count * stepsPerCount * 1024u);
	zeroCountPhasePosition = (phase - relativePhasePosition) & 4095u;
}

// Encoder polarity. Changing this will change the encoder reading.
void RelativeEncoder::SetBackwards(bool backwards) noexcept
{
	const int32_t newMultiplier = (backwards) ? -1 : 1;
	if (newMultiplier != reversePolarityMultiplier)
	{
		reversePolarityMultiplier = newMultiplier;
		currentCount = -currentCount;
	}
}

// Process the tuning data
TuningErrors RelativeEncoder::ProcessTuningData() noexcept
{
	TuningErrors result;

#ifdef DEBUG
	debugPrintf("slope %.5f %.5f, mean phase %.1f %.1f, mean reading %.3f %.3f, \n",
					(double)forwardSlope, (double)reverseSlope, (double)forwardXmean, (double)reverseXmean, (double)forwardYmean, (double)reverseYmean);
#endif

	// Check that the forward and reverse slopes are similar and a good match to the configured counts per step
	const float averageSlope = (forwardSlope + reverseSlope) * 0.5;
	measuredCountsPerStep = fabsf(averageSlope) * 1024;

	if (fabsf(averageSlope) < MinimumSlope || fabsf(forwardSlope - reverseSlope) > MaxSlopeMismatch * 2 * fabsf(averageSlope))
	{
		result = TuningError::InconsistentMotion;
	}
	else if (measuredCountsPerStep > countsPerStep * 1.05)
	{
		result = TuningError::TooMuchMotion;
	}
	else if (measuredCountsPerStep < countsPerStep * 0.95)
	{
		result = TuningError::TooLittleMotion;
	}
	else
	{
		const float slopeMultiplier = (averageSlope < 0.0) ? -1.0 : 1.0;
		const float forwardOrigin = forwardYmean - forwardXmean * countsPerStep * slopeMultiplier/1024.0;
		const float reverseOrigin = reverseYmean - reverseXmean * countsPerStep * slopeMultiplier/1024.0;
		measuredHysteresis = fabsf(forwardOrigin - reverseOrigin)/countsPerStep;
		const float forwardZeroPhase = forwardYmean * -slopeMultiplier * 1024.0/countsPerStep + forwardXmean;
		const float reverseZeroPhase = reverseYmean * -slopeMultiplier * 1024.0/countsPerStep + reverseXmean;
#ifdef DEBUG
		debugPrintf("count %" PRIi32 "\n", currentCount);
#endif
		SetBackwards(averageSlope < 0.0);
		const int32_t zcp = lrintf((forwardZeroPhase + reverseZeroPhase) * 0.5) % 4095;
		zeroCountPhasePosition = (uint32_t)((zcp < 0) ? zcp + 4096 : zcp);
#ifdef DEBUG
		debugPrintf("origins %.3f %.3f zph %.1f %.1f diff %.1f zcp %" PRIi32 " zcpp %" PRIu32 "\n",
					(double)forwardOrigin, (double) reverseOrigin, (double)forwardZeroPhase, (double)reverseZeroPhase, (double)fabsf(forwardZeroPhase - reverseZeroPhase),
					zcp, zeroCountPhasePosition);
		debugPrintf("count %" PRIi32 "\n", currentCount);
#endif
		result = 0;
	}

	return result;
}

#endif

// End
