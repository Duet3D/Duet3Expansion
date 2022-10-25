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
		uint32_t currentAngle = pos % (int32_t)countsPerRev;
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
	}
}

// Process the tuning data
uint8_t RelativeEncoder::ProcessTuningData() noexcept
{
	uint8_t result;

#ifdef DEBUG
	debugPrintf("forward slope %.4f reverse %.4f\n", (double)forwardSlope, (double)reverseSlope);
#endif

	// Check that the forward and reverse slopes are similar and a good match to the configured counts per step
	const float averageSlope = (forwardSlope + reverseSlope) * 0.5;

	// We sometimes read different forwards and reverse counts, so instead of taking an average of the origin, average the origin w.r.t. the mid points of the tuning moves
	forwardOrigin += (forwardSlope - averageSlope) * forwardXmean;
	reverseOrigin += (reverseSlope - averageSlope) * reverseXmean;

	measuredCountsPerStep = fabsf(averageSlope) * 1024;
	measuredHysteresis = fabsf(forwardOrigin - reverseOrigin)/measuredCountsPerStep;

	if (fabsf(averageSlope) < MinimumSlope || fabsf(forwardSlope - reverseSlope) > MaxSlopeMismatch * 2 * fabsf(averageSlope))
	{
		result = TuningError::InconsistentMotion;
	}
	else if (measuredCountsPerStep > GetCountsPerStep() * 1.05)
	{
		result = TuningError::TooMuchMotion;
	}
	else if (measuredCountsPerStep < GetCountsPerStep() * 0.95)
	{
		result = TuningError::TooLittleMotion;
	}
	else
	{
		result = 0;
	}

	return result;
}

#endif

// End
