/*
 * RelativeEncoder.cpp
 *
 *  Created on: 20 Oct 2022
 *      Author: David
 */

#include "RelativeEncoder.h"

#if SUPPORT_CLOSED_LOOP

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

#endif

// End
