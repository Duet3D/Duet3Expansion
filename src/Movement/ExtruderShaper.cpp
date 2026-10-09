/*
 * PressureAdvanceShaper.cpp
 *
 *  Created on: 14 May 2021
 *      Author: David
 */

#include "ExtruderShaper.h"

#if SUPPORT_DRIVERS

// Set the pressure advance parameters
void ExtruderShaper::SetParameters(const ShortPressureAdvanceParameters& params) noexcept
{
	k0 = (motioncalc_t)((float)params.k[0] * StepTimer::StepClockRate);
	k1 = (motioncalc_t)((float)params.k[1] * StepTimer::StepClockRate);
	dk = (motioncalc_t)params.dk;
	if (k0 == (motioncalc_t)0.0)
	{
		vk = dk = std::numeric_limits<motioncalc_t>::infinity();
		d0 = (motioncalc_t)0.0;
	}
	else
	{
		vk = dk/k0;
		d0 = dk * ((motioncalc_t)1.0 - (k1/k0));
	}
}

void ExtruderShaper::SetParametersSimple(float f) noexcept
{
	k0 = k1 = (motioncalc_t)f;
	d0 = (motioncalc_t)0.0;
	dk = vk = std::numeric_limits<motioncalc_t>::infinity();
}

void ExtruderShaper::UpdateStepsPerMm(float stepsPerMm) noexcept
{
	mmPerStep = (motioncalc_t)1.0/(motioncalc_t)stepsPerMm;
}

// Get the pressure advance distance for a given extrusion speed. This is not currently used.
motioncalc_t ExtruderShaper::GetPressureAdvanceDistance(motioncalc_t speed) const noexcept
{
	return (speed <= vk) ? k0 * speed : d0 + k1 * speed;
}

// Get the average number of pressure advance clocks for a move segment that changes speed. Must have lowSpeed <= highSpeed.
// lowSpeed and highSpeed are referenced to unit-normalised distances of the whole move, so they are actually the reciprocal of the time taken to the specified number of steps
motioncalc_t ExtruderShaper::GetAverageAdvanceClocks(motioncalc_t lowSpeed, motioncalc_t highSpeed, motioncalc_t steps) const noexcept
{
	const motioncalc_t actualLowSpeed = lowSpeed * steps * mmPerStep;
	const motioncalc_t actualHighSpeed = highSpeed * steps * mmPerStep;

	// Optimisation for when the speed change doesn't cross the knee
	if (actualHighSpeed <= vk)
	{
		return k0;
	}
	if (actualLowSpeed >= vk)
	{
		return k1;
	}

	// actualLowSpeed is below the knee and actualHighSpeed is above it
	const motioncalc_t lowDistance = k0 * actualLowSpeed;
	const motioncalc_t highDistance = d0 + k1 * actualHighSpeed;
	return (highDistance - lowDistance)/(actualHighSpeed - actualLowSpeed);
}

#endif

// End
