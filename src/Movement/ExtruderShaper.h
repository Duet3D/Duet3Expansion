/*
 * PressureAdvanceShaper.h
 *
 *  Created on: 14 May 2021
 *      Author: David
 */

#ifndef SRC_MOVEMENT_EXTRUDERSHAPER_H_
#define SRC_MOVEMENT_EXTRUDERSHAPER_H_

#include <RepRapFirmware.h>

#if SUPPORT_DRIVERS

#include "StepTimer.h"
#include <limits>

// This class holds pressure advance parameters for an extruder.
class ExtruderShaper
{
public:
	ExtruderShaper() noexcept
	{
		SetParametersSimple(0.0);
	}

	void SetParameters(const ShortPressureAdvanceParameters& params) noexcept;
	void SetParametersSimple(float f) noexcept;
	void UpdateStepsPerMm(float stepsPerMm) noexcept;

	bool IsActive() const noexcept { return k0 != (motioncalc_t)0.0; }
	motioncalc_t GetPressureAdvanceDistance(motioncalc_t speed) const noexcept;
	motioncalc_t GetAverageAdvanceClocks(motioncalc_t lowSpeed, motioncalc_t highSpeed, motioncalc_t steps) const noexcept
		pre(highSpeed > lowSpeed);

	void AppendParameters(const StringRef& reply) const noexcept;

private:
	// Specified parameters
	motioncalc_t k0;						// the initial pressure advance constant in step clocks
	motioncalc_t k1;						// the slope of pressure advance distance vs. speed above distance dk
	motioncalc_t dk;						// the pressure advance distance up to which k0 applies

	// Derived parameters
	motioncalc_t vk;						// the speed up to which k1 applies, equal to dk/k1
	motioncalc_t d0;						// the distance at which the k2 line intercepts the y-axis

	motioncalc_t mmPerStep;					// the reciprocal of steps/mm for this drive
};

#endif

#endif /* SRC_MOVEMENT_EXTRUDERSHAPER_H_ */
