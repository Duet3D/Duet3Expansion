/*
 * InputShaper.h
 *
 *  Created on: 20 Feb 2021
 *      Author: David
 */

#ifndef SRC_MOVEMENT_AXISSHAPER_H_
#define SRC_MOVEMENT_AXISSHAPER_H_

#include <RepRapFirmware.h>

#if SUPPORT_DRIVERS

struct CanMessageSetInputShapingNew;

class AxisShaper
{
public:
	AxisShaper() noexcept;

	// Handle a request from the master board to set input shaping parameters
	GCodeResult EutSetInputShaping(const CanMessageSetInputShapingNew& msg, size_t dataLength, const StringRef& reply) noexcept;

	size_t GetNumImpulses() const noexcept { return numImpulses; }
	motioncalc_t GetImpulseSize(size_t n) const noexcept { return coefficients[n]; }
	uint32_t GetImpulseDelay(size_t n) const noexcept { return delays[n]; }

private:
	static constexpr unsigned int MaxImpulses = 5;
	static constexpr float DefaultFrequency = 40.0;
	static constexpr float DefaultDamping = 0.05;

	// Parameters that fully define the shaping
	unsigned int numImpulses;							// the number of impulses
	motioncalc_t coefficients[MaxImpulses];				// the coefficients of all the impulses, must add up to 1.0
	uint32_t delays[MaxImpulses];						// the start delay in step clocks of each impulse, first one is normally zero
};

#endif

#endif /* SRC_MOVEMENT_AXISSHAPER_H_ */
