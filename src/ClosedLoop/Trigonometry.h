/*
 * Trigonometry.h
 *
 *  Created on: 12 Jul 2021
 *      Author: Louis
 */

#ifndef SRC_CLOSEDLOOP_TRIGONOMETRY_H_
# define SRC_CLOSEDLOOP_TRIGONOMETRY_H_

#include <array>
#include <math.h>
#include <RepRapFirmware.h>

namespace Trigonometry
{
	constexpr unsigned int Resolution = 1024;

	// Immediately Invoked Function Expression (IIFE) to calculate the lookup table
	// (Note the '()' at the end of the definition)
	static constexpr std::array<float, Resolution + 1> lookupTable = [] () noexcept
	{
		std::array<float, Resolution + 1> LUT = {};

		for (unsigned int i = 0; i <= Resolution; ++i)
		{
			LUT[i] = 248.0 * sinf(((float)i/Resolution) * (Pi / 2.0));
		}

		return LUT;
	}();

	static_assert(lookupTable[0] == 0.0);
	static_assert(lookupTable[Resolution] == 248.0);

	void FastSinCos(uint16_t phase, float& sine, float& cosine) noexcept;
}

// Calculate 248 * the sine and cosine of the phase value passed, where phase is between 0 and 4095, and 4096 would correspond to 2*pi
// The phase is normally in the range 0 to 4095 but when tuning it can be 0 to somewhat over 8192. We must take it modulo 4096.
// The reason for using 248n not 255 is this paragraph from section 16.2 of the TMC2160A data sheet:
// "The maximum resulting swing of the wave should be adjusted to a range of -248 to 248, in order to give the best possible resolution while
//  leaving headroom for the hysteresis-based chopper to add an offset."
inline void Trigonometry::FastSinCos(uint16_t phase, float& sine, float& cosine) noexcept
post(fabsf(sin) <= 248.0; fabsf(cosine) <= 248.0)
{
	unsigned int quadrant = (phase / Resolution) & 3;
	unsigned int index = phase % Resolution;

	const float r1 = lookupTable[index];
	const float r2 = lookupTable[Resolution - index];
	switch (quadrant) {
	case 0:
		sine = r1;
		cosine = r2;
		break;
	case 1:
		sine = r2;
		cosine = -r1;
		break;
	case 2:
		sine = -r1;
		cosine = -r2;
		break;
	case 3:
		sine = -r2;
		cosine = r1;
		break;
	}
}

#endif /* SRC_CLOSEDLOOP_TRIGONOMETRY_H_ */
