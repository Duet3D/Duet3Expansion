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
			LUT[i] = 255.0 * sinf(((float)i/Resolution) * (Pi / 2.0));
		}

		return LUT;
	}();

	static_assert(lookupTable[0] == 0.0);
	static_assert(lookupTable[Resolution] == 255.0);

	void FastSinCos(uint16_t phase, float& sine, float& cosine) noexcept;
}

// Calculate 255 * minus the sine and plus the cosine of the phase value passed, where phase is between 0 and 4095, and 4096 would correspond to 2*pi
inline void Trigonometry::FastSinCos(uint16_t phase, float& msine, float& cosine) noexcept
{
	unsigned int quadrant = (phase / Resolution) & 3;
	unsigned int index = phase % Resolution;

	const float r1 =lookupTable[index];
	const float r2 = lookupTable[Resolution - index];
	switch (quadrant) {
	case 0:
		msine = -r1;
		cosine = r2;
		break;
	case 1:
		msine = -r2;
		cosine = -r1;
		break;
	case 2:
		msine = r1;
		cosine = -r2;
		break;
	case 3:
		msine = r2;
		cosine = r1;
		break;
	}
}

#endif /* SRC_CLOSEDLOOP_TRIGONOMETRY_H_ */
