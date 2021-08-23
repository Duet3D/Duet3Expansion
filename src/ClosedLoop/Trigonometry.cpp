/*
 * Trigonometry.cpp
 *
 *  Created on: 12 Jul 2021
 *      Author: Louis
 */

#include "Trigonometry.h"

constexpr unsigned int Resolution = 1024;

// Immediately Invoked Function Expression (IIFE) to calculate the lookup table
// (Note the '()' at the end of the definition)
constexpr std::array<float, Resolution> lookupTable = []
{
	std::array<float, Resolution> LUT = {};

	for (unsigned int i = 0; i < Resolution; ++i)
	{
		LUT[i] = sinf(((float)i/(Resolution-1)) * (Pi / 2.0));
	}

	return LUT;
}();

static_assert(lookupTable[0] == 0);
static_assert(lookupTable[Resolution-1] == 1);

// Calculates sin in a fast, but approximate way.
// The phase argument is a value between 0-4095
// The return value is an approximate result of sin(2pi * phase/4095)
float Trigonometry::FastSin(uint16_t phase) noexcept
{
	unsigned int quadrant = phase / Resolution;
	unsigned int index = phase % Resolution;

	switch(quadrant) {
		case 0:
			return lookupTable[index];
		case 1:
			return lookupTable[Resolution - 1 - index];
		case 2:
			return -lookupTable[index];
		case 3:
			return -lookupTable[Resolution - 1 - index];
		default:
			return 0;
	}
}

// Calculates cos in a fast, but approximate way.
// The phase argument is a value between 0-4095
// The return value is an approximate result of cos(2pi * phase/4095)
float Trigonometry::FastCos(uint16_t phase) noexcept
{
	unsigned int quadrant = phase / Resolution;
	unsigned int index = phase % Resolution;

	switch(quadrant) {
		case 0:
			return lookupTable[Resolution - 1 - index];
		case 1:
			return -lookupTable[index];
		case 2:
			return -lookupTable[Resolution - 1 - index];
		case 3:
			return lookupTable[index];
		default:
			return 0;
	}
}
