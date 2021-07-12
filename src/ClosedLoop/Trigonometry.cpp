/*
 * Trigonometry.cpp
 *
 *  Created on: 12 Jul 2021
 *      Author: Louis
 */

#include "Trigonometry.h"

#define PI 3.1415926535898
#define RESOLUTION 1024

// Immediately Invoked Function Expression (IIFE) to calculate the lookup table
// (Note the '()' at the end of the definition)
constexpr std::array<float, RESOLUTION> lookupTable = []
{
	std::array<float, RESOLUTION> LUT = {};

    for (int i = 0; i < RESOLUTION; ++i)
    {
    	LUT[i] = sin(((float)i/(RESOLUTION-1)) * (PI / 2.0));
    }

    return LUT;
}();

static_assert(lookupTable[0] == 0);
static_assert(lookupTable[RESOLUTION-1] == 1);

// Calculates sin in a fast, but approximate way.
// The phase argument is a value between 0-4095
// The return value is an approximate result of sin(2pi * phase/4095)
float Trigonometry::FastSin(uint16_t phase) noexcept
{
	int quadrant = phase / RESOLUTION;
	int index = phase % (RESOLUTION - 1);

	switch(quadrant) {
		case 0:
			return lookupTable[index];
		case 1:
			return lookupTable[RESOLUTION - 1 - index];
		case 2:
			return -lookupTable[index];
		case 3:
			return -lookupTable[RESOLUTION - 1 - index];
		default:
			return 0;
	}
}

// Calculates cos in a fast, but approximate way.
// The phase argument is a value between 0-4095
// The return value is an approximate result of cos(2pi * phase/4095)
float Trigonometry::FastCos(uint16_t phase) noexcept
{
	int quadrant = phase / RESOLUTION;
	int index = phase % (RESOLUTION - 1);

	switch(quadrant) {
		case 0:
			return lookupTable[RESOLUTION - 1 - index];
		case 1:
			return -lookupTable[index];
		case 2:
			return -lookupTable[RESOLUTION - 1 - index];
		case 3:
			return lookupTable[index];
		default:
			return 0;
	}
}
