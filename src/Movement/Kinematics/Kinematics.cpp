/*
 * Kinematics.cpp
 *
 *  Created on: 24 Apr 2017
 *      Author: David
 */

#include "LinearDeltaKinematics.h"
#include "Kinematics.h"
#include "CartesianKinematics.h"
#include <Platform/Platform.h>

// Constructor. Pass segsPerSecond <= 0.0 to get non-segmented kinematics.
Kinematics::Kinematics(KinematicsType t, float segsPerSecond, float minSegLength, bool doUseRawG0)
	: segmentsPerSecond(segsPerSecond), minSegmentLength(minSegLength), useSegmentation(segsPerSecond > 0.0), useRawG0(doUseRawG0), type(t)
{
}

/*static*/ Kinematics *Kinematics::Create(KinematicsType k)
{
	switch (k)
	{
	case KinematicsType::linearDeltaPlusZ:	// not implemented yet
	default:
		return nullptr;

	case KinematicsType::cartesian:
		return new CartesianKinematics();
	case KinematicsType::linearDelta:
		return new LinearDeltaKinematics();
#if 0
	case KinematicsType::coreXY:
		return new CoreXYKinematics();
	case KinematicsType::coreXZ:
		return new CoreXZKinematics();
	case KinematicsType::scara:
		return new ScaraKinematics();
	case KinematicsType::coreXYU:
		return new CoreXYUKinematics();
	case KinematicsType::hangprinter:
		return new HangprinterKinematics();
	case KinematicsType::polar:
		return new PolarKinematics();
	case KinematicsType::coreXYUV:
		return new CoreXYUVKinematics();
	case KinematicsType::rotaryDelta:
		return new RotaryDeltaKinematics();
#endif
	}
}

/*static*/ void Kinematics::PrintMatrix(const char* s, const MathMatrix<floatc_t>& m, size_t maxRows, size_t maxCols)
{
	debugPrintf("%s\n", s);
	if (maxRows == 0)
	{
		maxRows = m.rows();
	}
	if (maxCols == 0)
	{
		maxCols = m.cols();
	}

	for (size_t i = 0; i < maxRows; ++i)
	{
		for (size_t j = 0; j < maxCols; ++j)
		{
			debugPrintf("%7.4f%c", (double)m(i, j), (j == maxCols - 1) ? '\n' : ' ');
		}
	}
}

/*static*/ void Kinematics::PrintVector(const char *s, const floatc_t *v, size_t numElems)
{
	debugPrintf("%s:", s);
	for (size_t i = 0; i < numElems; ++i)
	{
		debugPrintf(" %7.4f", (double)v[i]);
	}
	debugPrintf("\n");
}

// End
