/*
 * Kinematics.cpp
 *
 *  Created on: 24 Apr 2017
 *      Author: David
 */

#include "LinearDeltaKinematics.h"
#include "Kinematics.h"
#include "CartesianKinematics.h"
#include "Platform.h"
#include "GCodes/GCodes.h"

// Constructor. Pass segsPerSecond <= 0.0 to get non-segmented kinematics.
Kinematics::Kinematics(KinematicsType t, float segsPerSecond, float minSegLength, bool doUseRawG0)
	: segmentsPerSecond(segsPerSecond), minSegmentLength(minSegLength), useSegmentation(segsPerSecond > 0.0), useRawG0(doUseRawG0), type(t)
{
}

// Return true if the specified XY position is reachable by the print head reference point.
// This default implementation assumes a rectangular reachable area, so it just uses the bed dimensions give in the M208 command.
bool Kinematics::IsReachable(float x, float y, bool isCoordinated) const
{
#if 1
	return true;
#else
	const Platform& platform = reprap.GetPlatform();
	return x >= platform.AxisMinimum(X_AXIS) && y >= platform.AxisMinimum(Y_AXIS) && x <= platform.AxisMaximum(X_AXIS) && y <= platform.AxisMaximum(Y_AXIS);
#endif
}

// Limit the Cartesian position that the user wants to move to, returning true if any coordinates were changed
// This default implementation just applies the rectangular limits set up by M208 to those axes that have been homed.
bool Kinematics::LimitPosition(float coords[], size_t numVisibleAxes, AxesBitmap axesHomed, bool isCoordinated) const
{
	return LimitPositionFromAxis(coords, 0, numVisibleAxes, axesHomed);
}

// Apply the M208 limits to the Cartesian position that the user wants to move to for all axes from the specified one upwards
// Return true if any coordinates were changed
bool Kinematics::LimitPositionFromAxis(float coords[], size_t firstAxis, size_t numVisibleAxes, AxesBitmap axesHomed) const
{
	bool limited = false;
	for (size_t axis = firstAxis; axis < numVisibleAxes; axis++)
	{
		if (IsBitSet(axesHomed, axis))
		{
			float& f = coords[axis];
			if (f < Platform::AxisMinimum(axis))
			{
				f = Platform::AxisMinimum(axis);
				limited = true;
			}
			else if (f > Platform::AxisMaximum(axis))
			{
				f = Platform::AxisMaximum(axis);
				limited = true;
			}
		}
	}
	return limited;
}

// Return the initial Cartesian coordinates we assume after switching to this kinematics
// This default is suitable for Cartesian and CoreXY printers.
void Kinematics::GetAssumedInitialPosition(size_t numAxes, float positions[]) const
{
	for (size_t i = 0; i < numAxes; ++i)
	{
		positions[i] = 0.0;
	}
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
