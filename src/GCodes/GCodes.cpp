/*
 * GCodes.cpp
 *
 *  Created on: 10 Sep 2018
 *      Author: David
 */

#include "GCodes.h"

namespace GCodes
{
	static bool toggle = false;

	void Init()
	{

	}

	void Spin()
	{

	}

	// Called by the Move class to get a movement set by the last G Code
	bool ReadMove(RawMove& m)
	{
		toggle = !toggle;

		m.canPauseAfter = true;
		m.endStopsToCheck = 0;
		m.feedRate = 100.0;
		m.hasExtrusion = false;
		m.isCoordinated = true;
		m.isFirmwareRetraction = false;
		m.moveType = 0;
		m.proportionLeft = 1.0;
		m.usePressureAdvance = false;
		m.usingStandardFeedrate = true;
		m.virtualExtruderPosition = 0.0;
		m.xAxes = 1 << X_AXIS;
		m.yAxes = 1 << Y_AXIS;
		for (size_t i = 0; i < ARRAY_SIZE(m.coords); ++i)
		{
			m.coords[i] = 0.0;
		}
		if (toggle)
		{
			m.initialCoords[X_AXIS] = 0.0;
			m.initialCoords[Y_AXIS] = 0.0;
			m.initialCoords[Z_AXIS] = 0.0;
			m.coords[X_AXIS] = 200.0;
			m.coords[Y_AXIS] = 100.0;
			m.coords[Z_AXIS] = 50.0;
		}
		else
		{
			m.initialCoords[X_AXIS] = 200.0;
			m.initialCoords[Y_AXIS] = 100.0;
			m.initialCoords[Z_AXIS] = 50.0;
			m.coords[X_AXIS] = 0.0;
			m.coords[Y_AXIS] = 0.0;
			m.coords[Z_AXIS] = 0.0;
		}
		return true;
	}
}

// End
