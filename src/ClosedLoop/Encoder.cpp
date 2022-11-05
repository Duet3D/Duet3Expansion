/*
 * Encoder.cpp
 *
 *  Created on: 5 Nov 2022
 *      Author: David
 */

#include "Encoder.h"

#if SUPPORT_CLOSED_LOOP

// Get the initial set of tuning errors for this encoder
TuningErrors Encoder::MinimalTuningNeeded() const noexcept
{
	TuningErrors ret = 0;
	if (UsesBasicTuning()) { ret |= TuningError::NeedsBasicTuning; }
	if (UsesCalibration()) { ret |= TuningError::NotCalibrated; }
	return ret;
}

#endif

// End
