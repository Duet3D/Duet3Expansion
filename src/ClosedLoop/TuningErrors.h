/*
 * TuningErrors.h
 *
 *  Created on: 25 Oct 2022
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_TUNINGERRORS_H_
#define SRC_CLOSEDLOOP_TUNINGERRORS_H_

#include <cstdint>

typedef uint8_t TuningErrors;

// Possible tuning errors
namespace TuningError
{
	constexpr TuningErrors NeedsBasicTuning					= 1u << 0;
	constexpr TuningErrors NotCalibrated					= 1u << 1;
	constexpr TuningErrors TooLittleMotion					= 1u << 2;
	constexpr TuningErrors TooMuchMotion					= 1u << 3;
	constexpr TuningErrors InconsistentMotion				= 1u << 4;
	constexpr TuningErrors HysteresisTooHigh				= 1u << 5;
	constexpr TuningErrors SystemError						= 1u << 6;
	constexpr TuningErrors TuningOrCalibrationInProgress	= 1u << 7;

	// The following covers any situation in which tuning or calibration was attempted but failed
	constexpr TuningErrors AnyTuningFailure 		= SystemError | HysteresisTooHigh
													| TooLittleMotion | TooMuchMotion | InconsistentMotion;
}

#endif /* SRC_CLOSEDLOOP_TUNINGERRORS_H_ */
