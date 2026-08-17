/*
 * InductiveHeaterCalibrationParameters.h
 *
 *  Created on: 14 Aug 2026
 *      Author: David
 *
 *  Parameters that we store in nonvolatile memory after calibrating an inductive heater
 */

#ifndef SRC_PLATFORM_INDUCTIVEHEATERCALIBRATIONPARAMETERS_H_
#define SRC_PLATFORM_INDUCTIVEHEATERCALIBRATIONPARAMETERS_H_

#include <cstdint>

// CAUTION! if this struct is changed then we will need to adjust NonVolatileMemory.h
struct InductiveHeaterCalibrationParameters
{
	uint16_t firstOnTime;
	uint16_t mainOnTime;
	uint16_t offTime;
	uint16_t spare;									// this word is unused. Set is to 0xFFFF to make it easy to use in future.
};

#endif /* SRC_PLATFORM_INDUCTIVEHEATERCALIBRATIONPARAMETERS_H_ */
