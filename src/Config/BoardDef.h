/*
 * BoardDef.h
 *
 *  Created on: 30 Jun 2019
 *      Author: David
 */

#ifndef SRC_CONFIG_BOARDDEF_H_
#define SRC_CONFIG_BOARDDEF_H_

constexpr float DefaultThermistorR25 = 100000.0;
constexpr float DefaultThermistorbeta = 4388.0;
constexpr float DefaultThermistorC = 0.0;

#ifdef EXP3HC_V09
# include "EXP3HC_v09.h"
#endif

#ifdef TOOL1LC_V04
# include "TOOL1LC_v04.h"
#endif

#ifdef TOOL1XD_V04
# include "TOOL1XD_v04.h"
#endif

#define SUPPORT_CAN_EXPANSION	1

constexpr size_t MaxSensorsInSystem = 64;				// should match the value in RepRapFirmware
typedef uint64_t SensorsBitmap;

constexpr size_t MaxZProbes = 4;
constexpr size_t MaxZProbeProgramBytes = 8;				// maximum number of bytes in a Z probe program

constexpr size_t MaxGpioPorts = 16;						// should match the value in RepRapFirmware

// Fan defaults
constexpr size_t MaxFans = 12;							// should match the value in RepRapFirmware
constexpr float DefaultMinFanPwm = 0.1;					// minimum fan PWM
constexpr uint32_t DefaultFanBlipTime = 100;			// fan blip time in milliseconds

#endif /* SRC_CONFIG_BOARDDEF_H_ */
