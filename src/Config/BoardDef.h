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

#ifdef EXP3HC_V05
# include "EXP3HC_v05.h"
#endif

#ifdef TOOL1LC_V04
# include "TOOL1LC_v04.h"
#endif

#define SUPPORT_CAN_EXPANSION	1

constexpr size_t MaxSensorsInSystem = 64;
typedef uint64_t SensorsBitmap;

constexpr size_t MaxZProbes = 4;
constexpr size_t MaxZProbeProgramBytes = 8;				// Maximum number of bytes in a Z probe program

// Fan defaults
constexpr size_t NumTotalFans = 12;
constexpr float DefaultMinFanPwm = 0.1;					// minimum fan PWM
constexpr uint32_t DefaultFanBlipTime = 100;			// fan blip time in milliseconds

#endif /* SRC_CONFIG_BOARDDEF_H_ */
