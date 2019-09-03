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

#ifdef EXPANSION_1_V09
# include "Expansion1_v09.h"
#endif

#ifdef EXPANSION_1_V05
# include "Expansion1_v05.h"
#endif

#ifdef TOOL_1_V01
# include "Tool1_v01.h"
#endif

#define SUPPORT_CAN_EXPANSION	1

constexpr size_t MaxSensorsInSystem = 64;
typedef uint64_t SensorsBitmap;

// Fan defaults
constexpr size_t NumTotalFans = 12;
constexpr float DefaultMinFanPwm = 0.1;					// minimum fan PWM
constexpr uint32_t DefaultFanBlipTime = 100;			// fan blip time in milliseconds

#endif /* SRC_CONFIG_BOARDDEF_H_ */
