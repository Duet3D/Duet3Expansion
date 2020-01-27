/*
 * BoardDef.h
 *
 *  Created on: 30 Jun 2019
 *      Author: David
 */

#ifndef SRC_CONFIG_BOARDDEF_H_
#define SRC_CONFIG_BOARDDEF_H_

#include <Duet3Limits.h>								// this file is in the CANlib project because both main and expansion boards need it

constexpr float DefaultThermistorR25 = 100000.0;
constexpr float DefaultThermistorbeta = 4388.0;
constexpr float DefaultThermistorC = 0.0;

#ifdef EXP3HC_V09
# include "EXP3HC_v09.h"
#endif

#ifdef TOOL1LC_V04
# include "TOOL1LC_v04.h"
#endif

#ifdef TOOL1LC_V06
# include "TOOL1LC_v06.h"
#endif

#ifdef TOOL1XD_V04
# include "TOOL1XD_v04.h"
#endif

#define SUPPORT_CAN_EXPANSION	1

constexpr float DefaultMinFanPwm = 0.1;					// minimum fan PWM
constexpr uint32_t DefaultFanBlipTime = 100;			// fan blip time in milliseconds

#endif /* SRC_CONFIG_BOARDDEF_H_ */
