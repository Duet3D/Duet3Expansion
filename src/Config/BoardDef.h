/*
 * BoardDef.h
 *
 *  Created on: 30 Jun 2019
 *      Author: David
 */

#ifndef SRC_CONFIG_BOARDDEF_H_
#define SRC_CONFIG_BOARDDEF_H_

#include <Duet3Common.h>								// this file is in the CANlib project because both main and expansion boards need it

#ifdef EXP3HC
# include "EXP3HC.h"
#endif

#ifdef TOOL1LC
# include "TOOL1LC.h"
#endif

#ifdef EXP1XD
# include "EXP1XD.h"
#endif

#ifdef EXP1HCE
# include "EXP1HCE.h"
#endif

#ifdef SAMMYC21
# include "SAMMYC21.h"
#endif

#ifdef ATECM
# include "ATECM.h"
#endif

#ifdef ATEIO
# include "ATEIO.h"
#endif

// Default board features
#ifndef DIFFERENTIAL_STEPPER_OUTPUTS
# define DIFFERENTIAL_STEPPER_OUTPUTS	0
#endif

#ifndef USE_TC_FOR_STEP
# define USE_TC_FOR_STEP				0
#endif

#ifndef SUPPORT_CLOSED_LOOP
# define SUPPORT_CLOSED_LOOP			0
#endif

#if !SUPPORT_DRIVERS
# define HAS_SMART_DRIVERS				0
# define SUPPORT_TMC22xx				0
# define SUPPORT_TMC51xx				0
# define SUPPORT_SLOW_DRIVERS			0
constexpr size_t NumDrivers = 0;
#endif

#endif /* SRC_CONFIG_BOARDDEF_H_ */
