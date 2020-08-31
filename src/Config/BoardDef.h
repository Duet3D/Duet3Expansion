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

#ifndef DIFFERENTIAL_STEPPER_OUTPUTS
# define DIFFERENTIAL_STEPPER_OUTPUTS	0
#endif

#ifndef SUPPORT_CLOSED_LOOP
# define SUPPORT_CLOSED_LOOP			0
#endif

#endif /* SRC_CONFIG_BOARDDEF_H_ */
