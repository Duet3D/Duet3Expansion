/*
 * Startup.h
 *
 *  Created on: 26 Mar 2018
 *      Author: David
 */

#ifndef SRC_TASKS_H_
#define SRC_TASKS_H_

#include "RTOSIface/RTOSIface.h"

void AppMain();
void delay(uint32_t ms);

namespace Tasks
{
	uint32_t GetNeverUsedRam();
}

#endif /* SRC_TASKS_H_ */
