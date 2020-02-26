/*
 * Startup.h
 *
 *  Created on: 26 Mar 2018
 *      Author: David
 */

#ifndef SRC_TASKS_H_
#define SRC_TASKS_H_

#include "RepRapFirmware.h"
#include "RTOSIface/RTOSIface.h"

void AppMain();

namespace Tasks
{
	uint32_t GetNeverUsedRam();
	Mutex* GetSpiMutex();
	void Diagnostics(const StringRef& reply);
	uint32_t DoDivide(uint32_t a, uint32_t b);
}

#endif /* SRC_TASKS_H_ */
