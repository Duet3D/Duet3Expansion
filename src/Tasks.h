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
	void GetMemoryReport(const StringRef& reply);
	void GetTasksMemoryReport(const StringRef& reply);
	uint32_t GetNeverUsedRam();
	Mutex* GetSpiMutex();
}

#endif /* SRC_TASKS_H_ */
