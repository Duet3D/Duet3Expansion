/*
 * Startup.h
 *
 *  Created on: 26 Mar 2018
 *      Author: David
 */

#ifndef SRC_TASKS_H_
#define SRC_TASKS_H_

#include "RepRapFirmware.h"
#include <RTOSIface/RTOSIface.h>

[[noreturn]] void AppMain();

namespace Tasks
{
	ptrdiff_t GetNeverUsedRam() noexcept;
	void *AllocPermanent(size_t sz, std::align_val_t align = (std::align_val_t)__STDCPP_DEFAULT_NEW_ALIGNMENT__) noexcept;
	void Diagnostics(const StringRef& reply) noexcept;
	uint32_t DoDivide(uint32_t a, uint32_t b) noexcept;
	uint32_t DoMemoryRead(const uint32_t* addr) noexcept;
}

// Functions called by CanMessageBuffer in CANlib
void *MessageBufferAlloc(size_t sz, std::align_val_t align) noexcept;
void MessageBufferDelete(void *ptr, std::align_val_t align) noexcept;

#endif /* SRC_TASKS_H_ */
