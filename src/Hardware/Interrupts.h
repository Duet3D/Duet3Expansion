/*
 * PinInterrupts.h
 *
 *  Created on: 6 Jul 2019
 *      Author: David

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 3 as published by the Free Software Foundation.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef SRC_HARDWARE_PININTERRUPTS_H_
#define SRC_HARDWARE_PININTERRUPTS_H_

#include "RepRapFirmware.h"

// Functions and macrozs to enable/disable interrupts

static inline void cpu_irq_enable()
{
	__DMB();
	__enable_irq();
}

static inline void cpu_irq_disable()
{
	__disable_irq();
	__DMB();
}

typedef bool irqflags_t;

static inline bool cpu_irq_is_enabled()
{
	return __get_PRIMASK() == 0;
}

static inline irqflags_t cpu_irq_save(void)
{
	const irqflags_t flags = cpu_irq_is_enabled();
	cpu_irq_disable();
	return flags;
}

static inline bool cpu_irq_is_enabled_flags(irqflags_t flags)
{
	return flags;
}

static inline void cpu_irq_restore(irqflags_t flags)
{
	if (cpu_irq_is_enabled_flags(flags))
	{
		cpu_irq_enable();
	}
}

// Pin change interrupt support

enum class InterruptMode : uint8_t
{
	none = 0,
	low,
	high,
	change,
	falling,
	rising
};

typedef void (*StandardCallbackFunction)(CallbackParameter);

void InitialisePinChangeInterrupts();
bool AttachInterrupt(Pin pin, StandardCallbackFunction callback, InterruptMode mode, CallbackParameter param);
void DetachInterrupt(Pin pin);

// Return true if we are in any interrupt service routine
static inline bool inInterrupt()
{
	return (__get_IPSR() & 0x01FF) != 0;
}

#endif /* SRC_HARDWARE_PININTERRUPTS_H_ */
