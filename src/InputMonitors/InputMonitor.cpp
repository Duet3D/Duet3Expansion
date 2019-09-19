/*
 * InputMonitor.cpp
 *
 *  Created on: 17 Sep 2019
 *      Author: David
 */

#include "InputMonitor.h"

#include <CanMessageFormats.h>
#include <Hardware/IoPorts.h>
#include <CAN/CanInterface.h>

InputMonitor *InputMonitor::monitorsList = nullptr;
InputMonitor *InputMonitor::freeList = nullptr;
ReadWriteLock InputMonitor::listLock;

void InputMonitor::Activate()
{
}

void InputMonitor::Deactivate()
{
}

// Delete a monitor. Must own the write lock before calling this.
/*static*/ bool InputMonitor::Delete(uint16_t handle)
{
	InputMonitor *prev = nullptr, *current = monitorsList;
	while (current != nullptr)
	{
		if (current->handle == handle)
		{
			current->Deactivate();
			if (prev == nullptr)
			{
				monitorsList = current;
			}
			else
			{
				prev->next = current;
			}
			current->next = freeList;
			freeList = current;
			return true;
		}
		prev = current;
		current = current->next;
	}

	return false;
}

/*static*/ GCodeResult InputMonitor::Create(const CanMessageCreateInputMonitor& msg, size_t dataLength, const StringRef& reply, uint8_t& extra)
{
	WriteLocker lock(listLock);

	Delete(msg.handle.u.all);						// delete any existing lock with the same handle

	// Allocate a new one
	InputMonitor *newMonitor;
	if (freeList == nullptr)
	{
		newMonitor = new InputMonitor;
	}
	else
	{
		newMonitor = freeList;
		freeList = newMonitor->next;
	}

	newMonitor->handle = msg.handle.u.all;
	String<StringLength50> pinName;
	pinName.copy(msg.pinName, msg.GetMaxPinNameLength(dataLength));
	if (newMonitor->port.AssignPort(pinName.c_str(), reply, PinUsedBy::endstop, PinAccess::read))
	{
		newMonitor->next = monitorsList;
		monitorsList = newMonitor;
		newMonitor->Activate();
		extra = (newMonitor->state) ? 1 : 0;
		return GCodeResult::ok;
	}

	newMonitor->next = freeList;
	freeList = newMonitor;
	return GCodeResult::error;
}

/*static*/ GCodeResult InputMonitor::Change(const CanMessageChangeInputMonitor& msg, const StringRef& reply, uint8_t& extra)
{
	if (msg.action == CanMessageChangeInputMonitor::actionDelete)
	{
		{
			WriteLocker lock(listLock);

			if (Delete(msg.handle.u.all))
			{
				return GCodeResult::ok;
			}
		}
		reply.printf("Board %u does not have input handle %04x", CanInterface::GetCanAddress(), msg.handle.u.all);
		return GCodeResult::error;
	}

	ReadLocker lock(listLock);

	reply.copy("Change input monitor not yet implemented");
//	extra = (state) ? 1 : 0;
	return GCodeResult::error;
}

// End
