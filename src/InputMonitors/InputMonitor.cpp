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

bool InputMonitor::Activate()
{
	bool ok = true;
	if (!active)
	{
		if (threshold == 0)
		{
			// Digital input
			const irqflags_t flags = cpu_irq_save();
			ok = port.AttachInterrupt(CommonDigitalPortInterrupt, InterruptMode::change, CallbackParameter(this));
			state = port.Read();
			cpu_irq_restore(flags);
		}
		else
		{
			// Analog port
			state = port.ReadAnalog() >= threshold;
			ok = port.SetAnalogCallback(CommonAnalogPortInterrupt, CallbackParameter(this), 1);
		}
		active = true;
		whenLastSent = millis();
	}

	return ok;
}

void InputMonitor::Deactivate()
{
	//TODO
	active = false;
}

void InputMonitor::DigitalInterrupt()
{
	const bool newState = port.Read();
	if (newState != state)
	{
		state = newState;
		if (active)
		{
			sendDue = true;
			CanInterface::WakeAsyncSenderFromIsr();
		}
	}
}

void InputMonitor::AnalogInterrupt(uint16_t reading)
{
	const bool newState = reading >= threshold;
	if (newState != state)
	{
		state = newState;
		if (active)
		{
			sendDue = true;
			CanInterface::WakeAsyncSenderFromIsr();
		}
	}
}

/*static*/ void InputMonitor::Init()
{
	// Nothing needed here yet
}

/*static*/ void InputMonitor::CommonDigitalPortInterrupt(CallbackParameter cbp)
{
	static_cast<InputMonitor*>(cbp.vp)->DigitalInterrupt();
}

/*static*/ void InputMonitor::CommonAnalogPortInterrupt(CallbackParameter cbp, uint16_t reading)
{
	static_cast<InputMonitor*>(cbp.vp)->AnalogInterrupt(reading);
}

/*static*/ ReadLockedPointer<InputMonitor> InputMonitor::Find(uint16_t handle)
{
	ReadLocker lock(listLock);
	InputMonitor *current = monitorsList;
	while (current != nullptr && current->handle != handle)
	{
		current = current->next;
	}
	return ReadLockedPointer<InputMonitor>(lock, current);
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
				monitorsList = current->next;
			}
			else
			{
				prev->next = current->next;
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
	newMonitor->active = false;
	newMonitor->state = false;
	newMonitor->minInterval = msg.minInterval;
	newMonitor->threshold = msg.threshold;
	newMonitor->sendDue = false;
	String<StringLength50> pinName;
	pinName.copy(msg.pinName, msg.GetMaxPinNameLength(dataLength));
	if (newMonitor->port.AssignPort(pinName.c_str(), reply, PinUsedBy::endstop, (msg.threshold == 0) ? PinAccess::read : PinAccess::readAnalog))
	{
		newMonitor->next = monitorsList;
		monitorsList = newMonitor;
		const bool ok = newMonitor->Activate();
		extra = (newMonitor->state) ? 1 : 0;
		if (!ok)
		{
			reply.copy("Failed to set pin change interrupt");
			return GCodeResult::error;
		}
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
		WriteLocker lock(listLock);

		if (Delete(msg.handle.u.all))
		{
			return GCodeResult::ok;
		}

		reply.printf("Board %u does not have input handle %04x", CanInterface::GetCanAddress(), msg.handle.u.all);
		return GCodeResult::error;
	}

	auto m = Find(msg.handle.u.all);
	if (m.IsNull())
	{
		reply.printf("Board %u does not have input handle %04x", CanInterface::GetCanAddress(), msg.handle.u.all);
		return GCodeResult::error;
	}

	GCodeResult rslt;
	switch (msg.action)
	{
	case CanMessageChangeInputMonitor::actionDoMonitor:
		rslt = (m->Activate()) ? GCodeResult::ok : GCodeResult::error;
		break;

	case CanMessageChangeInputMonitor::actionDontMonitor:
		m->Deactivate();
		rslt = GCodeResult::ok;
		break;

	case CanMessageChangeInputMonitor::actionReturnPinName:
		m->port.AppendPinName(reply);
		rslt = GCodeResult::ok;
		break;

	case CanMessageChangeInputMonitor::actionChangeThreshold:
		m->threshold = msg.param;
		rslt = GCodeResult::ok;
		break;

	case CanMessageChangeInputMonitor::actionChangeMinInterval:
		m->minInterval = msg.param;
		rslt = GCodeResult::ok;
		break;

	default:
		reply.printf("ChangeInputMonitor action #%u not implemented", msg.action);
		rslt = GCodeResult::error;
		break;
	}

	extra = m->state;
	return rslt;
}

/*static*/ void InputMonitor::AddStateChanges(CanMessageInputChanged *msg, uint32_t& timeToWait)
{
	timeToWait = TaskBase::TimeoutUnlimited;
	ReadLocker lock(listLock);

	const uint32_t now = millis();
	for (InputMonitor *p = monitorsList; p != nullptr; p = p->next)
	{
		if (p->sendDue)
		{
			const uint32_t age = now - p->whenLastSent;
			if (age >= p->minInterval)
			{
				if (!msg->AddEntry(p->handle, p->state))
				{
					timeToWait = 0;
					return;
				}
				p->whenLastSent = now;
				p->sendDue = false;
			}
			else
			{
				const uint32_t timeLeft = p->minInterval - age;
				if (timeLeft < timeToWait)
				{
					timeToWait = timeLeft;
				}
			}
		}
	}
}

// End
