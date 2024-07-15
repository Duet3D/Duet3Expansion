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
#include <CanMessageBuffer.h>

#if SUPPORT_LDC1612
# include <CommandProcessing/ScanningSensorHandler.h>
#endif

#if SUPPORT_AS5601
# include <CommandProcessing/MFMHandler.h>
#endif

InputMonitor * volatile InputMonitor::monitorsList = nullptr;
InputMonitor * volatile InputMonitor::freeList = nullptr;
ReadWriteLock InputMonitor::listLock;

bool InputMonitor::Activate() noexcept
{
	bool ok = true;
	if (!active)
	{
		if (IsDigital())
		{
			// Digital input
#if SUPPORT_TCA6408A
			// Check if it's the button on the embedded MFM
			if (port.GetPin() == MfmButtonPin)
			{
				state = MFMHandler::EnableButton(this);
			}
			else
#endif
			{
				AtomicCriticalSectionLocker lock;
				ok = port.AttachInterrupt(CommonDigitalPortInterrupt, InterruptMode::change, CallbackParameter(this));
				state = port.ReadDigital();
			}
		}
		else
		{
			// Analog port
			state = port.ReadAnalog() >= threshold;
#ifdef ATEIO
			// We can't set an interrupt on the extended analog channels
			if (IsExtendedAnalogPin(port.GetPin()))
			{
				ok = true;
			}
			else
#endif
			{
				ok = port.SetAnalogCallback(CommonAnalogPortInterrupt, CallbackParameter(this), 1);
			}
		}
		active = true;

		// Respond to the next change within 2ms because we may be enabling an endstop switch or Z probe
		const uint32_t now = millis();
		whenLastSent = (minInterval <= 2) ? now : now - (minInterval - 2);
	}

	return ok;
}

void InputMonitor::Deactivate() noexcept
{
	if (active)
	{
		if (IsDigital())
		{
#if SUPPORT_TCA6408A
			// Check if it's the button on the embedded MFM
			if (port.GetPin() == MfmButtonPin)
			{
				state = MFMHandler::EnableButton(nullptr);
			}
			else
#endif
			{
				port.DetachInterrupt();
			}
		}
		else
		{
			port.ClearAnalogCallback();
		}
	}
	active = false;
}

// Return the analog value of this input
uint32_t InputMonitor::GetAnalogValue() const noexcept
{
	return (!IsDigital()) ? port.ReadAnalog()
			: port.ReadDigital() ? 0xFFFFFFFF
				: 0;
}

// Set the sensor drive current
GCodeResult InputMonitor::SetDriveLevel(uint32_t param, const StringRef& reply, uint8_t& extra) noexcept
{
#if SUPPORT_LDC1612
	if (port.IsLdc1612())
	{
		return ScanningSensorHandler::SetOrCalibrateCurrent(param, reply, extra);
	}
#endif

	reply.copy("drive level not applicable to this port");
	return GCodeResult::error;
}

void InputMonitor::DigitalInterrupt() noexcept
{
	const bool newState = port.ReadDigital();
	if (newState != state)
	{
		state = newState;
		if (active)
		{
			sendDue = true;
			CanInterface::WakeAsyncSender();
		}
	}
}

void InputMonitor::AnalogInterrupt(uint32_t reading) noexcept
{
	const bool newState = reading >= threshold;
	if (newState != state)
	{
		state = newState;
		if (active)
		{
			sendDue = true;
			CanInterface::WakeAsyncSender();
		}
	}
}

#if SUPPORT_AS5601

void InputMonitor::UpdateState(bool newState) noexcept
{
	if (newState != state)
	{
		state = newState;
		if (active)
		{
			sendDue = true;
			CanInterface::WakeAsyncSender();
		}
	}
}

#endif

/*static*/ void InputMonitor::Init() noexcept
{
	// Nothing needed here yet
}

/*static*/ void InputMonitor::Spin() noexcept
{
	// Nothing needed here yet
}

/*static*/ void InputMonitor::CommonDigitalPortInterrupt(CallbackParameter cbp) noexcept
{
	static_cast<InputMonitor*>(cbp.vp)->DigitalInterrupt();
}

/*static*/ void InputMonitor::CommonAnalogPortInterrupt(CallbackParameter cbp, uint32_t reading) noexcept
{
	static_cast<InputMonitor*>(cbp.vp)->AnalogInterrupt(reading);
}

/*static*/ ReadLockedPointer<InputMonitor> InputMonitor::Find(uint16_t hndl) noexcept
{
	ReadLocker lock(listLock);
	InputMonitor *current = monitorsList;
	while (current != nullptr && current->handle != hndl)
	{
		current = current->next;
	}
	return ReadLockedPointer<InputMonitor>(lock, current);
}

// Delete a monitor. Must own the write lock before calling this.
/*static*/ bool InputMonitor::Delete(uint16_t hndl) noexcept
{
	InputMonitor *prev = nullptr, *current = monitorsList;
	while (current != nullptr)
	{
		if (current->handle == hndl)
		{
			current->Deactivate();
			current->port.Release();
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

/*static*/ GCodeResult InputMonitor::Create(const CanMessageCreateInputMonitorNew& msg, size_t dataLength, const StringRef& reply, uint8_t& extra) noexcept
{
	WriteLocker lock(listLock);

	Delete(msg.handle.u.all);						// delete any existing monitor with the same handle

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
			reply.printf("Failed to activate input monitor on pin %u", newMonitor->port.GetPin());
			return GCodeResult::error;
		}
		return GCodeResult::ok;
	}

	newMonitor->next = freeList;
	freeList = newMonitor;
	return GCodeResult::error;
}

/*static*/ GCodeResult InputMonitor::Change(const CanMessageChangeInputMonitorNew& msg, const StringRef& reply, uint8_t& extra) noexcept
{
	if (msg.action == CanMessageChangeInputMonitorNew::actionDelete)
	{
		WriteLocker lock(listLock);

		if (Delete(msg.handle.u.all))
		{
			return GCodeResult::ok;
		}

		reply.printf("Board %u does not have input handle %04x", CanInterface::GetCanAddress(), msg.handle.u.all);
		return GCodeResult::warning;					// only a warning when deleting a non-existent handle
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
	case CanMessageChangeInputMonitorNew::actionDoMonitor:
		rslt = (m->Activate()) ? GCodeResult::ok : GCodeResult::error;
		break;

	case CanMessageChangeInputMonitorNew::actionDontMonitor:
		m->Deactivate();
		rslt = GCodeResult::ok;
		break;

	case CanMessageChangeInputMonitorNew::actionReturnPinName:
		m->port.AppendPinName(reply);
		reply.catf(", min interval %ums", m->minInterval);
		rslt = GCodeResult::ok;
		break;

	case CanMessageChangeInputMonitorNew::actionChangeThreshold:
		m->threshold = msg.param;
		m->state = m->port.ReadAnalog() >= m->threshold;
		rslt = GCodeResult::ok;
		break;

	case CanMessageChangeInputMonitorNew::actionChangeMinInterval:
		m->minInterval = msg.param;
		rslt = GCodeResult::ok;
		break;

	case CanMessageChangeInputMonitorNew::actionSetDriveLevel:
		rslt = m->SetDriveLevel(msg.param, reply, extra);
		break;

	default:
		reply.printf("ChangeInputMonitor action #%u not implemented", msg.action);
		rslt = GCodeResult::error;
		break;
	}

	extra = (m->state) ? 1 : 0;
	return rslt;
}

// Check the input monitors and add any pending ones to the message
// Return the number of ticks before we should be woken again, or TaskBase::TimeoutUnlimited if we shouldn't be woken until an input changes state
/*static*/ uint32_t InputMonitor::AddStateChanges(CanMessageInputChangedNew *msg) noexcept
{
	uint32_t timeToWait = TaskBase::TimeoutUnlimited;
	ReadLocker lock(listLock);

	const uint32_t now = millis();
	for (InputMonitor *p = monitorsList; p != nullptr; p = p->next)
	{
		if (p->sendDue)
		{
			const uint32_t age = now - p->whenLastSent;
			if (age >= p->minInterval)
			{
				bool added;
				{
					InterruptCriticalSectionLocker ilock;
					p->sendDue = false;
					added = msg->AddEntry(p->handle, p->GetAnalogValue(), p->state);
				}
				if (added)
				{
					p->whenLastSent = now;
				}
				else
				{
					p->sendDue = true;
					return 1;
				}
			}
			else
			{
				// The state has changed but we've recently sent a state change for this input
				const uint32_t timeLeft = p->minInterval - age;
				if (timeLeft < timeToWait)
				{
					timeToWait = timeLeft;
				}
			}
		}
	}
	return timeToWait;
}

// Read the specified inputs. The incoming message is a CanMessageReadInputsRequest. We return a CanMessageReadInputsReply in the same buffer.
// Return error if we didn't have any of the requested inputs.
/*static*/ void InputMonitor::ReadInputs(CanMessageBuffer *buf) noexcept
{
	// Extract data before we overwrite the message
	const CanMessageReadInputsRequest& req = buf->msg.readInputsRequest;
	const CanAddress srcAddress = buf->id.Src();
	const uint16_t rid = req.requestId;
	const uint16_t mask = req.mask.u.all;
	const uint16_t pattern = req.pattern.u.all & mask;

	// Construct the new message in the same buffer
	auto reply = buf->SetupResponseMessage<CanMessageReadInputsReply>(rid, CanInterface::GetCanAddress(), srcAddress);

	unsigned int count = 0;
	ReadLocker lock(listLock);
	InputMonitor *h = monitorsList;
	while (h != nullptr && count < ARRAY_SIZE(reply->results))
	{
		if ((h->handle & mask) == pattern)
		{
			reply->results[count].handle.Set(h->handle);
			StoreLEU32(&reply->results[count].reading, h->GetAnalogValue());
			++count;
		}
		h = h->next;
	}

	reply->numReported = count;
	reply->resultCode = (uint32_t)((count == 0) ? GCodeResult::error : GCodeResult::ok);
	buf->dataLength = reply->GetActualDataLength();
}

// Append analog handle data to the supplied buffer
/*static*/ unsigned int InputMonitor::AddAnalogHandleData(uint8_t *buffer, size_t spaceLeft) noexcept
{
	unsigned int count = 0;
	ReadLocker lock(listLock);
	InputMonitor *h = monitorsList;
	while (h != nullptr && spaceLeft >= sizeof(AnalogHandleData))
	{
		if (!h->IsDigital())
		{
			AnalogHandleData data;
			data.reading = h->GetAnalogValue();
			data.handle.u.all = h->handle;
			memcpy(buffer, &data, sizeof(AnalogHandleData));
			buffer += sizeof(AnalogHandleData);
			spaceLeft -= sizeof(AnalogHandleData);
			++count;
		}
		h = h->next;
	}
	return count;
}

// End
