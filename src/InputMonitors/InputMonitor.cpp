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
#include <Movement/StepTimer.h>

#if SUPPORT_LDC1612
# include <CommandProcessing/ScanningSensorHandler.h>
#endif

#if SUPPORT_AS5601
# include <CommandProcessing/MFMHandler.h>
#endif

#if SUPPORT_ADS131M02
# include <Platform/Platform.h>
#endif

// Time constant of the rolling average that a tare latches, as a power of 2 samples. At the load cell's ~1300Hz this is about 50ms,
// which divides the +/-50 count noise by 8 without making the tare stale if the head has just moved
constexpr unsigned int AverageShift = 6;

// Time constant of the baseline tracker, as a power of 2 samples. At ~1300Hz this is a first-order corner of ~0.4Hz,
// which follows thermal drift (all below 0.1Hz on the INDX) while a step such as locking a tool decays within ~1.5s
constexpr unsigned int TrackShift = 9;

InputMonitor * volatile InputMonitor::monitorsList = nullptr;
ReadWriteLock InputMonitor::listLock;

InputMonitor::~InputMonitor()
{
#if SUPPORT_LDC1612
	if (port.IsLdc1612())
	{
		ScanningSensorHandler::Deactivate();		// make sure that the scanning sensor doesn't retain a pointer to this object
	}
#endif
#if SUPPORT_ADS131M02
	if (port.IsAds131M02())
	{
		Platform::GetLoadCellAdc()->Deactivate();	// make sure that the load cell ADC doesn't retain a pointer to this object
	}
#endif
}

// Activate the monitor returning true if successful
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
#if SAME5x
				ok = port.AttachInterrupt(CommonDigitalPortInterrupt, InterruptMode::changeWithDebounce, CallbackParameter(this));
				delay(2);								// give the debouncer enough time to get the correct state
				state = (ok) ? port.ReadDebouncedDigital() : port.ReadDigital();
#else
				AtomicCriticalSectionLocker lock;
				ok = port.AttachInterrupt(CommonDigitalPortInterrupt, InterruptMode::change, CallbackParameter(this));
				state = port.ReadDigital();
#endif
			}
		}
		else
		{
			// Analog port
#if SUPPORT_LDC1612
			if (port.IsLdc1612())
			{
				ok = ScanningSensorHandler::Activate(*this);
			}
			else
#endif
#if SUPPORT_ADS131M02
				if (port.IsAds131M02())
				{
					ok = Platform::GetLoadCellAdc()->Activate(*this);
				}
				else
#endif
			{
				state = ReachedThreshold(port.ReadAnalog());
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
#if SUPPORT_LDC1612
			if (port.IsLdc1612())
			{
				ScanningSensorHandler::Deactivate();
			}
			else
#endif
#if SUPPORT_ADS131M02
				if (port.IsAds131M02())
				{
					Platform::GetLoadCellAdc()->Deactivate();
				}
				else
#endif
			{
				port.ClearAnalogCallback();
			}
		}
	}
	active = false;
}

// A negative threshold means the reading falls to it on contact, which is what a load cell does unless the bridge is wired the other way round.
// Probing towards a surface above the head produces the opposite sign again, so both directions have to work
bool InputMonitor::ReachedThreshold(int32_t reading) const noexcept
{
	const int64_t tared = (int64_t)reading - baseline;		// 64-bit because a railed reading and a baseline of opposite sign overflow int32
	return (threshold >= 0) ? tared >= threshold : tared <= threshold;
}

// Return the analog value of this input, relative to the baseline so that a tared handle reports what the threshold is compared against.
// The baseline is zero unless the handle has been tared, so most handles still report the raw reading
int32_t InputMonitor::GetAnalogValue() const noexcept
{
	return (!IsDigital()) ? (int32_t)constrain<int64_t>((int64_t)port.ReadAnalog() - baseline, std::numeric_limits<int32_t>::min(), std::numeric_limits<int32_t>::max())
#if SAME5x
			: port.ReadDebouncedDigital()
#else
			: port.ReadDigital()
#endif
			  ? std::numeric_limits<int32_t>::max()
				: 0;
}

#if SUPPORT_LDC1612

// Set the sensor drive current
GCodeResult InputMonitor::SetDriveLevel(uint32_t param, const StringRef& reply, uint8_t& extra) noexcept
{
	if (port.IsLdc1612())
	{
		return ScanningSensorHandler::SetOrCalibrateCurrent(param, reply, extra);
	}
	reply.copy("drive level not applicable to this port");
	return GCodeResult::error;
}

// Select touch mode and set the sensitivity
GCodeResult InputMonitor::SelectTouchMode(uint32_t param, const StringRef& reply, uint8_t& extra) noexcept
{
	if (port.IsLdc1612())
	{
		const GCodeResult ret = ScanningSensorHandler::SelectTouchMode(param, reply, extra);
		if (ret < GCodeResult::error)
		{
			isLdcInTouchMode = true;
			state = false;						// not triggered
		}
		return ret;
	}
	reply.copy("touch mode not applicable to this port");
	return GCodeResult::error;
}

// Set the state of this monitor to triggered and report it to the main board. Used when the probe is in touch mode.
void InputMonitor::SetTriggered() noexcept
{
	state = true;
	whenStateChanged = StepTimer::GetTimerTicks();
	sendDue = true;
	CanInterface::WakeAsyncSender();
}

#endif

void InputMonitor::DigitalInterrupt() noexcept
{
	const bool newState =
#if SAME5x
							port.ReadDebouncedDigital();
#else
							port.ReadDigital();
#endif
	if (newState != state)
	{
		state = newState;
		if (active)
		{
			whenStateChanged = StepTimer::GetTimerTicks();
			sendDue = true;
			CanInterface::WakeAsyncSenderFromIsr();
		}
	}
}

void InputMonitor::AnalogInterrupt(int32_t reading) noexcept
{
	// Seed the average from the first sample, because converging from zero against a large raw offset takes a visible fraction of a second
	if (averageValid)
	{
		averageAccumulator += reading - (averageAccumulator >> AverageShift);
	}
	else
	{
		averageAccumulator = (int64_t)reading << AverageShift;
	}
	averageReading = (int32_t)(averageAccumulator >> AverageShift);
	averageValid = true;							// set after publishing averageReading so that a concurrent tare cannot latch a stale value

	if (tracking)
	{
		// The tracker accumulator is touched by this task only; a tare re-points it via the reseed flag so the two tasks never share the 64-bit value
		if (trackerReseedPending)
		{
			trackerAccumulator = (int64_t)baseline << TrackShift;
			trackerReseedPending = false;
		}
		trackerAccumulator += reading - (trackerAccumulator >> TrackShift);
		baseline = (int32_t)(trackerAccumulator >> TrackShift);
	}

	const bool newState = ReachedThreshold(reading);
	if (newState != state)
	{
		state = newState;
		if (active)
		{
			whenStateChanged = StepTimer::GetTimerTicks();
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

/*static*/ void InputMonitor::CommonAnalogPortInterrupt(CallbackParameter cbp, int32_t reading) noexcept
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

			delete current;
			return true;
		}
		prev = current;
		current = current->next;
	}

	return false;
}

/*static*/ GCodeResult InputMonitor::Create(const CanMessageCreateInputMonitorV1& msg, size_t dataLength, const StringRef& reply, uint8_t& extra) noexcept
{
	WriteLocker lock(listLock);

	Delete(msg.handle.all);						// delete any existing monitor with the same handle

	// Allocate a new one
	InputMonitor *newMonitor = new InputMonitor;
	newMonitor->handle = msg.handle.all;
	newMonitor->active = false;
	newMonitor->state = false;
	newMonitor->minInterval = msg.minInterval;
	newMonitor->threshold = msg.threshold;
	newMonitor->baseline = 0;
	newMonitor->averageAccumulator = 0;
	newMonitor->trackerAccumulator = 0;
	newMonitor->averageReading = 0;
	newMonitor->averageValid = false;
	newMonitor->tracking = false;
	newMonitor->trackerReseedPending = false;
	newMonitor->sendDue = false;
#if SUPPORT_LDC1612
	newMonitor->isLdcInTouchMode = false;
#endif
	String<StringLength50> pinName;
	pinName.copy(msg.pinName, msg.GetMaxPinNameLength(dataLength));
	if (newMonitor->port.AssignPort(pinName.c_str(), reply, PinUsedBy::endstop, (msg.threshold == 0) ? PinAccess::read : PinAccess::readAnalog))
	{
#if SUPPORT_ADS131M02
		// The fast trigger path reads the ADC directly, so '!' would invert only the reported value; the sign of M558 V covers polarity instead
		if (newMonitor->port.IsAds131M02() && newMonitor->port.GetTotalInvert())
		{
			reply.copy("Pin inversion is not supported on the load cell input, use the sign of M558 V instead");
			delete newMonitor;
			return GCodeResult::error;
		}
#endif
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

	delete newMonitor;
	return GCodeResult::error;
}

/*static*/ GCodeResult InputMonitor::Change(const CanMessageChangeInputMonitorV1& msg, const StringRef& reply, uint8_t& extra, uint32_t *words, size_t& numWords) noexcept
{
	if (msg.action == CanMessageChangeInputMonitorV1::actionDelete)
	{
		WriteLocker lock(listLock);

		if (Delete(msg.handle.all))
		{
			return GCodeResult::ok;
		}

		reply.printf("Board %u does not have input handle %04x", CanInterface::GetCanAddress(), msg.handle.all);
		return GCodeResult::warning;					// only a warning when deleting a non-existent handle
	}

	auto m = Find(msg.handle.all);
	if (m.IsNull())
	{
		reply.printf("Board %u does not have input handle %04x", CanInterface::GetCanAddress(), msg.handle.all);
		return GCodeResult::error;
	}

	GCodeResult rslt;
	switch (msg.action)
	{
	case CanMessageChangeInputMonitorV1::actionDoMonitor:
		rslt = (m->Activate()) ? GCodeResult::ok : GCodeResult::error;
		break;

	case CanMessageChangeInputMonitorV1::actionDontMonitor:
		m->Deactivate();
#if SUPPORT_LDC1612
		if (m->port.IsLdc1612())
		{
			ScanningSensorHandler::ClearTouchMode();
		}
#endif
		rslt = GCodeResult::ok;
		break;

	case CanMessageChangeInputMonitorV1::actionReturnPinName:
		m->port.AppendPinName(reply);
		reply.catf(", min interval %ums", m->minInterval);
		rslt = GCodeResult::ok;
		break;

	case CanMessageChangeInputMonitorV1::actionChangeThreshold:
		m->threshold = (int32_t)msg.param;
		m->state = m->ReachedThreshold(m->port.ReadAnalog());
#if SUPPORT_LDC1612
		if (m->port.IsLdc1612())
		{
			ScanningSensorHandler::ClearTouchMode();
		}
#endif
		rslt = GCodeResult::ok;
		break;

	case CanMessageChangeInputMonitorV1::actionChangeMinInterval:
		m->minInterval = msg.param;
#if SUPPORT_LDC1612
		if (m->port.IsLdc1612())
		{
			ScanningSensorHandler::ClearTouchMode();
		}
#endif
		rslt = GCodeResult::ok;
		break;

#if SUPPORT_LDC1612
	case CanMessageChangeInputMonitorV1::actionSetDriveLevel:
		rslt = m->SetDriveLevel(msg.param, reply, extra);
		break;

	case CanMessageChangeInputMonitorV1::actionSelectTouchMode:
		rslt = m->SelectTouchMode(msg.param, reply, extra);
		break;
#endif

	case CanMessageChangeInputMonitorV1::actionTare:
		if (m->IsDigital())
		{
			reply.printf("Board %u cannot tare digital input handle %04x", CanInterface::GetCanAddress(), msg.handle.all);
			rslt = GCodeResult::error;
		}
		else
		{
			if (msg.param != CanMessageChangeInputMonitorV1::paramTrackOnly)
			{
				m->tracking = false;					// stop the sampling task moving the baseline before latching it; it preempts us, so once this is visible the baseline is ours
				m->baseline = (m->averageValid) ? m->averageReading : m->port.ReadAnalog();
				m->state = m->ReachedThreshold(m->port.ReadAnalog());
			}
			if (msg.param != CanMessageChangeInputMonitorV1::paramTareAndHold)
			{
				m->trackerReseedPending = true;
				m->tracking = true;
			}
			words[0] = (uint32_t)m->baseline;
			numWords = 1;
			rslt = GCodeResult::ok;
		}
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
/*static*/ uint32_t InputMonitor::AddStateChanges(CanMessageInputChangedV2 *msg) noexcept
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
				bool monitorState;
				{
					AtomicCriticalSectionLocker ilock;
					p->sendDue = false;
					monitorState = p->state;
				}

				if (msg->AddEntry(p->handle, StepTimer::ConvertToMasterTime(p->whenStateChanged), p->GetAnalogValue(), monitorState))
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
	const uint16_t mask = req.mask.all;
	const uint16_t pattern = req.pattern.all & mask;

	// Construct the new message in the same buffer
	auto reply = buf->SetupResponseMessage<CanMessageReadInputsReplyV0>(rid, CanInterface::GetCanAddress(), srcAddress);

	unsigned int count = 0;
	ReadLocker lock(listLock);
	InputMonitor *h = monitorsList;
	while (h != nullptr && count < ARRAY_SIZE(reply->results))
	{
		if ((h->handle & mask) == pattern)
		{
			reply->results[count].handle.Set(h->handle);
			StoreLEI32(&reply->results[count].reading, h->GetAnalogValue());
			++count;
		}
		h = h->next;
	}

	reply->numReported = count;
	reply->resultCode = (uint32_t)((count == 0) ? GCodeResult::error : GCodeResult::ok);
	buf->dataLength = reply->GetActualDataLength();
}

// Append analog handle data to the supplied buffer
/*static*/ unsigned int InputMonitor::AddAnalogHandleDataV1(uint8_t *buffer, size_t spaceLeft) noexcept
{
	unsigned int count = 0;
	ReadLocker lock(listLock);
	InputMonitor *h = monitorsList;
	while (h != nullptr && spaceLeft >= sizeof(AnalogHandleDataV1))
	{
		if (!h->IsDigital())
		{
			AnalogHandleDataV1 data;
			data.handle.all = h->handle;
			data.when = (uint16_t)StepTimer::GetMasterTime();
			data.reading = h->GetAnalogValue();
			memcpy(buffer, &data, sizeof(AnalogHandleDataV1));
			buffer += sizeof(AnalogHandleDataV1);
			spaceLeft -= sizeof(AnalogHandleDataV1);
			++count;
		}
		h = h->next;
	}
	return count;
}

// End
