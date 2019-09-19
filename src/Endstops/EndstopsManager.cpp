/*
 * Endstop.cpp
 *
 *  Created on: 3 Apr 2019
 *      Author: David
 */

#include "EndstopsManager.h"

#include "Endstop.h"
#include "StallDetectionEndstop.h"
#include "ZProbe.h"
#include "LocalZProbe.h"
#include "CanMessageFormats.h"
#include "CAN/CanInterface.h"

static ReadWriteLock endstopsLock;					// used to lock both endstops and Z probes

static EndstopOrZProbe * volatile activeEndstops;	// linked list of endstops and Z probes that are active for the current move

static Endstop *axisEndstops[MaxAxes];				// the endstops assigned to each axis (each one may have several switches), each may be null
static ZProbe *zProbes[MaxZProbes];					// the Z probes used

static bool isHomingMove;							// true if calls to CheckEndstops are for the purpose of homing

// Return a read-locked handle to a Z probe
static ReadLockedPointer<ZProbe> FindZProbe(size_t index)
{
	ReadLocker locker(endstopsLock);
	return ReadLockedPointer<ZProbe>(locker, (index < MaxZProbes) ? zProbes[index] : nullptr);
}

//static ReadLockedPointer<Endstop> FindEndstop(size_t axis);

// Add an endstop to the active list
static void AddToActive(EndstopOrZProbe& e)
{
	e.SetNext(activeEndstops);
	activeEndstops = &e;
}

void EndstopsManager::Init()
{
	for (Endstop *& es : axisEndstops)
	{
		es = nullptr;
	}
	for (ZProbe *& zp : zProbes)
	{
		zp = nullptr;
	}
	activeEndstops = nullptr;

	// Z probes
//	Platform::InitZProbeFilters();
}

GCodeResult EndstopsManager::CreateZProbe(const CanMessageCreateZProbe& msg, size_t dataLength, const StringRef& reply)
{
	if (msg.probeNumber >= MaxZProbes)
	{
		reply.copy("Invalid Z probe index");
		return GCodeResult::error;
	}

	if (msg.probeType >= (uint8_t)ZProbeType::numTypes || msg.probeType == (uint8_t)ZProbeType::none || msg.probeType == (uint8_t)ZProbeType::dumbModulated)
	{
		reply.printf("Invalid remote Z probe type %u", msg.probeType);
		return GCodeResult::error;
	}

	const ZProbeType probeType = (ZProbeType)msg.probeType;

	WriteLocker lock(endstopsLock);

	ZProbe * const existingProbe = zProbes[msg.probeNumber];
	zProbes[msg.probeNumber] = nullptr;
	delete existingProbe;					// delete the old probe first, the new one might use the same ports

	ZProbe *newProbe;
	switch ((ZProbeType)probeType)
	{
	case ZProbeType::zMotorStall:
		newProbe = new MotorStallZProbe(msg.probeNumber);
		break;

	default:
		newProbe = new LocalZProbe(msg.probeNumber, probeType);
		break;
	}

	const GCodeResult rslt = newProbe->ConfigurePorts(msg, dataLength, reply);
	if (rslt == GCodeResult::ok || rslt == GCodeResult::warning)
	{
		zProbes[msg.probeNumber] = newProbe;
	}
	else
	{
		delete newProbe;
	}

	return rslt;
}

GCodeResult EndstopsManager::ConfigureZProbe(const CanMessageConfigureZProbe& msg, const StringRef& reply, uint8_t& extra)
{
	auto zp = FindZProbe(msg.number);
	if (zp.IsNull())
	{
		reply.printf("Board %u does not have Z probe %u", CanInterface::GetCanAddress(), msg.number);
		return GCodeResult::error;
	}

	return zp->Configure(msg, reply, extra);
}

GCodeResult EndstopsManager::GetZProbePinNames(const CanMessageGetZProbePinNames& msg, const StringRef& reply)
{
	auto zp = FindZProbe(msg.number);
	if (zp.IsNull())
	{
		reply.printf("Board %u does not have Z probe %u", CanInterface::GetCanAddress(), msg.number);
		return GCodeResult::error;
	}

	return zp->AppendPinNames(reply);
}

GCodeResult EndstopsManager::DestroyZProbe(const CanMessageDestroyZProbe& msg, const StringRef& reply)
{
	WriteLocker lock(endstopsLock);

	if (msg.number >= MaxZProbes || zProbes[msg.number] == nullptr)
	{
		reply.printf("Board %u does not have Z probe %u", CanInterface::GetCanAddress(), msg.number);
		return GCodeResult::error;
	}

	ZProbe *zp = zProbes[msg.number];
	zProbes[msg.number] = nullptr;
	delete zp;
	return GCodeResult::ok;
}

GCodeResult EndstopsManager::SetProbing(const CanMessageSetProbing& msg, const StringRef& reply)
{
	auto zp = FindZProbe(msg.number);
	if (zp.IsNull())
	{
		reply.printf("Board %u does not haver Z probe %u", CanInterface::GetCanAddress(), msg.number);
		return GCodeResult::error;
	}

	zp->SetProbing(msg.isProbing != 0);
	return GCodeResult::ok;
}

// Set up the active endstops for Z probing
void EndstopsManager::EnableZProbe(size_t probeNumber)
{
	activeEndstops = nullptr;
	isHomingMove = false;
	if (probeNumber < MaxZProbes && zProbes[probeNumber] != nullptr)
	{
		AddToActive(*zProbes[probeNumber]);
	}
}

#if 0
// Enable extruder endstops
void EndstopsManager::EnableExtruderEndstop(size_t extruder)
{
#ifdef NO_EXTRUDER_ENDSTOPS
	// do nothing for  now
#else
	qq;		//TODO
#endif
}
#endif

// Check the endstops.
// If an endstop has triggered, remove it from the active list, return its action, and return a pointer to it via 'es'.
EndstopHitDetails EndstopsManager::CheckEndstops(bool goingSlow)
{
	EndstopHitDetails ret;									// the default constructor will clear all fields
	EndstopOrZProbe *actioned = nullptr;
	for (EndstopOrZProbe *esp = activeEndstops; esp != nullptr; esp = esp->GetNext())
	{
		EndstopHitDetails hd = esp->CheckTriggered(goingSlow);
		if (hd.GetAction() == EndstopHitAction::stopAll)
		{
			activeEndstops = nullptr;						// no need to do anything else
			if (!isHomingMove)
			{
				hd.setAxisHigh = false;
				hd.setAxisLow = false;
			}
			return hd;
		}
		if (hd.GetAction() > ret.GetAction())
		{
			ret = hd;
			actioned = esp;
		}
	}

	if (ret.GetAction() > EndstopHitAction::reduceSpeed)
	{
		if (actioned->Acknowledge(ret))
		{
			// The actioned endstop has completed so remove it from the active list
			EndstopOrZProbe *prev = nullptr;
			for (EndstopOrZProbe *es = activeEndstops; es != nullptr; )
			{
				if (es == actioned)
				{
					if (prev == nullptr)
					{
						activeEndstops = es->GetNext();
					}
					else
					{
						prev->SetNext(es->GetNext());
					}
					break;
				}
				prev = es;
				es = es->GetNext();
			}
		}
		if (!isHomingMove)
		{
			ret.setAxisHigh = false;
			ret.setAxisLow = false;
		}
	}
	return ret;
}

#if 0
// Handle signalling of a remote switch change, when the handle indicates that it is being used as an endstop.
// We must re-use or free the buffer.
void EndstopsManager::HandleRemoteInputChange(CanAddress src, uint8_t handleMajor, uint8_t handleMinor, bool state, CanMessageBuffer* buf)
{
	if (handleMajor < ARRAY_SIZE(axisEndstops))
	{
		Endstop * const es = axisEndstops[handleMajor];
		if (es != nullptr && es->HandleRemoteInputChange(src, handleMinor, state, buf))
		{
			return;					// buffer has been re-used
		}
	}
	CanMessageBuffer::Free(buf);
}
#endif

// End
