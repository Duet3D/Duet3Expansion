/*
 * Endstop.h
 *
 *  Created on: 4 Apr 2019
 *      Author: David
 */

#ifndef SRC_ENDSTOPS_ENDSTOP_H_
#define SRC_ENDSTOPS_ENDSTOP_H_

#include "RepRapFirmware.h"
#include "EndstopDefs.h"
#include "Hardware/IoPorts.h"
#include <General/FreelistManager.h>
#include "CanId.h"

class CanMessageBuffer;

// This is the base class for all types of endstops and for ZProbe.
class EndstopOrZProbe
{
public:
	EndstopOrZProbe() : next(nullptr) {}
	virtual ~EndstopOrZProbe() {}

	virtual EndStopHit Stopped() const = 0;
	virtual EndstopHitDetails CheckTriggered(bool goingSlow) = 0;
	virtual bool Acknowledge(EndstopHitDetails what) = 0;

	EndstopOrZProbe *GetNext() const { return next; }
	void SetNext(EndstopOrZProbe *e) { next = e; }

	static void UpdateStalledDrivers(uint32_t driverMask, bool isStalled);

protected:
	static DriversBitmap GetStalledDrivers() { return stalledDrivers; }

private:
	EndstopOrZProbe *next;								// next endstop in linked list

	static DriversBitmap stalledDrivers;				// used to track which drivers are reported as stalled, for stall detect endstops and stall detect Z probes
};

inline void EndstopOrZProbe::UpdateStalledDrivers(uint32_t driverMask, bool isStalled)
{
	if (isStalled)
	{
		stalledDrivers |= driverMask;
	}
	else
	{
		stalledDrivers &= ~driverMask;
	}
}

class Endstop : public EndstopOrZProbe
{
public:
	virtual EndStopType GetEndstopType() const = 0;

	// Process a remote endstop input change that relates to this endstop. Return true if the buffer has been freed.
	virtual bool HandleRemoteInputChange(CanAddress src, uint8_t handleMinor, bool state, CanMessageBuffer *buf) { return false; }

	unsigned int GetAxis() const { return axis; }
	bool GetAtHighEnd() const { return atHighEnd; }

protected:
	Endstop(uint8_t axis, EndStopPosition pos);

	void SetAtHighEnd(bool b) { atHighEnd = b; }

private:
	uint8_t axis;										// which axis this endstop is on
	bool atHighEnd;										// whether this endstop is at the max (true) or the min (false)
};

#endif /* SRC_ENDSTOPS_ENDSTOP_H_ */
