/*
 * ZProbe.cpp
 *
 *  Created on: 13 Feb 2018
 *      Author: David
 */

#include "ZProbe.h"
#include "Platform.h"
#include "GCodes/GCodes.h"
#include "Heating/Heat.h"

ZProbe::ZProbe(unsigned int num, ZProbeType p_type) : EndstopOrZProbe(), number(num), type(p_type), adcValue(500), invertReading(false)
{
}

GCodeResult ZProbe::Configure(const CanMessageConfigureZProbe& msg, const StringRef& reply, uint8_t& extra)
{
	invertReading = msg.invertReading;
	adcValue = msg.adcValue;
	type = (ZProbeType)msg.type;
	extra = 0;							// set a default value, which the overriding function should replace
	return GCodeResult::ok;
}

int ZProbe::GetReading() const
{
	const int zProbeVal = GetRawReading()/4;
	return (invertReading) ? 1000 - zProbeVal : zProbeVal;
}

// Test whether we are at or near the stop
EndStopHit ZProbe::Stopped() const
{
	const int zProbeVal = GetReading();
	return (zProbeVal >= adcValue) ? EndStopHit::atStop
			: (zProbeVal * 10 >= adcValue * 9) ? EndStopHit::nearStop	// if we are at/above 90% of the target value
				: EndStopHit::noStop;
}

// Check whether the probe is triggered and return the action that should be performed. Called from the step ISR.
EndstopHitDetails ZProbe::CheckTriggered(bool goingSlow)
{
	EndstopHitDetails rslt;
	switch (Stopped())
	{
	case EndStopHit::atStop:
		rslt.SetAction(EndstopHitAction::stopAll);
		rslt.isZProbe = true;
		break;

	case EndStopHit::nearStop:
		if (!goingSlow)
		{
			rslt.SetAction(EndstopHitAction::reduceSpeed);
		}
		break;

	default:
		break;
	}
	return rslt;
}

// This is called by the ISR to acknowledge that it is acting on the return from calling CheckTriggered. Called from the step ISR.
// Return true if we have finished with this endstop or probe in this move.
bool ZProbe::Acknowledge(EndstopHitDetails what)
{
	return what.GetAction() == EndstopHitAction::stopAll;
}

// Default implementation of SendProgram, overridden in some classes
GCodeResult ZProbe::SendProgram(const uint32_t zProbeProgram[], size_t len, const StringRef& reply)
{
	reply.copy("This configuration of Z probe does not support programming");
	return GCodeResult::error;
}

// End
