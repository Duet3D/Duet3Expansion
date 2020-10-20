/*
 * SimpleFilamentSensor.cpp
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#include "SimpleFilamentMonitor.h"
#include "Platform.h"

SimpleFilamentMonitor::SimpleFilamentMonitor(unsigned int extruder, unsigned int monitorType) noexcept
	: FilamentMonitor(extruder, monitorType), highWhenNoFilament(monitorType == 2), filamentPresent(false), enabled(false)
{
}

#if 0
// Configure this sensor, returning true if error and setting 'seen' if we processed any configuration parameters
GCodeResult SimpleFilamentMonitor::Configure(GCodeBuffer& gb, const StringRef& reply, bool& seen)
{
	const GCodeResult rslt = CommonConfigure(gb, reply, INTERRUPT_MODE_NONE, seen);
	if (rslt <= GCodeResult::warning)
	{
		if (gb.Seen('S'))
		{
			seen = true;
			enabled = (gb.GetIValue() > 0);
		}

		if (seen)
		{
			Check(false, false, 0, 0.0);
			reprap.SensorsUpdated();
		}
		else
		{
			reply.copy("Simple filament sensor on pin ");
			GetPort().AppendPinName(reply);
			reply.catf(", %s, output %s when no filament, filament present: %s",
						(enabled) ? "enabled" : "disabled",
						(highWhenNoFilament) ? "high" : "low",
						(filamentPresent) ? "yes" : "no");
		}
	}
	return rslt;
}
#endif

// ISR for when the pin state changes
bool SimpleFilamentMonitor::Interrupt() noexcept
{
	// Nothing needed here
	GetPort().DetachInterrupt();
	return false;
}

// Call the following regularly to keep the status up to date
void SimpleFilamentMonitor::Poll() noexcept
{
	const bool b = GetPort().ReadDigital();
	filamentPresent = (highWhenNoFilament) ? !b : b;
}

// Call the following at intervals to check the status. This is only called when extrusion is in progress or imminent.
// 'filamentConsumed' is the net amount of extrusion since the last call to this function.
FilamentSensorStatus SimpleFilamentMonitor::Check(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed) noexcept
{
	Poll();
	return (!enabled || filamentPresent) ? FilamentSensorStatus::ok : FilamentSensorStatus::noFilament;
}

// Clear the measurement state - called when we are not printing a file. Return the present/not present status if available.
FilamentSensorStatus SimpleFilamentMonitor::Clear() noexcept
{
	Poll();
	return (!enabled || filamentPresent) ? FilamentSensorStatus::ok : FilamentSensorStatus::noFilament;
}

// Print diagnostic info for this sensor
void SimpleFilamentMonitor::Diagnostics(const StringRef& reply, unsigned int extruder) noexcept
{
	Poll();
	reply.lcatf("Extruder %u sensor: %s\n", extruder, (filamentPresent) ? "ok" : "no filament");
}

// End
