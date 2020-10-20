/*
 * FilamentSensor.cpp
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#include "FilamentMonitor.h"
#include "SimpleFilamentMonitor.h"
#include "RotatingMagnetFilamentMonitor.h"
#include "LaserFilamentMonitor.h"
#include "PulsedFilamentMonitor.h"
#include "Platform.h"
#include "Movement/Move.h"

// Static data
ReadWriteLock FilamentMonitor::filamentMonitorsLock;
FilamentMonitor *FilamentMonitor::filamentSensors[NumDrivers] = { 0 };
bool FilamentMonitor::unreportedFilamentError = false;

// Constructor
FilamentMonitor::FilamentMonitor(uint8_t p_driver, unsigned int t) noexcept
	: type(t), driver(p_driver), lastStatus(FilamentSensorStatus::ok)
{
}

// Default destructor
FilamentMonitor::~FilamentMonitor() noexcept
{
}

// Call this to disable the interrupt before deleting or re-configuring a local filament monitor
void FilamentMonitor::Disable() noexcept
{
	port.Release();
}

#if 0
// Do the configuration that is
// Try to get the pin number from the GCode command in the buffer, setting Seen if a pin number was provided and returning true if error.
// Also attaches the ISR.
// For a remote filament monitor, this does the full configuration or query of the remote object instead, and we always return seen true because we don't need to report local status.
GCodeResult FilamentMonitor::CommonConfigure(GCodeBuffer& gb, const StringRef& reply, InterruptMode interruptMode, bool& seen) noexcept
{
	if (gb.Seen('C'))
	{
		seen = true;
		if (!port.AssignPort(gb, reply, PinUsedBy::filamentMonitor, PinAccess::read))
		{
			return GCodeResult::error;
		}

		haveIsrStepsCommanded = false;
		if (interruptMode != INTERRUPT_MODE_NONE && !port.AttachInterrupt(InterruptEntry, interruptMode, this))
		{
			reply.copy("unsuitable pin");
			return GCodeResult::error;
		}
	}
	return GCodeResult::ok;
}
#endif

// Static initialisation
/*static*/ void FilamentMonitor::InitStatic() noexcept
{
	// Nothing needed here yet
}

#if 0
// Handle M591
/*static*/ GCodeResult FilamentMonitor::Configure(GCodeBuffer& gb, const StringRef& reply, unsigned int extruder) THROWS(GCodeException)
{
	bool seen = false;
	uint32_t newSensorType;
	gb.TryGetUIValue('P', newSensorType, seen);

	WriteLocker lock(filamentMonitorsLock);
	FilamentMonitor*& sensor = filamentSensors[extruder];

	if (seen)
	{
		// Creating a filament monitor. First delete the old one for this extruder.
		if (sensor != nullptr)
		{
			sensor->Disable();
			sensor = nullptr;
			std::swap(sensor, filamentSensors[extruder]);
			delete sensor;
			reprap.SensorsUpdated();
		}

		gb.MustSee('C');														// make sure the port name parameter is present
		sensor = Create(extruder, newSensorType, reply);						// create the new sensor
		if (sensor == nullptr)
		{
			return GCodeResult::error;
		}

		try
		{
			const GCodeResult rslt = sensor->Configure(gb, reply, seen);		// configure the sensor (may throw)
			if (rslt <= GCodeResult::warning)
			{
				filamentSensors[extruder] = sensor;
			}
			else
			{
				delete sensor;
			}
			return rslt;
		}
		catch (const GCodeException&)
		{
			delete sensor;
			throw;
		}
	}

	// Here if configuring or reporting on an existing filament monitor
	if (sensor == nullptr)
	{
		reply.printf("Extruder %u has no filament sensor", extruder);
		return GCodeResult::ok;
	}

	return sensor->Configure(gb, reply, seen);									// configure or report on the existing sensor (may throw)
}
#endif

// Factory function to create a filament monitor
/*static*/ FilamentMonitor *FilamentMonitor::Create(uint8_t p_driver, unsigned int monitorType, const StringRef& reply) noexcept
{
	FilamentMonitor *fm;
	switch (monitorType)
	{
	case 1:		// active high switch
	case 2:		// active low switch
		fm = new SimpleFilamentMonitor(p_driver, monitorType);
		break;

	case 3:		// duet3d rotating magnet, no switch
	case 4:		// duet3d rotating magnet + switch
		fm = new RotatingMagnetFilamentMonitor(p_driver, monitorType);
		break;

	case 5:		// duet3d laser, no switch
	case 6:		// duet3d laser + switch
		fm = new LaserFilamentMonitor(p_driver, monitorType);
		break;

	case 7:		// simple pulse output sensor
		fm = new PulsedFilamentMonitor(p_driver, monitorType);
		break;

	default:	// no sensor, or unknown sensor
		reply.printf("Unknown filament monitor type %u", monitorType);
		return nullptr;
	}
	return fm;
}

// Return an error message corresponding to a status code
/*static*/ const char *FilamentMonitor::GetErrorMessage(FilamentSensorStatus f) noexcept
{
	switch(f)
	{
	case FilamentSensorStatus::ok:					return "no error";
	case FilamentSensorStatus::noFilament:			return "no filament";
	case FilamentSensorStatus::tooLittleMovement:	return "too little movement";
	case FilamentSensorStatus::tooMuchMovement:		return "too much movement";
	case FilamentSensorStatus::sensorError:			return "sensor not working";
	default:										return "unknown error";
	}
}

// ISR
/*static*/ void FilamentMonitor::InterruptEntry(CallbackParameter param) noexcept
{
	FilamentMonitor * const fm = static_cast<FilamentMonitor*>(param.vp);
	if (fm->Interrupt())
	{
		fm->isrExtruderStepsCommanded = moveInstance->GetAccumulatedExtrusion(fm->extruderNumber, fm->isrWasPrinting);
		fm->haveIsrStepsCommanded = true;
		fm->lastIsrMillis = millis();
	}
}

/*static*/ void FilamentMonitor::Spin() noexcept
{
	ReadLocker lock(filamentMonitorsLock);

	for (size_t driver = 0; driver < NumDrivers; ++driver)
	{
		if (filamentSensors[driver] != nullptr)
		{
			FilamentMonitor& fs = *filamentSensors[driver];
			{
				bool isPrinting;
				bool fromIsr;
				int32_t extruderStepsCommanded;
				uint32_t locIsrMillis;
				cpu_irq_disable();
				if (fs.haveIsrStepsCommanded)
				{
					extruderStepsCommanded = fs.isrExtruderStepsCommanded;
					isPrinting = fs.isrWasPrinting;
					locIsrMillis = fs.lastIsrMillis;
					fs.haveIsrStepsCommanded = false;
					cpu_irq_enable();
					fromIsr = true;
				}
				else
				{
					cpu_irq_enable();
					extruderStepsCommanded = moveInstance->GetAccumulatedExtrusion(driver, isPrinting);		// get and clear the net extrusion commanded
					fromIsr = false;
					locIsrMillis = 0;
				}

				//TODO
				// In the RRF version, the following code is guarded by "if (gCodes.IsReallyPrinting() && !gCodes.IsSimulating())".
				// We don't need to worry about simulation here, but we ought to take account of whether filament monitoring is enabled or not.
				if (true)
				{
					const float extrusionCommanded = (float)extruderStepsCommanded/Platform::DriveStepsPerUnit(driver);
					fs.lastStatus = fs.Check(isPrinting, fromIsr, locIsrMillis, extrusionCommanded);
					if (fs.lastStatus != FilamentSensorStatus::ok)
					{
#if 0
						if (reprap.Debug(moduleFilamentSensors))
						{
							debugPrintf("Filament error: extruder %u reports %s\n", extruder, FilamentMonitor::GetErrorMessage(fstat));
						}
						else
#endif
						{
							unreportedFilamentError = true;
						}
					}
				}
				else
				{
					fs.lastStatus = fs.Clear();
				}
			}
		}
	}
}

// Close down the filament monitors, in particular stop them generating interrupts. Called when we are about to update firmware.
/*static*/ void FilamentMonitor::Exit() noexcept
{
	WriteLocker lock(filamentMonitorsLock);

	for (FilamentMonitor *&f : filamentSensors)
	{
		FilamentMonitor *temp;
		std::swap(temp, f);
		delete temp;
	}
}

// Return the status of the filament sensor for a drive
/*static*/ FilamentSensorStatus FilamentMonitor::GetFilamentStatus(size_t drive)
{
	TaskCriticalSectionLocker lock;
	FilamentMonitor *fs;
	return (drive >= NumDrivers || (fs = filamentSensors[drive]) == nullptr) ? FilamentSensorStatus::noMonitor : fs->lastStatus;
}

// Send diagnostics info
/*static*/ void FilamentMonitor::Diagnostics(const StringRef& reply) noexcept
{
	bool first = true;
	ReadLocker lock(filamentMonitorsLock);

	for (size_t i = 0; i < NumDrivers; ++i)
	{
		FilamentMonitor * const fs = filamentSensors[i];
		if (fs != nullptr)
		{
			if (first)
			{
				reply.lcat("=== Filament sensors ===");
				first = false;
			}
			fs->Diagnostics(reply, i);
		}
	}
}

// End
