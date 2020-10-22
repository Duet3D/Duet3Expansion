/*
 * FilamentSensor.cpp
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#include "FilamentMonitor.h"

#if SUPPORT_DRIVERS

#include "SimpleFilamentMonitor.h"
#include "RotatingMagnetFilamentMonitor.h"
#include "LaserFilamentMonitor.h"
#include "PulsedFilamentMonitor.h"
#include "Platform.h"
#include "Movement/Move.h"
#include <CAN/CanInterface.h>
#include <CanMessageFormats.h>
#include <CanMessageGenericParser.h>

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
	port.Release();				// this also detaches the ISR
}

// Do the configuration that is common to all filament monitor types
// Try to get the pin number from the GCode command in the buffer, setting Seen if a pin number was provided and returning true if error.
// Also attaches the ISR.
// For a remote filament monitor, this does the full configuration or query of the remote object instead, and we always return seen true because we don't need to report local status.
GCodeResult FilamentMonitor::CommonConfigure(const CanMessageGenericParser& parser, const StringRef& reply, InterruptMode interruptMode, bool& seen) noexcept
{
	String<StringLength20> portName;
	if (parser.GetStringParam('C', portName.GetRef()))
	{
		seen = true;
		if (!port.AssignPort(portName.c_str(), reply, PinUsedBy::filamentMonitor, PinAccess::read))
		{
			return GCodeResult::error;
		}

		haveIsrStepsCommanded = false;
		if (interruptMode != InterruptMode::none && !port.AttachInterrupt(InterruptEntry, interruptMode, this))
		{
			reply.copy("unsuitable pin");
			return GCodeResult::error;
		}
	}
	return GCodeResult::ok;
}

// Static initialisation
/*static*/ void FilamentMonitor::InitStatic() noexcept
{
	// Nothing needed here yet
}

// Create a new filament monitor, or replace an existing one
/*static*/ GCodeResult FilamentMonitor::Create(const CanMessageCreateFilamentMonitor& msg, const StringRef& reply) noexcept
{
	const uint8_t p_driver = msg.driver;
	if (p_driver >= NumDrivers)
	{
		reply.copy("Driver number out of range");
		return GCodeResult::error;
	}

	WriteLocker lock(filamentMonitorsLock);

	// Delete any existing filament monitor
	FilamentMonitor *fm = nullptr;
	std::swap(fm, filamentSensors[p_driver]);
	delete fm;

	// Create the new one
	const uint8_t monitorType = msg.type;
	switch (msg.type)
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
		return GCodeResult::error;
	}

	filamentSensors[p_driver] = fm;
	return GCodeResult::ok;
}

// Delete a filament monitor
/*static*/ GCodeResult FilamentMonitor::Delete(const CanMessageDeleteFilamentMonitor& msg, const StringRef& reply) noexcept
{
	const uint8_t p_driver = msg.driver;
	if (p_driver >= NumDrivers)
	{
		reply.copy("Driver number out of range");
		return GCodeResult::error;
	}

	WriteLocker lock(filamentMonitorsLock);

	FilamentMonitor *fm = nullptr;
	std::swap(fm, filamentSensors[p_driver]);

	if (fm == nullptr)
	{
		reply.printf("Driver %u.%u has no filament monitor", CanInterface::GetCanAddress(), p_driver);
		return GCodeResult::warning;
	}

	delete fm;
	return GCodeResult::ok;
}

// Configure a filament monitor
/*static*/ GCodeResult FilamentMonitor::Configure(const CanMessageGeneric& msg, const StringRef& reply) noexcept
{
	CanMessageGenericParser parser(msg, ConfigureFilamentMonitorParams);
	uint8_t p_driver;
	if (!parser.GetUintParam('d', p_driver) || p_driver >= NumDrivers)
	{
		reply.copy("Bad or missing driver number");
		return GCodeResult::error;
	}

	WriteLocker lock(filamentMonitorsLock);

	FilamentMonitor *fm = filamentSensors[p_driver];
	if (fm == nullptr)
	{
		reply.printf("Driver %u.%u has no filament monitor", CanInterface::GetCanAddress(), p_driver);
		return GCodeResult::error;
	}

	return fm->Configure(parser, reply);
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
		fm->isrExtruderStepsCommanded = moveInstance->GetAccumulatedExtrusion(fm->driver, fm->isrWasPrinting);
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
/*static*/ void FilamentMonitor::GetDiagnostics(const StringRef& reply) noexcept
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
			fs->Diagnostics(reply);
		}
	}
}

#endif	// SUPPORT_DRIVERS

// End
