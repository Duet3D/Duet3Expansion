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
#include <Platform/Platform.h>
#include <Movement/Move.h>
#include <CAN/CanInterface.h>
#include <CanMessageFormats.h>
#include <CanMessageBuffer.h>
#include <CanMessageGenericParser.h>
#include <CanMessageGenericTables.h>

#if SUPPORT_AS5601
# include <CommandProcessing/MFMHandler.h>
#endif

// Static data
ReadWriteLock FilamentMonitor::filamentMonitorsLock;
FilamentMonitor *FilamentMonitor::filamentSensors[NumDrivers] = { 0 };
uint32_t FilamentMonitor::whenStatusLastSent = 0;
size_t FilamentMonitor::firstDriveToSend = 0;
uint32_t FilamentMonitor::minInterruptTime = 0xFFFFFFFF, FilamentMonitor::maxInterruptTime = 0;
uint32_t FilamentMonitor::minPollTime = 0xFFFFFFFF, FilamentMonitor::maxPollTime = 0;

// Constructor
FilamentMonitor::FilamentMonitor(uint8_t p_driver, unsigned int t) noexcept
	: type(t), driver(p_driver), enableMode(0), lastStatus(FilamentSensorStatus::noDataReceived)
{
}

// Default destructor
FilamentMonitor::~FilamentMonitor() noexcept
{
#if SUPPORT_AS5601
	if (IsDirectMagneticEncoder())
	{
		MFMHandler::DetachEncoderVirtualInterrupt(this);
	}
#endif
}

// Call this to disable the interrupt before deleting or re-configuring a local filament monitor
void FilamentMonitor::Disable() noexcept
{
#if SUPPORT_AS5601
	if (IsDirectMagneticEncoder())
	{
		MFMHandler::DetachEncoderVirtualInterrupt(this);
	}
#endif
	port.Release();				// this also detaches the ISR
}

// Do the configuration that is common to all filament monitor types
// Try to get the pin number from the GCode command in the buffer, setting Seen if a pin number was provided and returning true if error.
// Also attaches the ISR.
// For a remote filament monitor, this does the full configuration or query of the remote object instead, and we always return seen true because we don't need to report local status.
GCodeResult FilamentMonitor::CommonConfigure(const CanMessageGenericParser& parser, const StringRef& reply, InterruptMode interruptMode, bool& seen) noexcept
{
	if (parser.GetUintParam('S', enableMode))
	{
		seen = true;
		if (enableMode > 2)
		{
			enableMode = 2;
		}
	}

	String<StringLength20> portName;
	if (parser.GetStringParam('C', portName.GetRef()))
	{
		seen = true;
		if (!port.AssignPort(portName.c_str(), reply, PinUsedBy::filamentMonitor, PinAccess::read))
		{
			return GCodeResult::error;
		}

		haveIsrStepsCommanded = false;

#if SUPPORT_AS5601
		if (IsDirectMagneticEncoder())
		{
			if (type != 3)
			{
				reply.copy("wrong filament monitor type for this port");
				return GCodeResult::error;
			}
			if (!MFMHandler::AttachEncoderVirtualInterrupt(AS5601VirtualInterruptEntry, this))
			{
				reply.copy("encoder not found or already in use");
				return GCodeResult::error;
			}
		}
		else
#endif
		{
			if (interruptMode != InterruptMode::none && !port.AttachInterrupt(InterruptEntry, interruptMode, CallbackParameter(this)))
			{
				reply.copy("unsuitable pin");
				return GCodeResult::error;
			}
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
	switch(f.RawValue())
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
	const uint32_t startTime = StepTimer::GetTimerTicks();
	FilamentMonitor * const fm = static_cast<FilamentMonitor*>(param.vp);
	if (fm->Interrupt())
	{
		fm->isrExtruderStepsCommanded = moveInstance->GetAccumulatedExtrusion(fm->driver, fm->isrWasPrinting);
		fm->haveIsrStepsCommanded = true;
		fm->lastIsrMillis = millis();
	}
	const uint32_t elapsedTime = StepTimer::GetTimerTicks() - startTime;
	if (elapsedTime > maxInterruptTime)
	{
		maxInterruptTime = elapsedTime;
	}
	if (elapsedTime < minInterruptTime)
	{
		minInterruptTime = elapsedTime;
	}
}

#if SUPPORT_AS5601

// Virtual ISR from AS5601 task
/*static*/ void FilamentMonitor::AS5601VirtualInterruptEntry(CallbackParameter param) noexcept
{
	FilamentMonitor * const fm = static_cast<FilamentMonitor*>(param.vp);
	fm->isrExtruderStepsCommanded = moveInstance->GetAccumulatedExtrusion(fm->driver, fm->isrWasPrinting);
	fm->haveIsrStepsCommanded = true;
	fm->lastIsrMillis = millis();
}

#endif

// Check the status of all the filament monitors.
// Currently, the status for all filament monitors (on expansion boards as well as on the main board) is checked by the main board, which generates any necessary events.
/*static*/ void FilamentMonitor::Spin() noexcept
{
	CanMessageBuffer buf;
	auto msg = buf.SetupStatusMessage<CanMessageFilamentMonitorsStatusNew>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress());
	size_t slotIndex = 0;
	size_t firstDriveNotSent = NumDrivers;
	Bitmap<uint32_t> driversReported;
	bool forceSend = false, haveLiveData = false;

	{
		ReadLocker lock(filamentMonitorsLock);

		for (size_t drv = 0; drv < NumDrivers; ++drv)
		{
			FilamentSensorStatus fst(FilamentSensorStatus::noMonitor);
			if (filamentSensors[drv] != nullptr)
			{
				const uint32_t startTime = StepTimer::GetTimerTicks();
				FilamentMonitor& fs = *filamentSensors[drv];
				bool isPrinting;
				bool fromIsr;
				int32_t extruderStepsCommanded;
				uint32_t locIsrMillis;
				IrqDisable();
				if (fs.haveIsrStepsCommanded)
				{
					extruderStepsCommanded = fs.isrExtruderStepsCommanded;
					isPrinting = fs.isrWasPrinting;
					locIsrMillis = fs.lastIsrMillis;
					fs.haveIsrStepsCommanded = false;
					IrqEnable();
					fromIsr = true;
				}
				else
				{
					extruderStepsCommanded = moveInstance->GetAccumulatedExtrusion(drv, isPrinting);		// get and clear the net extrusion commanded
					IrqEnable();
					fromIsr = false;
					locIsrMillis = 0;
				}

				if (fs.enableMode == 2 || Platform::IsPrinting())
				{
					const float extrusionCommanded = (float)extruderStepsCommanded/Platform::DriveStepsPerUnit(drv);
					fst = fs.Check(isPrinting, fromIsr, locIsrMillis, extrusionCommanded);
				}
				else
				{
					fst = fs.Clear();
				}

				const uint32_t elapsedTime = StepTimer::GetTimerTicks() - startTime;
				if (elapsedTime > maxPollTime)
				{
					maxPollTime = elapsedTime;
				}
				if (elapsedTime < minPollTime)
				{
					minPollTime = elapsedTime;
				}

				if (drv >= firstDriveToSend)
				{
					if (slotIndex < ARRAY_SIZE(msg->data))
					{
						auto& slot = msg->data[slotIndex];
						slot.status = fst.ToBaseType();
						fs.GetLiveData(slot);
						if (fst != fs.lastStatus)
						{
							forceSend = true;
							fs.lastStatus = fst;
						}
						else if (slot.hasLiveData)
						{
							haveLiveData = true;
						}
						driversReported.SetBit(drv);
						++slotIndex;
					}
					else if (drv < firstDriveNotSent)
					{
						firstDriveNotSent = drv;
					}
				}
			}
		}
	}

	uint32_t now;
	if (   slotIndex != 0
		&& (   forceSend
			|| (now = millis()) - whenStatusLastSent >= StatusUpdateInterval
			|| (haveLiveData && now - whenStatusLastSent >= LiveStatusUpdateInterval)

		   )
	   )
	{
		msg->SetStandardFields(driversReported);
		buf.dataLength = msg->GetActualDataLength();
		CanInterface::Send(&buf);
		whenStatusLastSent = millis();
	}
	firstDriveToSend = (firstDriveNotSent < NumDrivers) ? firstDriveNotSent : 0;
}

// Close down the filament monitors, in particular stop them generating interrupts. Called when we are about to update firmware.
/*static*/ void FilamentMonitor::Exit() noexcept
{
	WriteLocker lock(filamentMonitorsLock);

	for (FilamentMonitor *&f : filamentSensors)
	{
		DeleteObject(f);
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
				reply.lcatf("=== Filament sensors ===\nInterrupt %" PRIu32 " to %" PRIu32 "us, poll %" PRIu32 " to %" PRIu32 "us",
								StepTimer::TicksToIntegerMicroseconds(minInterruptTime), StepTimer::TicksToIntegerMicroseconds(maxInterruptTime),
								StepTimer::TicksToIntegerMicroseconds(minPollTime), StepTimer::TicksToIntegerMicroseconds(maxPollTime));
				minPollTime = minInterruptTime = 0xFFFFFFFF;
				maxPollTime = maxInterruptTime = 0;
				first = false;
			}
			fs->Diagnostics(reply);
		}
	}
}

#endif	// SUPPORT_DRIVERS

// End
