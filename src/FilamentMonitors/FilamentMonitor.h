/*
 * FilamentSensor.h
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#ifndef SRC_FILAMENTSENSORS_FILAMENTMONITOR_H_
#define SRC_FILAMENTSENSORS_FILAMENTMONITOR_H_

#include "RepRapFirmware.h"
#include "Hardware/IoPorts.h"
#include "MessageType.h"
#include "GCodes/GCodeResult.h"
#include "RTOSIface/RTOSIface.h"
#include <Duet3Common.h>

class CanMessageGeneric;
class CanMessageCreateFilamentMonitor;
class CanMessageDeleteFilamentMonitor;
class CanMessageConfigureFilamentMonitor;
class CanMessageGenericParser;

class FilamentMonitor
{
public:
	FilamentMonitor(const FilamentMonitor&) = delete;

	// Configure this sensor, returning an error code and setting 'seen' if we processed any configuration parameters
	virtual GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) = 0;

	// Print diagnostic info for this sensor
	virtual void Diagnostics(const StringRef& reply) noexcept = 0;

	// ISR for when the pin state changes. It should return true if the ISR wants the commanded extrusion to be fetched.
	virtual bool Interrupt() noexcept = 0;

	// Call this to disable the interrupt before deleting a filament monitor
	virtual void Disable() noexcept;

	// Override the virtual destructor if your derived class allocates any dynamic memory
	virtual ~FilamentMonitor() noexcept;

	// Return the type of this sensor
	unsigned int GetType() const noexcept { return type; }

	// Static initialisation
	static void InitStatic() noexcept;

	// Return an error message corresponding to a status code
	static const char *GetErrorMessage(FilamentSensorStatus f) noexcept;

	// Poll the filament sensors
	static void Spin() noexcept;

	// Close down the filament monitors, in particular stop them generating interrupts. Called when we are about to update firmware.
	static void Exit() noexcept;

	// Create a new filament monitor, or replace an existing one
	static GCodeResult Create(const CanMessageCreateFilamentMonitor& msg, const StringRef& reply) noexcept;

	// Delete a filament monitor
	static GCodeResult Delete(const CanMessageDeleteFilamentMonitor& msg, const StringRef& reply) noexcept;

	// Configure a filament monitor
	static GCodeResult Configure(const CanMessageGeneric& msg, const StringRef& reply) noexcept;

	// Generate diagnostics info
	static void GetDiagnostics(const StringRef& reply) noexcept;

	// This must be public so that the array descriptor in class RepRap can lock it
	static ReadWriteLock filamentMonitorsLock;

	// Return the status of the filament sensor for a drive
	static FilamentSensorStatus GetFilamentStatus(size_t drive);

	// Check and clear the unreported filament error flag
	static bool CheckUnreportedError()
	{
		TaskCriticalSectionLocker lock;
		const bool ret = unreportedFilamentError;
		unreportedFilamentError = false;
		return ret;
	}

protected:
	FilamentMonitor(uint8_t p_driver, unsigned int t) noexcept;

	// Call the following at intervals to check the status. This is only called when extrusion is in progress or imminent.
	// 'filamentConsumed' is the net amount of extrusion since the last call to this function.
	virtual FilamentSensorStatus Check(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed) noexcept = 0;

	// Clear the measurement state - called when we are not printing a file. Return the present/not present status if available.
	virtual FilamentSensorStatus Clear() noexcept = 0;

	GCodeResult CommonConfigure(const CanMessageGenericParser& parser, const StringRef& reply, InterruptMode interruptMode, bool& seen) noexcept;

	uint8_t GetDriver() const noexcept { return driver; }
	const IoPort& GetPort() const noexcept { return port; }
	bool HaveIsrStepsCommanded() const noexcept { return haveIsrStepsCommanded; }

	static int32_t ConvertToPercent(float f)
	{
		return lrintf(100 * f);
	}

private:
	static void InterruptEntry(CallbackParameter param) noexcept;

	static FilamentMonitor *filamentSensors[NumDrivers];
	static bool unreportedFilamentError;

	int32_t isrExtruderStepsCommanded;
	uint32_t lastIsrMillis;
	unsigned int type;
	IoPort port;
	uint8_t driver;

	bool isrWasPrinting;
	bool haveIsrStepsCommanded;
	FilamentSensorStatus lastStatus;
};

#endif /* SRC_FILAMENTSENSORS_FILAMENTMONITOR_H_ */
