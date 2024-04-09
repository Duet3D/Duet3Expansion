/*
 * FilamentSensor.h
 *
 *  Created on: 20 Jul 2017
 *      Author: David
 */

#ifndef SRC_FILAMENTSENSORS_FILAMENTMONITOR_H_
#define SRC_FILAMENTSENSORS_FILAMENTMONITOR_H_

#include <RepRapFirmware.h>

#if SUPPORT_DRIVERS

#include <Hardware/IoPorts.h>
#include <RTOSIface/RTOSIface.h>
#include <Duet3Common.h>

class CanMessageGeneric;
class CanMessageCreateFilamentMonitor;
class CanMessageDeleteFilamentMonitor;
class CanMessageConfigureFilamentMonitor;
class CanMessageGenericParser;
class FilamentMonitorDataNew;

class FilamentMonitor
{
public:
	// Override the virtual destructor if your derived class allocates any dynamic memory
	virtual ~FilamentMonitor() noexcept;

	FilamentMonitor(const FilamentMonitor&) = delete;

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

protected:
	FilamentMonitor(uint8_t p_driver, unsigned int t) noexcept;

	// Configure this sensor, returning an error code and setting 'seen' if we processed any configuration parameters
	virtual GCodeResult Configure(const CanMessageGenericParser& parser, const StringRef& reply) noexcept = 0;

	// Print diagnostic info for this sensor
	virtual void Diagnostics(const StringRef& reply) noexcept = 0;

	// ISR for when the pin state changes. It should return true if the ISR wants the commanded extrusion to be fetched.
	virtual bool Interrupt() noexcept = 0;

	// Call this to disable the interrupt before deleting a filament monitor
	virtual void Disable() noexcept;

	// Return the type of this sensor
	unsigned int GetType() const noexcept { return type; }

	// Return the enable state of this sensor
	uint8_t GetEnableMode() const noexcept { return enableMode; }

	// Call the following at intervals to check the status. This is only called when extrusion is in progress or imminent.
	// 'filamentConsumed' is the net amount of extrusion since the last call to this function.
	virtual FilamentSensorStatus Check(bool isPrinting, bool fromIsr, uint32_t isrMillis, float filamentConsumed) noexcept = 0;

	// Clear the measurement state - called when we are not printing a file. Return the present/not present status if available.
	virtual FilamentSensorStatus Clear() noexcept = 0;

	// Store collected data in a CAN message slot returning true if there was data worth sending
	virtual void GetLiveData(FilamentMonitorDataNew& data) const noexcept = 0;

	// Configuration common to all filament monitor types
	GCodeResult CommonConfigure(const CanMessageGenericParser& parser, const StringRef& reply, InterruptMode interruptMode, bool& seen) noexcept;

	uint8_t GetDriver() const noexcept { return driver; }
	const IoPort& GetPort() const noexcept { return port; }
	bool HaveIsrStepsCommanded() const noexcept { return haveIsrStepsCommanded; }

#if SUPPORT_AS5601
	bool IsDirectMagneticEncoder() const noexcept { return port.GetPin() == MfmPin; }
#endif

	static int32_t ConvertToPercent(float f)
	{
		return lrintf(100 * f);
	}

private:
	static void InterruptEntry(CallbackParameter param) noexcept;

#if SUPPORT_AS5601
	static void AS5601VirtualInterruptEntry(CallbackParameter param) noexcept;
#endif

	static FilamentMonitor *filamentSensors[NumDrivers];
	static uint32_t whenStatusLastSent;
	static size_t firstDriveToSend;

	static uint32_t minInterruptTime, maxInterruptTime;
	static uint32_t minPollTime, maxPollTime;

	static constexpr uint32_t StatusUpdateInterval = 2000;				// how often we send status reports when there isn't a change
	static constexpr uint32_t LiveStatusUpdateInterval = 250;			// how often we report live status

	int32_t isrExtruderStepsCommanded;
	uint32_t lastIsrMillis;
	unsigned int type;
	IoPort port;
	uint8_t driver;

	uint8_t enableMode;													// 0 = disabled, 1 = enabled when SD card printing, 2 = always enabled
	bool isrWasPrinting;
	bool haveIsrStepsCommanded;
	FilamentSensorStatus lastStatus;
};

#endif	// SUPPORT_DRIVERS

#endif /* SRC_FILAMENTSENSORS_FILAMENTMONITOR_H_ */
