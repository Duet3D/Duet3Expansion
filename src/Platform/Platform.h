/*
 * Platform.h
 *
 *  Created on: 9 Sep 2018
 *      Author: David
 */

#ifndef SRC_PLATFORM_H_
#define SRC_PLATFORM_H_

#include <RepRapFirmware.h>
#include "AveragingFilter.h"
#include <Movement/StepTimer.h>
#include <Heating/Heat.h>
#include <UniqueIdBase.h>

#if SUPPORT_CLOSED_LOOP
# include "ClosedLoop/ClosedLoop.h"
#endif

#if SUPPORT_SPI_SENSORS || SUPPORT_CLOSED_LOOP || defined(ATEIO)
# include <Hardware/SharedSpiDevice.h>
#endif

#if SUPPORT_I2C_SENSORS
# include <Hardware/SharedI2CMaster.h>
# include <Hardware/LIS3DH.h>
#endif

#if RP2040
# include <hardware/structs/sio.h>
#endif

class CanMessageDiagnosticTest;
class CanMessageBuffer;

#if HAS_CPU_TEMP_SENSOR
constexpr size_t McuTempReadingsAveraged = 8;
#endif

#if SUPPORT_THERMISTORS
// Define the number of temperature readings we average for each thermistor. This should be a power of 2 and at least 4 ^ AD_OVERSAMPLE_BITS.
#if SAMC21
constexpr size_t ThermistorReadingsAveraged = 8;	// Reduced to save RAM. We already do x64 averaging in the ADC itself.
#else
constexpr size_t ThermistorReadingsAveraged = 64;
#endif
constexpr size_t ZProbeReadingsAveraged = 8;		// We average this number of readings with IR on, and the same number with IR off
constexpr size_t VinReadingsAveraged = 8;

typedef AveragingFilter<ThermistorReadingsAveraged> ThermistorAveragingFilter;
typedef AveragingFilter<ZProbeReadingsAveraged> ZProbeAveragingFilter;

# if HAS_VREF_MONITOR

constexpr size_t VssaFilterIndex = NumThermistorInputs;
constexpr size_t VrefFilterIndex = NumThermistorInputs + 1;

#  if SAMC21 && SUPPORT_SDADC
// On the SAMC21 we have 2 additional filters for the SDADC inputs
constexpr size_t SdAdcTemp0FilterIndex = NumThermistorInputs + 2;
constexpr size_t SdAdcVrefFilterIndex = NumThermistorInputs + 3;
constexpr size_t NumThermistorFilters = NumThermistorInputs + 4;
#  else
// SAME51 or not supporting SDADC
constexpr size_t NumThermistorFilters = NumThermistorInputs + 2;
#  endif

# else

// No VREF or VSSA monitor inputs
#  if SAMC21 && SUPPORT_SDADC
// On the SAMC21 we have 2 additional filters for the SDADC inputs
constexpr size_t SdAdcTemp0FilterIndex = NumThermistorInputs;
constexpr size_t SdAdcVrefFilterIndex = NumThermistorInputs + 1;
constexpr size_t NumThermistorFilters = NumThermistorInputs + 2;
#  else
constexpr size_t NumThermistorFilters = NumThermistorInputs;
#  endif

# endif
#endif

// Enumeration of error condition bits
enum class ErrorCode : uint32_t
{
	BadTemp = 1u << 0,
	BadMove = 1u << 1,
	OutputStarvation = 1u << 2,
	OutputStackOverflow = 1u << 3,
	HsmciTimeout = 1u << 4
};

class IoPort;

namespace Platform
{
#if SUPPORT_DRIVERS
	// The data would ideally be private, but they are used by inline functions so they can't be unless we convert this namespace to a static class
# if SINGLE_DRIVER
	constexpr uint32_t DriverBit = 1u << (StepPins[0] & 31);
# else
	extern uint32_t driveDriverBits[NumDrivers];
	extern uint32_t allDriverBits;
# endif

# if SUPPORT_SLOW_DRIVERS
	// Support for slow step pulse generation to suit external drivers
	extern uint32_t slowDriverStepTimingClocks[4];

#  if USE_TC_FOR_STEP		// the first element has a special meaning when we use a TC to generate the steps
	inline uint32_t GetSlowDriverStepPeriodClocks() { return slowDriverStepTimingClocks[1]; }
	inline uint32_t GetSlowDriverDirHoldFromLeadingEdgeClocks() { return slowDriverStepTimingClocks[3]; }
#  else
	inline uint32_t GetSlowDriverStepHighClocks() { return slowDriverStepTimingClocks[0]; }
	inline uint32_t GetSlowDriverStepLowClocks() { return slowDriverStepTimingClocks[1]; }
	inline uint32_t GetSlowDriverDirHoldFromTrailingEdgeClocks() { return slowDriverStepTimingClocks[3]; }
#  endif

	inline uint32_t GetSlowDriverDirSetupClocks() { return slowDriverStepTimingClocks[2]; }

	float GetSlowDriverStepHighMicroseconds();
	float GetSlowDriverStepLowMicroseconds();
	float GetSlowDriverDirSetupMicroseconds();
	float GetSlowDriverDirHoldMicroseconds();

	void SetDriverStepTiming(size_t drive, const float timings[4]);

#  if SINGLE_DRIVER
	extern bool isSlowDriver;
	inline bool IsSlowDriver() { return isSlowDriver; }
#  else
	extern DriversBitmap slowDriversBitmap;
	inline DriversBitmap GetSlowDriversBitmap() { return slowDriversBitmap; }
	inline bool IsSlowDriver(size_t drive) { return slowDriversBitmap.IsBitSet(drive); }
#  endif
# endif
#endif	//SUPPORT_DRIVERS

	// Public functions
#if SUPPORT_SPI_SENSORS || SUPPORT_CLOSED_LOOP || defined(ATEIO)
	extern SharedSpiDevice *sharedSpi;
	inline SharedSpiDevice& GetSharedSpi() noexcept { return *sharedSpi; }
#endif

#if SUPPORT_I2C_SENSORS
	extern SharedI2CMaster *sharedI2C;
	inline SharedI2CMaster& GetSharedI2C() noexcept { return *sharedI2C; }
#endif

	// We don't really want the following to be writable, but we've no choice because we want to inline functions that access them
	extern bool isPrinting;
	extern uint32_t realTime;

	void Init();
	void InitMinimal();
	void Spin();
	void SpinMinimal();

	inline bool IsPrinting() { return isPrinting; }
	inline void SetPrinting(bool b) { isPrinting = b; }

	// Message output (see MessageType for further details)
	void LogError(ErrorCode e);
	bool Debug(Module module);
	void WriteLed(uint8_t ledNumber, bool turnOn);

#if USE_SERIAL_DEBUG
	bool DebugPutc(char c);
#endif

#if SUPPORT_DRIVERS
	float DriveStepsPerUnit(size_t drive);
	const float *GetDriveStepsPerUnit();
	void SetDriveStepsPerUnit(size_t drive, float val);

# if SINGLE_DRIVER
	inline void StepDriverLow()
	{
#  if DIFFERENTIAL_STEPPER_OUTPUTS || ACTIVE_HIGH_STEP
#   if RP2040
		sio_hw->gpio_clr = DriverBit;
#   else
		StepPio->OUTCLR.reg = DriverBit;
#   endif
#  else
		StepPio->OUTSET.reg = DriverBit;
#  endif
	}

	inline void StepDriverHigh()
	{
#  if DIFFERENTIAL_STEPPER_OUTPUTS || ACTIVE_HIGH_STEP
#   if RP2040
		sio_hw->gpio_set = DriverBit;
#   else
		StepPio->OUTSET.reg = DriverBit;
#   endif
#  else
		StepPio->OUTCLR.reg = DriverBit;
#  endif
	}

# else

	inline void StepDriversLow()
	{
#  if DIFFERENTIAL_STEPPER_OUTPUTS || ACTIVE_HIGH_STEP
#   if RP2040
		sio_hw->gpio_clr = allDriverBits;
#   else
		StepPio->OUTCLR.reg = allDriverBits;
#   endif
#  else
		StepPio->OUTSET.reg = allDriverBits;
#  endif
	}

	inline void StepDriversHigh(uint32_t driverMap)
	{
#  if DIFFERENTIAL_STEPPER_OUTPUTS || ACTIVE_HIGH_STEP
#   if RP2040
		sio_hw->gpio_set = driverMap;
#   else
		StepPio->OUTSET.reg = driverMap;
#   endif
#  else
		StepPio->OUTCLR.reg = driverMap;
#  endif
	}

	inline uint32_t GetDriversBitmap(size_t driver) { return driveDriverBits[driver]; } 		// Get the step bit for this driver
# endif

	inline unsigned int GetProhibitedExtruderMovements(unsigned int extrusions, unsigned int retractions) { return 0; }
# if SINGLE_DRIVER
	void SetDirection(bool direction);
# else
	void SetDirection(size_t driver, bool direction);
# endif
	void SetDirectionValue(size_t driver, bool dVal);
	bool GetDirectionValue(size_t driver) noexcept;
	bool GetDirectionValueNoCheck(size_t driver) noexcept;
	void SetEnableValue(size_t driver, int8_t eVal);
	int8_t GetEnableValue(size_t driver);
	void EnableDrive(size_t driver, uint16_t brakeOffDelay);
	void DisableDrive(size_t driver, uint16_t motorOffDelay);
	void DisableAllDrives();
	void SetDriverIdle(size_t driver, uint16_t idlePercent);
# if SUPPORT_CLOSED_LOOP
	bool EnableIfIdle(size_t driver);						// if the driver is idle, enable it; return true if driver enabled on return
# endif

	GCodeResult ProcessM569Point7(const CanMessageGeneric& msg, const StringRef& reply);

# if HAS_SMART_DRIVERS
	void SetMotorCurrent(size_t driver, float current);		//TODO avoid the int->float->int conversion
	float GetTmcDriversTemperature();
#  if HAS_STALL_DETECT
	void SetOrResetEventOnStall(DriversBitmap drivers, bool enable) noexcept;
	bool GetEventOnStall(unsigned int driver) noexcept;
#  endif
# else
	StandardDriverStatus GetStandardDriverStatus(size_t driver);
# endif

	// Signal that a new drivers fault has occurred and the main board needs to be told about it urgently
	inline void NewDriverFault() { Heat::NewDriverFault(); }

	// Function to send the status of our drivers - must be called only by the Heat task
	void SendDriversStatus(CanMessageBuffer& buf);
#endif	//SUPPORT_DRIVERS

#if SUPPORT_THERMISTORS
	int GetAveragingFilterIndex(const IoPort&);
	ThermistorAveragingFilter *GetAdcFilter(unsigned int filterNumber);

# if HAS_VREF_MONITOR
	ThermistorAveragingFilter *GetVssaFilter(unsigned int filterNumber);
	ThermistorAveragingFilter *GetVrefFilter(unsigned int filterNumber);
# endif
#endif

	const MinCurMax& GetMcuTemperatures();

	void KickHeatTaskWatchdog();
	uint32_t GetHeatTaskIdleTicks();

#if HAS_ADDRESS_SWITCHES
	uint8_t ReadBoardAddress();
#endif

	const UniqueIdBase& GetUniqueId() noexcept;

	void Tick() noexcept;

	void StartFirmwareUpdate();
	void StartBootloaderUpdate();
	void StartReset();

	void OnProcessingCanMessage();

	GCodeResult DoDiagnosticTest(const CanMessageDiagnosticTest& msg, const StringRef& reply);

	void EmergencyStop();

	[[noreturn]]inline void ResetProcessor()
	{
		SCB->AIRCR = (0x5FA << 16) | (1u << 2);						// reset the processor
		for (;;) { }
	}

#if HAS_VOLTAGE_MONITOR
	MinCurMax GetPowerVoltages(bool resetMinMax) noexcept;
	float GetCurrentVinVoltage() noexcept;
#endif

#if HAS_12V_MONITOR
	MinCurMax GetV12Voltages(bool resetMinMax) noexcept;
	float GetCurrentV12Voltage() noexcept;
#endif

	inline uint32_t GetDateTime() noexcept { return realTime; }
	inline void SetDateTime(uint32_t tim) noexcept { realTime = tim; }
	bool WasDeliberateError() noexcept;

#if SAME5x
	void SetInterruptPriority(IRQn base, unsigned int num, uint32_t prio) noexcept;
#endif

	void AppendBoardAndFirmwareDetails(const StringRef& reply) noexcept;
	void AppendDiagnostics(const StringRef& reply) noexcept;

#ifdef TOOL1LC
	uint8_t GetBoardVariant() noexcept;
#endif

	// Return true if there should be an accelerometer on board. Used by the test report.
	inline bool AlwaysHasAccelerometer() noexcept
	{
#if SUPPORT_LIS3DH && !ACCELEROMETER_USES_SPI
# ifdef TOOL1LC
		return GetBoardVariant() != 0;
# else
		return true;
# endif
#else
		return false;
#endif
	}

	// Return true if there is an inductive sensor chip on board. Used by the test report.
	inline bool AlwaysHasLDC1612() noexcept
	{
#if SUPPORT_LDC1612 && !defined(TOOL1LC)
		return true;
#else
		return false;
#endif
	}
}

#endif /* SRC_PLATFORM_H_ */
