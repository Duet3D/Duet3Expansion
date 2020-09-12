/*
 * Platform.h
 *
 *  Created on: 9 Sep 2018
 *      Author: David
 */

#ifndef SRC_PLATFORM_H_
#define SRC_PLATFORM_H_

#include "RepRapFirmware.h"
#include "AdcAveragingFilter.h"
#include "GCodes/GCodeResult.h"

#if SUPPORT_SPI_SENSORS
# include <Hardware/SharedSpiDevice.h>
#endif

class CanMessageDiagnosticTest;

// Define the number of temperature readings we average for each thermistor. This should be a power of 2 and at least 4 ^ AD_OVERSAMPLE_BITS.
constexpr size_t ThermistorReadingsAveraged = 64;
constexpr size_t ZProbeReadingsAveraged = 8;		// We average this number of readings with IR on, and the same number with IR off
constexpr size_t McuTempReadingsAveraged = 16;
constexpr size_t VinReadingsAveraged = 8;

typedef AdcAveragingFilter<ThermistorReadingsAveraged> ThermistorAveragingFilter;
typedef AdcAveragingFilter<ZProbeReadingsAveraged> ZProbeAveragingFilter;

#if HAS_VREF_MONITOR
constexpr size_t VssaFilterIndex = NumThermistorInputs;
constexpr size_t VrefFilterIndex = NumThermistorInputs + 1;
# if SAMC21 && SUPPORT_SDADC
// On the SAMC21 we have 2 additional filters for the SDADC inputs
constexpr size_t SdAdcTemp0FilterIndex = NumThermistorInputs + 2;
constexpr size_t SdAdcVrefFilterIndex = NumThermistorInputs + 3;
constexpr size_t NumThermistorFilters = NumThermistorInputs + 4;
# else
// SAME51 or not supporting SDADC
constexpr size_t NumThermistorFilters = NumThermistorInputs + 2;
# endif
#else
# if SAMC21 && SUPPORT_SDADC
// On the SAMC21 we have 2 additional filters for the SDADC inputs
constexpr size_t SdAdcTemp0FilterIndex = NumThermistorInputs;
constexpr size_t SdAdcVrefFilterIndex = NumThermistorInputs + 1;
constexpr size_t NumThermistorFilters = NumThermistorInputs + 2;
# else
constexpr size_t NumThermistorFilters = NumThermistorInputs;
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
	// The data would ideally be private, but they are used by inline functions so they can't be unless we convert this namespace to a static class
#if SINGLE_DRIVER
	constexpr uint32_t DriverBit = 1u << (StepPins[0] & 31);
#else
	extern uint32_t driveDriverBits[NumDrivers];
	extern uint32_t allDriverBits;
#endif

#if SUPPORT_SLOW_DRIVERS
	// Support for slow step pulse generation to suit external drivers
	extern uint32_t slowDriverStepTimingClocks[4];

	inline uint32_t GetSlowDriverStepHighClocks() { return slowDriverStepTimingClocks[0]; }
	inline uint32_t GetSlowDriverStepLowClocks() { return slowDriverStepTimingClocks[1]; }
	inline uint32_t GetSlowDriverDirSetupClocks() { return slowDriverStepTimingClocks[2]; }
	inline uint32_t GetSlowDriverDirHoldClocks() { return slowDriverStepTimingClocks[3]; }

	void SetDriverStepTiming(size_t drive, const float timings[4]);

# if SINGLE_DRIVER
	extern bool isSlowDriver;
	inline bool IsSlowDriver() { return isSlowDriver; }
# else
	extern DriversBitmap slowDriversBitmap;
	inline DriversBitmap GetSlowDriversBitmap() { return slowDriversBitmap; }
	inline bool IsSlowDriver(size_t drive) { return slowDriversBitmap.IsBitSet(drive); }
# endif
#endif

	// Public functions
#if SUPPORT_SPI_SENSORS
	extern SharedSpiDevice *sharedSpi;
	inline SharedSpiDevice& GetSharedSpi() noexcept { return *sharedSpi; }
#endif

	void Init();
	void Spin();

	// Message output (see MessageType for further details)
	void Message(MessageType type, const char *message);
	void MessageF(MessageType type, const char *fmt, ...) __attribute__ ((format (printf, 2, 3)));
	void MessageF(MessageType type, const char *fmt, va_list vargs);
	void LogError(ErrorCode e);
	bool Debug(Module module);

	float DriveStepsPerUnit(size_t drive);
	const float *GetDriveStepsPerUnit();
	float GetPressureAdvance(size_t driver);
	void SetPressureAdvance(size_t driver, float advance);

#if SINGLE_DRIVER
	inline void StepDriverLow()
	{
#if DIFFERENTIAL_STEPPER_OUTPUTS || ACTIVE_HIGH_STEP
		StepPio->OUTCLR.reg = DriverBit;
#else
		StepPio->OUTSET.reg = DriverBit;
#endif
	}

	inline void StepDriverHigh()
	{
#if DIFFERENTIAL_STEPPER_OUTPUTS || ACTIVE_HIGH_STEP
		StepPio->OUTSET.reg = DriverBit;
#else
		StepPio->OUTCLR.reg = DriverBit;
#endif
	}

#else

	inline void StepDriversLow()
	{
#if DIFFERENTIAL_STEPPER_OUTPUTS || ACTIVE_HIGH_STEP
		StepPio->OUTCLR.reg = allDriverBits;
#else
		StepPio->OUTSET.reg = allDriverBits;
#endif
	}

	inline void StepDriversHigh(uint32_t driverMap)
	{
#if DIFFERENTIAL_STEPPER_OUTPUTS || ACTIVE_HIGH_STEP
		StepPio->OUTSET.reg = driverMap;
#else
		StepPio->OUTCLR.reg = driverMap;
#endif
	}

	inline uint32_t GetDriversBitmap(size_t driver) { return driveDriverBits[driver]; } 		// Get the step bit for this driver
#endif

	inline unsigned int GetProhibitedExtruderMovements(unsigned int extrusions, unsigned int retractions) { return 0; }
	void SetDirection(size_t driver, bool direction);
	void SetDirectionValue(size_t driver, bool dVal);
	bool GetDirectionValue(size_t driver);
	void SetEnableValue(size_t driver, int8_t eVal);
	int8_t GetEnableValue(size_t driver);
	void EnableDrive(size_t driver);
	void DisableDrive(size_t driver);
	void DisableAllDrives();
	void SetDriverIdle(size_t driver);

#if HAS_SMART_DRIVERS
	void SetMotorCurrent(size_t driver, float current);		//TODO avoid the int->float->int conversion
	float GetTmcDriversTemperature();
#endif

	int GetAveragingFilterIndex(const IoPort&);
	ThermistorAveragingFilter *GetAdcFilter(unsigned int filterNumber);

#if HAS_VREF_MONITOR
	ThermistorAveragingFilter *GetVssaFilter(unsigned int filterNumber);
	ThermistorAveragingFilter *GetVrefFilter(unsigned int filterNumber);
#endif

	void GetMcuTemperatures(float& minTemp, float& currentTemp, float& maxTemp);

	void HandleHeaterFault(unsigned int heater);

	void KickHeatTaskWatchdog();
	uint32_t GetHeatTaskIdleTicks();

#if HAS_ADDRESS_SWITCHES
	uint8_t ReadBoardAddress();
#endif

	void AppendUniqueId(const StringRef& str);

	void Tick();

	void StartFirmwareUpdate();
	void StartReset();

	void OnProcessingCanMessage();

	GCodeResult DoDiagnosticTest(const CanMessageDiagnosticTest& msg, const StringRef& reply);

	[[noreturn]]void EmergencyStop();

	[[noreturn]]inline void ResetProcessor()
	{
		SCB->AIRCR = (0x5FA << 16) | (1u << 2);						// reset the processor
		for (;;) { }
	}

#if HAS_VOLTAGE_MONITOR
	float GetMinVinVoltage();
	float GetCurrentVinVoltage();
	float GetMaxVinVoltage();
#endif

#if HAS_12V_MONITOR
	float GetMinV12Voltage();
	float GetCurrentV12Voltage();
	float GetMaxV12Voltage();
#endif

	uint32_t GetDateTime() noexcept;
	void SetDateTime(uint32_t tim) noexcept;
	bool WasDeliberateError() noexcept;
}

#endif /* SRC_PLATFORM_H_ */
