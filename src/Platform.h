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
# ifdef SAMC21
// On the SAMC21 we have 2 additional filters for the SDADC inputs
constexpr size_t SdAdcTemp0FilterIndex = NumThermistorInputs + 2;
constexpr size_t SdAdcVrefFilterIndex = NumThermistorInputs + 3;
constexpr size_t NumThermistorFilters = NumThermistorInputs + 4;
# else
// SAME51
constexpr size_t NumThermistorFilters = NumThermistorInputs + 2;
# endif
#else
constexpr size_t NumThermistorFilters = NumThermistorInputs;
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
	// These would ideally be private
	extern DriversBitmap slowDriversBitmap;
	extern uint32_t slowDriverStepTimingClocks[4];

	void Init();
	void Spin();

	// Message output (see MessageType for further details)
	void Message(MessageType type, const char *message);
	//void Message(MessageType type, OutputBuffer *buffer);
	void MessageF(MessageType type, const char *fmt, ...) __attribute__ ((format (printf, 2, 3)));
	void MessageF(MessageType type, const char *fmt, va_list vargs);
	void LogError(ErrorCode e);
	bool Debug(Module module);

	float DriveStepsPerUnit(size_t drive);
	const float *GetDriveStepsPerUnit();
	float GetPressureAdvance(size_t driver);
	void SetPressureAdvance(size_t driver, float advance);
	void StepDriversLow();
	void StepDriversHigh(uint32_t driverMap);
	uint32_t GetDriversBitmap(size_t driver); 	// get the bitmap of driver step bits for this driver

	inline bool IsSlowDriver(size_t drive) { return slowDriversBitmap.IsBitSet(drive); }
	inline DriversBitmap GetSlowDriversBitmap() { return slowDriversBitmap; }
	inline uint32_t GetSlowDriverStepHighClocks() { return slowDriverStepTimingClocks[0]; }
	inline uint32_t GetSlowDriverStepLowClocks() { return slowDriverStepTimingClocks[1]; }
	inline uint32_t GetSlowDriverDirSetupClocks() { return slowDriverStepTimingClocks[2]; }
	inline uint32_t GetSlowDriverDirHoldClocks() { return slowDriverStepTimingClocks[3]; }
	void SetDriverStepTiming(size_t drive, const float timings[4]);

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
	ThermistorAveragingFilter *GetVssaFilter(unsigned int filterNumber);
	ThermistorAveragingFilter *GetVrefFilter(unsigned int filterNumber);

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

	GCodeResult DoDiagnosticTest(const CanMessageDiagnosticTest& msg, const StringRef& reply);

	[[noreturn]]void EmergencyStop();
	[[noreturn]]void SoftwareReset(uint16_t reason, const uint32_t *stk = nullptr);

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
}

#endif /* SRC_PLATFORM_H_ */
