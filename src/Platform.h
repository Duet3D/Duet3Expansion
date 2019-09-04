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

// Define the number of temperature readings we average for each thermistor. This should be a power of 2 and at least 4 ^ AD_OVERSAMPLE_BITS.
// Keep THERMISTOR_AVERAGE_READINGS * NUM_HEATERS * 2ms no greater than HEAT_SAMPLE_TIME or the PIDs won't work well.
constexpr size_t ThermistorReadingsAveraged = 16;
constexpr size_t ZProbeReadingsAveraged = 8;		// We average this number of readings with IR on, and the same number with IR off
constexpr size_t McuTempReadingsAveraged = 16;
constexpr size_t VinReadingsAveraged = 8;

typedef AdcAveragingFilter<ThermistorReadingsAveraged> ThermistorAveragingFilter;
typedef AdcAveragingFilter<ZProbeReadingsAveraged> ZProbeAveragingFilter;

#if HAS_VREF_MONITOR
constexpr size_t NumThermistorFilters = NumThermistorInputs + 2;
constexpr size_t VssaFilterIndex = NumThermistorInputs;
constexpr size_t VrefFilterIndex = NumThermistorInputs + 1;
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

enum class EndStopHit
{
  noStop = 0,		// no endstop hit
  lowHit = 1,		// low switch hit, or Z-probe in use and above threshold
  highHit = 2,		// high stop hit
  nearStop = 3		// approaching Z-probe threshold
};

// The values of the following enumeration must tally with the X,Y,... parameters for the M574 command
enum class EndStopPosition
{
	noEndStop = 0,
	lowEndStop = 1,
	highEndStop = 2
};

// Type of an endstop input - values must tally with the M574 command S parameter
enum class EndStopInputType : unsigned int
{
	activeLow = 0,
	activeHigh = 1,
	zProbe = 2,
	motorStallAny = 3,
	motorStallIndividual = 4,
	numInputTypes = 5
};

class IoPort;

namespace Platform
{
	// These would ideally be private
	extern uint32_t slowDriversBitmap;
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

	void SetDriversIdle();
	float DriveStepsPerUnit(size_t drive);
	const float *GetDriveStepsPerUnit();
	float GetPressureAdvance(size_t extruder) ;
//	void SetPressureAdvance(size_t extruder, float factor);
	EndStopHit Stopped(size_t axisOrExtruder) ;
	bool EndStopInputState(size_t axis) ;
	void StepDriversLow();
	void StepDriversHigh(uint32_t driverMap);
	uint32_t GetDriversBitmap(size_t driver); 	// get the bitmap of driver step bits for this driver

	inline bool IsSlowDriver(size_t drive) { return IsBitSet(slowDriversBitmap, drive); }
	inline uint32_t GetSlowDriversBitmap() { return slowDriversBitmap; }
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
	void SetDriversIdle();

	EndStopHit GetZProbeResult() ;

	int GetAveragingFilterIndex(const IoPort&);
	ThermistorAveragingFilter& GetAdcFilter(unsigned int filterNumber);
	void GetMcuTemperatures(float& minTemp, float& currentTemp, float& maxTemp);
	float GetTmcDriversTemperature();

	void HandleHeaterFault(unsigned int heater);

	void KickHeatTaskWatchdog();

	uint8_t ReadBoardId();

	void Tick();

	void StartFirmwareUpdate();
}

#endif /* SRC_PLATFORM_H_ */
