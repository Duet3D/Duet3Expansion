/*
 * Platform.h
 *
 *  Created on: 9 Sep 2018
 *      Author: David
 */

#ifndef SRC_PLATFORM_H_
#define SRC_PLATFORM_H_

#include "RepRapFirmware.h"

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
enum class EndStopInputType
{
	activeLow = 0,
	activeHigh = 1,
	zProbe = 2,
	motorStall = 3
};

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
	float AxisMaximum(size_t axis) ;
//	void SetAxisMaximum(size_t axis, float value, bool byProbing);
	float AxisMinimum(size_t axis) ;
//	void SetAxisMinimum(size_t axis, float value, bool byProbing);
//	float AxisTotalLength(size_t axis) ;
	float GetPressureAdvance(size_t extruder) ;
//	void SetPressureAdvance(size_t extruder, float factor);
	float Acceleration(size_t axisOrExtruder) ;
	const float* Accelerations() ;
//	void SetAcceleration(size_t axisOrExtruder, float value);
	float MaxFeedrate(size_t axisOrExtruder) ;
	const float* MaxFeedrates() ;
//	void SetMaxFeedrate(size_t axisOrExtruder, float value);
	float GetInstantDv(size_t axis) ;
//	void SetInstantDv(size_t axis, float value);
	EndStopHit Stopped(size_t axisOrExtruder) ;
	bool EndStopInputState(size_t axis) ;
	void StepDriversLow();
	void StepDriversHigh(uint32_t driverMap);
//	uint32_t CalcDriverBitmap(size_t driver);
	uint32_t GetDriversBitmap(size_t axisOrExtruder); 	// get the bitmap of driver step bits for this axis or extruder

	inline uint32_t GetSlowDriversBitmap() { return slowDriversBitmap; }
	inline uint32_t GetSlowDriverStepHighClocks() { return slowDriverStepTimingClocks[0]; }
	inline uint32_t GetSlowDriverStepLowClocks() { return slowDriverStepTimingClocks[1]; }
	inline uint32_t GetSlowDriverDirSetupClocks() { return slowDriverStepTimingClocks[2]; }
	inline uint32_t GetSlowDriverDirHoldClocks() { return slowDriverStepTimingClocks[3]; }

	inline unsigned int GetProhibitedExtruderMovements(unsigned int extrusions, unsigned int retractions) { return 0; }
	void SetDirection(size_t axisOrExtruder, bool direction);
	EndStopHit GetZProbeResult() ;
	void EnableDrive(size_t axisOrExtruder);
	void DisableDrive(size_t axisOrExtruder);
	void DisableAllDrives();
	void SetDriversIdle();

	uint8_t ReadBoardAddress();
}

#endif /* SRC_PLATFORM_H_ */
