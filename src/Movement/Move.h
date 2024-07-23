/*
 * Move.h
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#ifndef MOVE_H_
#define MOVE_H_

#include <RepRapFirmware.h>

#if SUPPORT_DRIVERS

#ifndef SUPPORT_INPUT_SHAPING
# error SUPPORT_INPUT_SHAPING not defined
#endif

#include "DriveMovement.h"
#include "ExtruderShaper.h"
#include <CanMessageFormats.h>
#include <CanMessageBuffer.h>
#include <Hardware/IoPorts.h>

#if SUPPORT_INPUT_SHAPING
# include "AxisShaper.h"
#endif

#if SUPPORT_CLOSED_LOOP
# include "StepperDrivers/TMC51xx.h"				// for SmartDrivers::GetMicrostepShift
#endif

struct CanMessageStopMovement;
struct CanMessageSetInputShapingNew;
struct CanMessageMovementLinearShaped;

// Struct for passing parameters to the DriveMovement Prepare methods
struct PrepParams
{
	// Parameters used for all types of motion
	static constexpr float totalDistance = 1.0;
	float accelDistance;
	float decelStartDistance;
	uint32_t accelClocks, steadyClocks, decelClocks;
	float acceleration, deceleration;				// the acceleration and deceleration to use, both positive
	float topSpeed;
	bool useInputShaping;

	// Get the total clocks needed
	uint32_t TotalClocks() const noexcept { return accelClocks + steadyClocks + decelClocks; }

	void DebugPrint() const noexcept;
};

/**
 * This is the master movement class.  It controls all movement in the machine.
 */
class Move
{
public:
	Move() noexcept;
	void Init() noexcept;															// Start me up
	void Exit() noexcept;															// Shut down
#if HAS_VOLTAGE_MONITOR || HAS_12V_MONITOR
	void Spin(bool powered) noexcept;
#else
	void Spin() noexcept;
#endif

	StandardDriverStatus GetDriverStatus(size_t driver, bool accumulated, bool clearAccumulated) const noexcept;
	void Diagnostics(const StringRef& reply) noexcept;								// Report useful stuff

	void SetDirectionValue(size_t driver, bool dVal);
	bool GetDirectionValue(size_t driver) const noexcept;
#if SUPPORT_CLOSED_LOOP
	bool GetDirectionValueNoCheck(size_t driver) const noexcept { return directions[driver]; }
#endif

	void SetEnableValue(size_t driver, int8_t eVal) noexcept;
	int8_t GetEnableValue(size_t driver) const noexcept;
	void EnableDrive(size_t driver) noexcept;
	void DisableDrive(size_t driver) noexcept;
	void DisableAllDrives() noexcept;
	void SetDriverIdle(size_t driver, uint16_t idlePercent) noexcept;

	GCodeResult ProcessM569(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	GCodeResult ProcessM569Point7(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	GCodeResult SetMotorCurrents(const CanMessageMultipleDrivesRequest<float>& msg, size_t dataLength, const StringRef& reply) noexcept;
	GCodeResult SetStandstillCurrentFactor(const CanMessageMultipleDrivesRequest<float>& msg, size_t dataLength, const StringRef& reply) noexcept;

# if HAS_SMART_DRIVERS
	void SetMotorCurrent(size_t driver, float current) noexcept;		//TODO avoid the int->float->int conversion
	float GetTmcDriversTemperature() noexcept;
#  if HAS_STALL_DETECT
	void SetOrResetEventOnStall(DriversBitmap drivers, bool enable) noexcept;
	bool GetEventOnStall(unsigned int driver) noexcept;
#  endif
# else
	StandardDriverStatus GetStandardDriverStatus(size_t driver);
# endif

	// Function to send the status of our drivers - must be called only by the Heat task
	void SendDriversStatus(CanMessageBuffer& buf) noexcept;

	float DriveStepsPerUnit(size_t drive) const noexcept;
	void SetDriveStepsPerUnit(size_t drive, float val) noexcept;

#if SUPPORT_SLOW_DRIVERS
	void SetDriverStepTiming(size_t drive, const float timings[4]) noexcept;
# if SINGLE_DRIVER
	bool IsSlowDriver() const noexcept { return isSlowDriver; }
# else
	bool IsSlowDriver(size_t drive) const noexcept { return slowDriversBitmap.IsBitSet(drive); }
# endif

	float GetSlowDriverStepHighMicroseconds() const noexcept;
	float GetSlowDriverStepLowMicroseconds() const noexcept;
	float GetSlowDriverDirSetupMicroseconds() const noexcept;
	float GetSlowDriverDirHoldMicroseconds() const noexcept;
#endif

	void Interrupt() noexcept SPEED_CRITICAL;										// Timer callback for step generation
	void StopDrivers(uint16_t whichDrives) noexcept;

#if !DEDICATED_STEP_TIMER
	static void TimerCallback(CallbackParameter cb) noexcept
	{
		static_cast<Move*>(cb.vp)->Interrupt();
	}
#endif

	void ResetMoveCounters() noexcept { scheduledMoves = 0; }
	void UpdateExtrusionPendingLimits(float extrusionPending) noexcept;

	int32_t GetPosition(size_t driver) const noexcept;

	// Filament monitor support
	int32_t GetAccumulatedExtrusion(size_t driver, bool& isPrinting) noexcept;		// Return and reset the accumulated commanded extrusion amount
	uint32_t ExtruderPrintingSince(size_t driver) const noexcept;					// When we started doing normal moves after the most recent extruder-only move

	// Input shaping support
	GCodeResult HandleInputShaping(const CanMessageSetInputShapingNew& msg, size_t dataLength, const StringRef& reply) noexcept;
	void AddLinearSegments(size_t logicalDrive, uint32_t startTime, const PrepParams& params, motioncalc_t steps, MovementFlags moveFlags) noexcept;

	// Pressure advance
	ExtruderShaper& GetExtruderShaper(size_t drive) noexcept { return dms[drive].extruderShaper; }

#if HAS_SMART_DRIVERS
	uint32_t GetStepInterval(size_t axis, uint32_t microstepShift) const noexcept;	// Get the current step interval for this axis or extruder
	bool SetMicrostepping(size_t driver, unsigned int microsteps, bool interpolate) noexcept;
#endif

	// Movement error handling
	void LogStepError(uint8_t type) noexcept;										// stop all movement because of a step error

	int32_t GetLastMoveStepsTaken(size_t drive) const noexcept { return lastMoveStepsTaken[drive]; }

	[[noreturn]] void TaskLoop() noexcept;

#if SUPPORT_CLOSED_LOOP
	GCodeResult ProcessM569Point1(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	GCodeResult ProcessM569Point4(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	GCodeResult ProcessM569Point5(const CanMessageStartClosedLoopDataCollection&, const StringRef& reply) noexcept;
	GCodeResult ProcessM569Point6(const CanMessageGeneric& msg, const StringRef& reply) noexcept;

	bool IsClosedLoopEnabled(size_t driver) const noexcept { return dms[driver].closedLoopControl.IsClosedLoopEnabled(); }
	bool EnableIfIdle(size_t driver) noexcept;										// if the driver is idle, enable it; return true if driver enabled on return
	bool GetCurrentMotion(size_t driver, uint32_t when, MotionParameters& mParams) noexcept;	// get the net full steps taken, including in the current move so far, also speed and acceleration; return true if moving
	void SetCurrentMotorSteps(size_t driver, float fullSteps) noexcept;
	void InvertCurrentMotorSteps(size_t driver) noexcept;

	void ClosedLoopControlLoop() noexcept;
	void ClosedLoopDiagnostics(size_t driver, const StringRef& reply) noexcept;
#endif

private:
	static constexpr float DefaultStepsPerMm = 80.0;

	enum class StepErrorState : uint8_t
	{
		noError = 0,	// no error
		haveError,		// had an error, movement is stopped
		resetting		// had an error, ready to reset it
	};

	bool AddMove(const CanMessageMovementLinearShaped& msg) noexcept;				// Add a new move received via CAN
	void StepDrivers(uint32_t now) noexcept SPEED_CRITICAL;							// Take one step of the DDA, called by timer interrupt.
	void PrepareForNextSteps(DriveMovement *stopDm, uint32_t now) noexcept SPEED_CRITICAL;
	bool ScheduleNextStepInterrupt() noexcept SPEED_CRITICAL;						// Schedule the next interrupt, returning true if we can't because it is already due
	bool StopAxisOrExtruder(bool executingMove, size_t logicalDrive) noexcept;		// stop movement of a drive and recalculate the endpoint

	void StepDriversLow() noexcept;
	void StepDriversHigh(uint32_t driverMap) noexcept;

	void InternalDisableDrive(size_t driver) noexcept;
	void DisengageBrake(size_t driver) noexcept;									// Turn the brake solenoid on to disengage the brake
	void EngageBrake(size_t driver) noexcept;										// Turn the brake solenoid off to re-engage the brake

	void UpdateMotorCurrent(size_t driver) noexcept;

	// Convert microseconds to step clocks, rounding up
	static uint32_t MicrosecondsToStepClocks(float us) noexcept
	{
		return (uint32_t)(((float)StepTimer::StepClockRate * us * 0.000001) + 0.99);
	}

	float stepsPerMm[NumDrivers];
	bool directions[NumDrivers];
	uint32_t scheduledMoves = 0;													// Move counters for the code queue

	bool driverAtIdleCurrent[NumDrivers];
	int8_t enableValues[NumDrivers];

#if SUPPORT_BRAKE_PWM
# if defined(M23CL)
	static constexpr float M23CLBrakeVoltage = 24.0;
# else
	PwmPort brakePwmPorts[NumDrivers];
	Pin brakeOnPins[NumDrivers];
	float brakeVoltages[NumDrivers];
	static constexpr float FullyOnBrakeVoltage = 100.0;			// this value means always use full voltage (don't PWM)
# endif
	float currentBrakePwm[NumDrivers];
#else
	IoPort brakePorts[NumDrivers];
#endif

	MillisTimer brakeOffTimers[NumDrivers];
	MillisTimer motorOffTimers[NumDrivers];
	uint16_t brakeOffDelays[NumDrivers];
	uint16_t motorOffDelays[NumDrivers];

	uint8_t driverStates[NumDrivers];
	float motorCurrents[NumDrivers];
	float idleCurrentFactor[NumDrivers];

#if HAS_SMART_DRIVERS
	DriversBitmap temperatureShutdownDrivers, temperatureWarningDrivers;
	uint8_t nextDriveToPoll = 0;
	StandardDriverStatus lastEventStatus[NumDrivers];			// the status which we last reported as an event
	MillisTimer openLoadTimers[NumDrivers];
#else
	bool driverIsEnabled[NumDrivers];
#endif

#if HAS_SMART_DRIVERS && HAS_VOLTAGE_MONITOR
	bool warnDriversNotPowered = false;
#endif

#if HAS_STALL_DETECT
	DriversBitmap eventOnStallDrivers;
#endif

#if SUPPORT_CLOSED_LOOP
	float netMicrostepsTaken[NumDrivers];											// the net microsteps taken not counting any move that is in progress
#endif

	TaskBase * volatile taskWaitingForMoveToComplete = nullptr;

	DriveMovement dms[NumDrivers];

#if SINGLE_DRIVER
	void SetDirection(bool direction) noexcept;										// set the direction of a driver, observing timing requirements

	static constexpr uint32_t allDriverBits = 1u << (StepPins[0] & 31);
#else
	void SetDirection(size_t axisOrExtruder, bool direction) noexcept;				// set the direction of a driver, observing timing requirements
	void InsertDM(DriveMovement *dm) noexcept;										// insert a DM into the active list, keeping it in step time order
	void DeactivateDM(DriveMovement *dmToRemove) noexcept;							// remove a DM from the active list

	DriveMovement *activeDMs = nullptr;
	uint32_t allDriverBits = 0;
#endif

#if SUPPORT_SLOW_DRIVERS
	// Support for slow step pulse generation to suit external drivers
# ifdef EXP1XD
#  if USE_TC_FOR_STEP
	// The first element of slowDriverStepTimingClocks is the pulse width in TC clocks. The other elements are in step clock units. The second element is the step high + step low time
	static constexpr unsigned int TcPrescaler = 4;													// use prescaler 4
	static constexpr uint32_t TcPrescalerRegVal = TC_CTRLA_PRESCALER_DIV4;							// use prescaler 4
	static constexpr float StepPulseClocksPerMicrosecond = 48.0/TcPrescaler;						// we clock the TC from the 48MHz clock
	static constexpr unsigned int StepClocksToStepTCClocks = 64/TcPrescaler;						// step clock uses the same 48MHz clock and x64 prescaler
	static constexpr float MinimumStepHighMicroseconds = 0.2;

	// Convert microseconds to StepTC step clocks, rounding up
	static constexpr uint32_t MicrosecondsToStepTCClocks(float us) noexcept
	{
		return (uint32_t)((StepPulseClocksPerMicrosecond * us) + 0.99);
	}

	uint32_t slowDriverStepTimingClocks[4] = { 2 * StepClocksToStepTCClocks, 4, 2, 4 };				// default to slower timing for external drivers (2 clocks = 2.67us)

#  else
	// All elements are in step clock units. The second element is the step low time.
	uint32_t slowDriverStepTimingClocks[4] = { 2, 2, 2, 2 };										// default to slower timing for external drivers (2 clocks = 2.67us)
#  endif
# else
	uint32_t slowDriverStepTimingClocks[4] = { 0, 0, 0, 0 };										// default to fast timing
# endif
# if SINGLE_DRIVER
#  ifdef EXP1XD
	bool isSlowDriver = true;
#  else
	bool isSlowDriver = false;
#  endif
# else
	DriversBitmap slowDriversBitmap;
# endif

# if USE_TC_FOR_STEP		// the first element has a special meaning when we use a TC to generate the steps
	inline uint32_t GetSlowDriverStepPeriodClocks() const noexcept { return slowDriverStepTimingClocks[1]; }
	inline uint32_t GetSlowDriverDirHoldFromLeadingEdgeClocks() const noexcept { return slowDriverStepTimingClocks[3]; }
# else
	inline uint32_t GetSlowDriverStepHighClocks() const noexcept { return slowDriverStepTimingClocks[0]; }
	inline uint32_t GetSlowDriverStepLowClocks() const noexcept { return slowDriverStepTimingClocks[1]; }
	inline uint32_t GetSlowDriverDirHoldFromTrailingEdgeClocks() const noexcept { return slowDriverStepTimingClocks[3]; }
# endif

	inline uint32_t GetSlowDriverDirSetupClocks() { return slowDriverStepTimingClocks[2]; }

# if USE_TC_FOR_STEP
	volatile uint32_t lastStepHighTime = 0;							// when we last started a step pulse
# else
	volatile uint32_t lastStepLowTime = 0;							// when we last completed a step pulse to a slow driver
# endif
	volatile uint32_t lastDirChangeTime = 0;						// when we last changed the DIR signal to a slow driver

#endif	// SUPPORT_SLOW_DRIVERS

#if !DEDICATED_STEP_TIMER
	StepTimer timer;
#endif

	int32_t lastMoveStepsTaken[NumDrivers];							// how many steps were taken in the last move we did, used for reverting
	unsigned int numHiccups = 0;									// The number of hiccups inserted

#if SUPPORT_INPUT_SHAPING
	AxisShaper axisShaper;
#endif

	volatile uint8_t stepErrorType;
	volatile StepErrorState stepErrorState = StepErrorState::noError;

	uint32_t maxPrepareTime = 0;
	float minExtrusionPending = 0.0, maxExtrusionPending = 0.0;

	unsigned int numStepErrors = 0;
	Bitmap<uint16_t> stepErrorTypesLogged;
};

//******************************************************************************************************

// Update the min and max extrusion pending values. These are reported by M122 to assist with debugging print quality issues.
// Inlined because this is only called from one place.
inline void Move::UpdateExtrusionPendingLimits(float extrusionPending) noexcept
{
	if (extrusionPending > maxExtrusionPending) { maxExtrusionPending = extrusionPending; }
	else if (extrusionPending < minExtrusionPending) { minExtrusionPending = extrusionPending; }
}

inline int32_t Move::GetPosition(size_t driver) const noexcept
{
	return dms[driver].currentMotorPosition;
}

// Handle a CAN request to set the input shaping parameters
inline GCodeResult Move::HandleInputShaping(const CanMessageSetInputShapingNew& msg, size_t dataLength, const StringRef& reply) noexcept
{
#if SUPPORT_INPUT_SHAPING
	return axisShaper.EutSetInputShaping(msg, dataLength, reply);
#else
	return GCodeResult::ok;						// ignore it but return success
#endif
}

// Schedule the next interrupt, returning true if we can't because it is already due
// Base priority must be >= NvicPriorityStep when calling this
inline __attribute__((always_inline)) bool Move::ScheduleNextStepInterrupt() noexcept
{
#if SINGLE_DRIVER
	if (
# if SUPPORT_CLOSED_LOOP
		!dms[0].closedLoopControl.IsClosedLoopEnabled() &&
# endif
		dms[0].state >= DMState::firstMotionState
	   )
	{
# if DEDICATED_STEP_TIMER
		return StepTimer::ScheduleMovementCallbackFromIsr(dms[0].nextStepTime);
# else
		return timer.ScheduleMovementCallbackFromIsr(dms[0].nextStepTime);
# endif
	}
#else
	if (activeDMs != nullptr)
	{
# if DEDICATED_STEP_TIMER
		return StepTimer::ScheduleMovementCallbackFromIsr(activeDMs->nextStepTime);
# else
		return timer.ScheduleMovementCallbackFromIsr(activeDMs->nextStepTime);
# endif
	}
#endif
	return false;
}

inline void Move::StepDriversLow() noexcept
{
# if DIFFERENTIAL_STEPPER_OUTPUTS || ACTIVE_HIGH_STEP
#  if RP2040
	sio_hw->gpio_clr = allDriverBits;
#  else
	StepPio->OUTCLR.reg = allDriverBits;
#  endif
# else
	StepPio->OUTSET.reg = allDriverBits;
# endif
}

inline void Move::StepDriversHigh(uint32_t driverMap) noexcept
{
# if DIFFERENTIAL_STEPPER_OUTPUTS || ACTIVE_HIGH_STEP
#  if RP2040
	sio_hw->gpio_set = driverMap;
#  else
	StepPio->OUTSET.reg = driverMap;
#  endif
# else
	StepPio->OUTCLR.reg = driverMap;
# endif
}

#if SINGLE_DRIVER

inline void Move::SetDirection(bool direction) noexcept
{
# if DIFFERENTIAL_STEPPER_OUTPUTS || ACTIVE_HIGH_DIR
	// Active high direction signal
	const bool d = (direction) ? directions[0] : !directions[0];
# else
	// Active low direction signal
	const bool d = (direction) ? !directions[0] : directions[0];
# endif

# if SUPPORT_CLOSED_LOOP
	if (dms[0].closedLoopControl.IsClosedLoopEnabled())
	{
		return;
	}
# endif
# if SUPPORT_SLOW_DRIVERS
	if (isSlowDriver)
	{
#  if USE_TC_FOR_STEP
		while (StepTimer::GetTimerTicks() - lastStepHighTime < GetSlowDriverDirHoldFromLeadingEdgeClocks()) { }
#  else
		while (StepTimer::GetTimerTicks() - lastStepLowTime < GetSlowDriverDirHoldFromTrailingEdgeClocks()) { }
#  endif
	}
# endif
	digitalWrite(DirectionPins[0], d);
# if DIFFERENTIAL_STEPPER_OUTPUTS
	digitalWrite(InvertedDirectionPins[0], !d);
# endif
# if SUPPORT_SLOW_DRIVERS
	if (isSlowDriver)
	{
		lastDirChangeTime = StepTimer::GetTimerTicks();
	}
# endif
}

#else

inline void Move::SetDirection(size_t driver, bool direction) noexcept
{
	if (driver < NumDrivers)
	{
# if DIFFERENTIAL_STEPPER_OUTPUTS || ACTIVE_HIGH_DIR
		// Active high direction signal
		const bool d = (direction) ? directions[driver] : !directions[driver];
# else
		// Active low direction signal
		const bool d = (direction) ? !directions[driver] : directions[driver];
# endif

# if SUPPORT_SLOW_DRIVERS
		const bool isSlowDriver = slowDriversBitmap.IsBitSet(driver);
		if (isSlowDriver)
		{
			while (StepTimer::GetTimerTicks() - DDA::lastStepLowTime < GetSlowDriverDirHoldClocks()) { }
		}
# endif
		digitalWrite(DirectionPins[driver], d);
# if DIFFERENTIAL_STEPPER_OUTPUTS
		digitalWrite(InvertedDirectionPins[driver], !d);
# endif
# if SUPPORT_SLOW_DRIVERS
		if (isSlowDriver)
		{
			lastDirChangeTime = StepTimer::GetTimerTicks();
		}
# endif
	}
}

// Insert the specified drive into the step list, in step time order.
// We insert the drive before any existing entries with the same step time for best performance.
// Now that we generate step pulses for multiple motors simultaneously, there is no need to preserve round-robin order.
// Base priority must be >= NvicPriorityStep when calling this, unless we are simulating.
inline void Move::InsertDM(DriveMovement *dm) noexcept
{
	DriveMovement **dmp = &activeDMs;
	while (*dmp != nullptr && (int32_t)((*dmp)->nextStepTime - dm->nextStepTime) < 0)
	{
		dmp = &((*dmp)->nextDM);
	}
	dm->nextDM = *dmp;
	*dmp = dm;
}

#endif	// SINGLE_DRIVER

#if HAS_SMART_DRIVERS

// Get the current step interval for this axis or extruder, or 0 if it is not moving
// This is called from the stepper drivers SPI interface ISR
inline __attribute__((always_inline)) uint32_t Move::GetStepInterval(size_t drive, uint32_t microstepShift) const noexcept
{
	AtomicCriticalSectionLocker lock;
	return dms[drive].GetStepInterval(microstepShift);
}

#endif

#if SUPPORT_CLOSED_LOOP

// Get the motor position in the current move so far, also speed and acceleration. Units are full steps and step clocks.
// Inlined because it is only called from one place
inline bool Move::GetCurrentMotion(size_t driver, uint32_t when, MotionParameters& mParams) noexcept
{
	const float multiplier = ldexpf((GetDirectionValueNoCheck(driver)) ? -1.0 : 1.0, -(int)SmartDrivers::GetMicrostepShift(driver));
	if (dms[driver].GetCurrentMotion(when, mParams))
	{
		// Convert microsteps to full steps
		mParams.position = (mParams.position + netMicrostepsTaken[driver]) * multiplier;
		mParams.speed *= multiplier;
		mParams.acceleration *= multiplier;
		return true;
	}

	// Here when there is no current move
	mParams.position = netMicrostepsTaken[driver] * multiplier;
	mParams.speed = mParams.acceleration = 0.0;
	return false;
}

inline void Move::SetCurrentMotorSteps(size_t driver, float fullSteps) noexcept
{
	const float multiplier = ldexpf((GetDirectionValueNoCheck(driver)) ? -1.0 : 1.0, (int)SmartDrivers::GetMicrostepShift(driver));
	netMicrostepsTaken[driver] = fullSteps * multiplier;
}

// Invert the current number of microsteps taken. Called when the driver direction control is changed.
inline void Move::InvertCurrentMotorSteps(size_t driver) noexcept
{
	netMicrostepsTaken[driver] = -netMicrostepsTaken[driver];
}

#endif	// SUPPORT_CLOSED_LOOP

#endif	// SUPPORT_DRIVERS

#endif /* MOVE_H_ */
