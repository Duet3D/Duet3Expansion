/*
 * DDA.h
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#ifndef DDA_H_
#define DDA_H_

#include <RepRapFirmware.h>
#include <Platform/Tasks.h>

#if SUPPORT_DRIVERS

#include "DriveMovement.h"
#include "StepTimer.h"

struct CanMessageMovementLinear;
struct CanMessageStopMovement;

// This defines a single coordinated movement of one or several motors
class DDA
{
	friend class DriveMovement;

public:

	enum DDAState : uint8_t
	{
		empty,				// empty or being filled in
		provisional,		// ready, but could be subject to modifications
		frozen,				// ready, no further modifications allowed
		executing,			// steps are currently being generated for this DDA
		completed			// move has been completed or aborted
	};

	DDA(DDA* n) noexcept;

	void* operator new(size_t count) { return Tasks::AllocPermanent(count); }
	void* operator new(size_t count, std::align_val_t align) { return Tasks::AllocPermanent(count, align); }
	void operator delete(void* ptr) noexcept {}
	void operator delete(void* ptr, std::align_val_t align) noexcept {}

	void Init() noexcept;														// Set up initial positions for machine startup
	bool Init(const CanMessageMovementLinear& msg) noexcept SPEED_CRITICAL;		// Set up a move from a CAN message
	void Start(uint32_t tim) noexcept SPEED_CRITICAL;							// Start executing the DDA, i.e. move the move.
	void StepDrivers(uint32_t now) noexcept SPEED_CRITICAL;						// Take one step of the DDA, called by timed interrupt.
	bool ScheduleNextStepInterrupt(StepTimer& timer) const noexcept SPEED_CRITICAL;		// Schedule the next interrupt, returning true if we can't because it is already due

	void SetNext(DDA *n) noexcept { next = n; }
	void SetPrevious(DDA *p) noexcept { prev = p; }
	void Complete() noexcept { state = completed; }
	void Free() noexcept;
	bool HasStepError() const noexcept;
	bool IsPrintingMove() const noexcept { return flags.isPrintingMove; }

	DDAState GetState() const noexcept { return state; }
	DDA* GetNext() const noexcept { return next; }
	DDA* GetPrevious() const noexcept { return prev; }
	int32_t GetTimeLeft() const noexcept;
	void InsertHiccup(uint32_t now) noexcept;

	// Filament monitor support
	int32_t GetStepsTaken(size_t drive) const noexcept;

	void StopDrivers(uint16_t whichDrives) noexcept;

	uint32_t GetClocksNeeded() const noexcept { return clocksNeeded; }
	uint32_t GetMoveFinishTime() const noexcept { return afterPrepare.moveStartTime + clocksNeeded; }

	int32_t GetPosition(size_t driver) const noexcept { return endPoint[driver]; }

#if HAS_SMART_DRIVERS
	uint32_t GetStepInterval(size_t axis, uint32_t microstepShift) const noexcept;	// Get the current full step interval for this axis or extruder
#endif

#if SUPPORT_CLOSED_LOOP
	void GetCurrentMotion(MotionParameters& mParams, int32_t netMicrostepsTaken, int microstepShift) const noexcept;
#endif

	void DebugPrint() const noexcept;												// print the DDA only
	void DebugPrintAll() const noexcept;												// print the DDA and active DMs

	static unsigned int GetAndClearStepErrors() noexcept;
	static uint32_t GetAndClearMaxTicksOverdue() noexcept;
	static uint32_t GetAndClearMaxOverdueIncrement() noexcept;

	static void RecordStepError() noexcept { ++stepErrors; }

	// Note on the following constant:
	// If we calculate the step interval on every clock, we reach a point where the calculation time exceeds the step interval.
	// The worst case is pure Z movement on a delta. On a Mini Kossel with 80 steps/mm with this firmware running on a Duet (84MHx SAM3X8 processor),
	// the calculation can just be managed in time at speeds of 15000mm/min (step interval 50us), but not at 20000mm/min (step interval 37.5us).
	// Therefore, where the step interval falls below 60us, we don't calculate on every step.
	// Note: the above measurements were taken some time ago, before some firmware optimisations.
	// The system clock of the SAME70 is running at 150MHz. Use the same defaults as for the SAM4E for now.
#if SAMC21
	static constexpr uint32_t MinCalcIntervalDelta = (100 * StepTimer::StepClockRate)/1000000; 		// the smallest sensible interval between calculations (40us) in step timer clocks
	static constexpr uint32_t MinCalcIntervalCartesian = (100 * StepTimer::StepClockRate)/1000000;	// same as delta for now, but could be lower
	static constexpr uint32_t HiccupTime = (50 * StepTimer::StepClockRate)/1000000;					// how long we hiccup for
	static constexpr uint32_t MaxStepInterruptTime = (80 * StepTimer::StepClockRate)/1000000;		// the maximum time we spend looping in the ISR in step clocks
#elif SAME5x
	static constexpr uint32_t MinCalcIntervalDelta = (50 * StepTimer::StepClockRate)/1000000; 		// the smallest sensible interval between calculations (40us) in step timer clocks
	static constexpr uint32_t MinCalcIntervalCartesian = (50 * StepTimer::StepClockRate)/1000000;	// same as delta for now, but could be lower
	static constexpr uint32_t HiccupTime = (40 * StepTimer::StepClockRate)/1000000;					// how long we hiccup for
	static constexpr uint32_t MaxStepInterruptTime = (80 * StepTimer::StepClockRate)/1000000;		// the maximum time we spend looping in the ISR in step clocks
#endif
	static constexpr uint32_t WakeupTime = (100 * StepTimer::StepClockRate)/1000000;				// stop resting 100us before the move is due to end

	static void PrintMoves();											// print saved moves for debugging

#if USE_TC_FOR_STEP
	static uint32_t lastStepHighTime;									// when we last started a step pulse to a slow driver
#else
	static uint32_t lastStepLowTime;									// when we last completed a step pulse to a slow driver
#endif
	static uint32_t lastDirChangeTime;									// when we last change the DIR signal to a slow driver

	static uint32_t stepsRequested[NumDrivers], stepsDone[NumDrivers];

private:
	void StopDrive(size_t drive) noexcept;								// stop movement of a drive and recalculate the endpoint
	uint32_t WhenNextInterruptDue() const noexcept;						// return when the next interrupt is due relative to the move start time

#if !SINGLE_DRIVER
	void InsertDM(DriveMovement *dm) noexcept SPEED_CRITICAL;
	void RemoveDM(size_t drive) noexcept;
#endif

	void DebugPrintVector(const char *name, const float *vec, size_t len) const noexcept;

    DDA *next;								// The next one in the ring
	DDA *prev;								// The previous one in the ring

	volatile DDAState state;				// What state this DDA is in

	union
	{
		struct
		{
			uint16_t isPrintingMove : 1,	// True if this is a printing move and any of our extruders is moving
					 goingSlow : 1,			// True if we have slowed the movement because the Z probe is approaching its threshold
					 hadHiccup : 1;			// True if we had a hiccup while executing this move
		} flags;
		uint16_t all;						// so that we can print all the flags at once for debugging
	};

	int32_t endPoint[NumDrivers];  			// Machine coordinates in steps of the endpoint

	float acceleration;						// The acceleration to use
	float deceleration;						// The deceleration to use

    // These vary depending on how we connect the move with its predecessor and successor, but remain constant while the move is being executed
	float startSpeed;
	float endSpeed;
	float topSpeed;
	float accelDistance;
	float decelDistance;

	uint32_t clocksNeeded;

	// Values that are not set or accessed before Prepare is called
	struct
	{
		// These are calculated from the above and used in the ISR, so they are set up by Prepare()
		uint32_t moveStartTime;				// clock count at which the move was started
		uint32_t startSpeedTimesCdivA;		// the number of clocks it would have taken to reach the start speed from rest
		uint32_t topSpeedTimesCdivDPlusDecelStartClocks;
		int32_t extraAccelerationClocks;	// the additional number of clocks needed because we started the move at less than topSpeed. Negative after ReduceHomingSpeed has been called.

		// These are used only in delta calculations
#if DM_USE_FPU
		float zFraction;					// the Z movement fraction
#else
		int32_t cKc;						// the Z movement fraction multiplied by Kc and converted to integer
#endif
	} afterPrepare;

#if !SINGLE_DRIVER
    DriveMovement* activeDMs;				// list of contained DMs that need steps, in step time order
#endif

    DriveMovement ddms[NumDrivers];			// These describe the state of each drive movement

	static unsigned int stepErrors;
	static uint32_t maxTicksOverdue;
	static uint32_t maxOverdueIncrement;
};

// Return when the next interrupt is due relative to the move start time
inline uint32_t DDA::WhenNextInterruptDue() const noexcept
{
	return
#if SINGLE_DRIVER
			(ddms[0].state == DMState::moving) ? ddms[0].nextStepTime
#else
			(activeDMs != nullptr) ? activeDMs->nextStepTime
#endif
				: (clocksNeeded > DDA::WakeupTime) ? clocksNeeded - DDA::WakeupTime
					: 0;
}

// Schedule the next interrupt, returning true if we can't because it is already due
// Base priority must be >= NvicPriorityStep or interrupts disabled when calling this
inline bool DDA::ScheduleNextStepInterrupt(StepTimer& timer) const noexcept
{
	if (state == executing)
	{
		return timer.ScheduleCallbackFromIsr(WhenNextInterruptDue() + afterPrepare.moveStartTime);
	}
	return false;
}

// Insert a hiccup long enough to guarantee that we will exit the ISR
inline void DDA::InsertHiccup(uint32_t now) noexcept
{
	const uint32_t ticksDueAfterStart =
#if SINGLE_DRIVER
		(ddms[0].state == DMState::moving) ? ddms[0].nextStepTime
#else
									(activeDMs != nullptr) ? activeDMs->nextStepTime
#endif
										: (clocksNeeded > DDA::WakeupTime) ? clocksNeeded - DDA::WakeupTime
											: 0;
	afterPrepare.moveStartTime = now + DDA::HiccupTime - ticksDueAfterStart;
}

// Return the number of net steps already taken in this move by a particular drive
inline int32_t DDA::GetStepsTaken(size_t drive) const
{
	return ddms[drive].GetNetStepsTaken();
}

// Free up this DDA
inline void DDA::Free()
{
	state = empty;
}

#if HAS_SMART_DRIVERS

// Get the current full step interval for this axis or extruder
inline uint32_t DDA::GetStepInterval(size_t axis, uint32_t microstepShift) const noexcept
{
	const DriveMovement& dm = ddms[axis];
	return dm.state == DMState::moving ? dm.GetStepInterval(microstepShift) : 0;
}

#endif

#if SUPPORT_CLOSED_LOOP

// Get the current position, speed and acceleration
inline void DDA::GetCurrentMotion(MotionParameters& mParams, int32_t netMicrostepsTaken, int microstepShift) const noexcept
{
	return ddms[0].GetCurrentMotion(mParams, netMicrostepsTaken, microstepShift);
}

#endif

#endif	// SUPPORT_DRIVERS

#endif /* DDA_H_ */
