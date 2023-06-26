/*
 * DDA.h
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#ifndef DDA_H_
#define DDA_H_

#include <RepRapFirmware.h>
#include <InputShaperPlan.h>
#include <Platform/Tasks.h>
#include "MoveSegment.h"

#if SUPPORT_DRIVERS

# include "DriveMovement.h"
# include "StepTimer.h"

# if SUPPORT_CLOSED_LOOP
#  include <ClosedLoop/ClosedLoop.h>
# endif

struct CanMessageMovementLinear;
struct CanMessageMovementLinearShaped;
struct CanMessageStopMovement;

// Struct for passing parameters to the DriveMovement Prepare methods
struct PrepParams
{
	// Parameters used for all types of motion
	static constexpr float totalDistance = 1.0;
	float accelDistance;
	float decelStartDistance;
	float accelClocks, steadyClocks, decelClocks;
	float acceleration, deceleration;				// the acceleration and deceleration to use, both positive

	InputShaperPlan shapingPlan;

	// Parameters used only for delta moves
	float initialX;
	float initialY;
	const LinearDeltaKinematics *dparams;
	float a2plusb2;								// sum of the squares of the X and Y movement fractions
	float dvecX, dvecY, dvecZ;

	// Calculate the steady clocks and set the total clocks in the DDA
	void Finalise(float topSpeed) noexcept;

	// Get the total clocks needed
	float TotalClocks() const noexcept { return accelClocks + steadyClocks + decelClocks; }

	// Set up the parameters from the DDA, excluding steadyClocks because that may be affected by input shaping
	void SetFromDDA(const DDA& dda) noexcept;
};

// This defines a single coordinated movement of one or several motors
class DDA
{
	friend class DriveMovement;
	friend class AxisShaper;
	friend class ExtruderShaper;
	friend class PrepParams;

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

	void Init() noexcept;															// Set up initial positions for machine startup
	bool Init(const CanMessageMovementLinear& msg) noexcept SPEED_CRITICAL;			// Set up a move from a CAN message
	bool Init(const CanMessageMovementLinearShaped& msg) noexcept SPEED_CRITICAL;	// Set up a move from a CAN message
	void Start(uint32_t tim) noexcept SPEED_CRITICAL;								// Start executing the DDA, i.e. move the move.
	void StepDrivers(uint32_t now) noexcept SPEED_CRITICAL;							// Take one step of the DDA, called by timed interrupt.

#if DEDICATED_STEP_TIMER
	bool ScheduleNextStepInterrupt() const noexcept SPEED_CRITICAL;					// Schedule the next interrupt, returning true if we can't because it is already due
#else
	bool ScheduleNextStepInterrupt(StepTimer& timer) const noexcept SPEED_CRITICAL;	// Schedule the next interrupt, returning true if we can't because it is already due
#endif

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
	uint32_t GetStepInterval(size_t axis, uint32_t microstepShift) const noexcept;					// Get the current full step interval for this axis or extruder
#endif

#if SUPPORT_CLOSED_LOOP
	void GetCurrentMotion(size_t driver, uint32_t ticksSinceStart, MotionParameters& mParams) noexcept;	// get the current desired position, speed and acceleration
	uint32_t GetStartTime() const noexcept { return afterPrepare.moveStartTime; }
	void SetCompleted() noexcept { state = completed; }
	float GetFullDistance(size_t drive) const noexcept { return directionVector[drive]; }
#endif

	void DebugPrint() const noexcept;																// print the DDA only
	void DebugPrintAll() const noexcept;															// print the DDA and active DMs

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
#if SAMC21 || RP2040
	static constexpr uint32_t MinCalcInterval = (100 * StepTimer::StepClockRate)/1000000;			// the smallest sensible interval between calculations (40us) in step timer clocks
	static constexpr uint32_t HiccupTime = (50 * StepTimer::StepClockRate)/1000000;					// how long we hiccup for
#elif SAME5x
	static constexpr uint32_t MinCalcInterval = (50 * StepTimer::StepClockRate)/1000000; 			// the smallest sensible interval between calculations (40us) in step timer clocks
	static constexpr uint32_t HiccupTime = (40 * StepTimer::StepClockRate)/1000000;					// how long we hiccup for
#endif
	static constexpr uint32_t MaxStepInterruptTime = (80 * StepTimer::StepClockRate)/1000000;		// the maximum time we spend looping in the ISR in step clocks
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
	uint32_t WhenNextInterruptDue() const noexcept;						// return when the next interrupt is due relative to the move start time
	void EnsureSegments(const PrepParams& params) noexcept;
	void ReleaseSegments() noexcept;

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
			 	 	 usePressureAdvance : 1,	// True if pressure advance should be applied to any forward extrusion
					 hadHiccup : 1;			// True if we had a hiccup while executing this move
		};
		uint16_t all;						// so that we can print all the flags at once for debugging
	} flags;

	int32_t endPoint[NumDrivers];  			// Machine coordinates in steps of the endpoint

	float directionVector[NumDrivers];		// How much each drive is moving
	float acceleration;						// The acceleration to use
	float deceleration;						// The deceleration to use

    // These vary depending on how we connect the move with its predecessor and successor, but remain constant while the move is being executed
	float startSpeed;
	float endSpeed;
	float topSpeed;
	static constexpr float totalDistance = 1.0;		// we normalise all move to unit distance

	uint32_t clocksNeeded;

	// Values that are not set or accessed before Prepare is called
	struct
	{
		// These are calculated from the above and used in the ISR, so they are set up by Prepare()
		uint32_t moveStartTime;				// clock count at which the move was started
	} afterPrepare;

	MoveSegment* segments;					// linked list of move segments used by axis DMs

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
			(likely(ddms[0].state >= DMState::firstMotionState)) ? ddms[0].nextStepTime
#else
			(activeDMs != nullptr) ? activeDMs->nextStepTime
#endif
				: (clocksNeeded > DDA::WakeupTime) ? clocksNeeded - DDA::WakeupTime
					: 0;
}

// Schedule the next interrupt, returning true if we can't because it is already due
// Base priority must be >= NvicPriorityStep or interrupts disabled when calling this
#if DEDICATED_STEP_TIMER

inline bool DDA::ScheduleNextStepInterrupt() const noexcept
{
# if SUPPORT_CLOSED_LOOP
	if (!ClosedLoop::GetClosedLoopInstance(0)->IsClosedLoopEnabled())
# endif
	{
		if (likely(state == executing))
		{
			return StepTimer::ScheduleStepInterruptFromIsr(WhenNextInterruptDue() + afterPrepare.moveStartTime);
		}
	}
	return false;
}

#else

inline bool DDA::ScheduleNextStepInterrupt(StepTimer& timer) const noexcept
{
# if SUPPORT_CLOSED_LOOP
	if (!ClosedLoop::GetClosedLoopEnabled(0))
# endif
	{
		if (likely(state == executing))
		{
			return timer.ScheduleCallbackFromIsr(WhenNextInterruptDue() + afterPrepare.moveStartTime);
		}
	}
	return false;
}

#endif

// Insert a hiccup long enough to guarantee that we will exit the ISR
inline void DDA::InsertHiccup(uint32_t now) noexcept
{
	const uint32_t ticksDueAfterStart =
#if SINGLE_DRIVER
		(ddms[0].state >= DMState::firstMotionState) ? ddms[0].nextStepTime
#else
									(activeDMs != nullptr) ? activeDMs->nextStepTime
#endif
										: (clocksNeeded > DDA::WakeupTime) ? clocksNeeded - DDA::WakeupTime
											: 0;
	afterPrepare.moveStartTime = now + DDA::HiccupTime - ticksDueAfterStart;
}

// Return the number of net steps already taken in this move by a particular drive
inline int32_t DDA::GetStepsTaken(size_t drive) const noexcept
{
#if SUPPORT_CLOSED_LOOP
	if (ClosedLoop::GetClosedLoopInstance(drive)->IsClosedLoopEnabled())
	{
		return ddms[drive].GetNetStepsTakenClosedLoop(topSpeed, (int32_t)(StepTimer::GetTimerTicks() - afterPrepare.moveStartTime));
	}
#endif
	return ddms[drive].GetNetStepsTaken();
}

// Free up this DDA
inline void DDA::Free()
{
	ReleaseSegments();
	state = empty;
}

#if HAS_SMART_DRIVERS

// Get the current full step interval for this axis or extruder
inline uint32_t DDA::GetStepInterval(size_t axis, uint32_t microstepShift) const noexcept
{
	const DriveMovement& dm = ddms[axis];
	return dm.state >= DMState::firstMotionState ? dm.GetStepInterval(microstepShift) : 0;
}

#endif

#if SUPPORT_CLOSED_LOOP

// Get the current position relative to the start of this move, speed and acceleration. Units are microsteps and step clocks.
// Interrupts are disabled on entry and must remain disabled.
inline void DDA::GetCurrentMotion(size_t driver, uint32_t ticksSinceStart, MotionParameters& mParams) noexcept
{
	return ddms[driver].GetCurrentMotion(*this, ticksSinceStart, mParams);
}

#endif

#endif	// SUPPORT_DRIVERS

#endif /* DDA_H_ */
