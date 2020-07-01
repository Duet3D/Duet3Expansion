/*
 * DDA.h
 *
 *  Created on: 7 Dec 2014
 *      Author: David
 */

#ifndef DDA_H_
#define DDA_H_

#include "RepRapFirmware.h"
#include "DriveMovement.h"
#include "GCodes/GCodes.h"			// for class RawMove, HomeAxes
#include "StepTimer.h"

struct CanMessageMovement;

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

	DDA(DDA* n);

	bool Init(const CanMessageMovement& msg);
	void Init();													// Set up initial positions for machine startup
	void Start(uint32_t tim) __attribute__ ((hot));					// Start executing the DDA, i.e. move the move.
	void StepDrivers(uint32_t now) __attribute__ ((hot));			// Take one step of the DDA, called by timed interrupt.
	bool ScheduleNextStepInterrupt(StepTimer& timer) const;			// Schedule the next interrupt, returning true if we can't because it is already due

	void SetNext(DDA *n) { next = n; }
	void SetPrevious(DDA *p) { prev = p; }
	void Complete() { state = completed; }
	bool Free();
	void Prepare(const CanMessageMovement& msg) __attribute__ ((hot));	// Calculate all the values and freeze this DDA
	bool HasStepError() const;

	DDAState GetState() const { return state; }
	DDA* GetNext() const { return next; }
	DDA* GetPrevious() const { return prev; }
	int32_t GetTimeLeft() const;
	void InsertHiccup(uint32_t now);

	// Filament monitor support
	int32_t GetStepsTaken(size_t drive) const;

	void MoveAborted();
	void StopDrivers(uint16_t whichDrivers);

	uint32_t GetClocksNeeded() const { return clocksNeeded; }
	uint32_t GetMoveFinishTime() const { return afterPrepare.moveStartTime + clocksNeeded; }

#if HAS_SMART_DRIVERS
	uint32_t GetStepInterval(size_t axis, uint32_t microstepShift) const;	// Get the current full step interval for this axis or extruder
#endif

	void DebugPrint() const;												// print the DDA only
	void DebugPrintAll() const;												// print the DDA and active DMs

	// Note on the following constant:
	// If we calculate the step interval on every clock, we reach a point where the calculation time exceeds the step interval.
	// The worst case is pure Z movement on a delta. On a Mini Kossel with 80 steps/mm with this firmware running on a Duet (84MHx SAM3X8 processor),
	// the calculation can just be managed in time at speeds of 15000mm/min (step interval 50us), but not at 20000mm/min (step interval 37.5us).
	// Therefore, where the step interval falls below 60us, we don't calculate on every step.
	// Note: the above measurements were taken some time ago, before some firmware optimisations.
	// The system clock of the SAME70 is running at 150MHz. Use the same defaults as for the SAM4E for now.
#ifdef SAMC21
	static constexpr uint32_t MinCalcIntervalDelta = (50 * StepTimer::StepClockRate)/1000000; 		// the smallest sensible interval between calculations (40us) in step timer clocks
	static constexpr uint32_t MinCalcIntervalCartesian = (50 * StepTimer::StepClockRate)/1000000;	// same as delta for now, but could be lower
	static constexpr uint32_t HiccupTime = (40 * StepTimer::StepClockRate)/1000000;					// how long we hiccup for
	static constexpr uint32_t MaxStepInterruptTime = (80 * StepTimer::StepClockRate)/1000000;		// the maximum time we spend looping in the ISR in step clocks
#else
	static constexpr uint32_t MinCalcIntervalDelta = (50 * StepTimer::StepClockRate)/1000000; 		// the smallest sensible interval between calculations (40us) in step timer clocks
	static constexpr uint32_t MinCalcIntervalCartesian = (50 * StepTimer::StepClockRate)/1000000;	// same as delta for now, but could be lower
	static constexpr uint32_t HiccupTime = (40 * StepTimer::StepClockRate)/1000000;					// how long we hiccup for
	static constexpr uint32_t MaxStepInterruptTime = (80 * StepTimer::StepClockRate)/1000000;		// the maximum time we spend looping in the ISR in step clocks
#endif
	static constexpr uint32_t WakeupTime = (100 * StepTimer::StepClockRate)/1000000;				// stop resting 100us before the move is due to end

	static void PrintMoves();										// print saved moves for debugging

	static uint32_t lastStepLowTime;								// when we last completed a step pulse to a slow driver
	static uint32_t lastDirChangeTime;								// when we last change the DIR signal to a slow driver

private:
	DriveMovement *FindDM(size_t drive) const;
	void StopDrive(size_t drive);									// stop movement of a drive and recalculate the endpoint
	void InsertDM(DriveMovement *dm) __attribute__ ((hot));
	void RemoveDM(size_t drive);
	void ReleaseDMs();
	void DebugPrintVector(const char *name, const float *vec, size_t len) const;

    DDA *next;								// The next one in the ring
	DDA *prev;								// The previous one in the ring

	volatile DDAState state;				// What state this DDA is in

	union
	{
		struct
		{
			uint16_t goingSlow : 1,					// True if we have slowed the movement because the Z probe is approaching its threshold
					 hadHiccup : 1,					// True if we had a hiccup while executing this move
					 stopAllDrivesOnEndstopHit : 1;	// True if hitting an endstop stops the entire move
		} flags;
		uint16_t all;								// so that we can print all the flags at once for debugging
	};

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
		int32_t cKc;						// The Z movement fraction multiplied by Kc and converted to integer
	} afterPrepare;

    DriveMovement* activeDMs;				// list of contained DMs that need steps, in step time order
	DriveMovement *pddm[NumDrivers];		// These describe the state of each drive movement
};

// Find the DriveMovement record for a given drive, or return nullptr if there isn't one
inline DriveMovement *DDA::FindDM(size_t drive) const
{
	return pddm[drive];
}

// Schedule the next interrupt, returning true if we can't because it is already due
// Base priority must be >= NvicPriorityStep or interrupts disabled when calling this
inline bool DDA::ScheduleNextStepInterrupt(StepTimer& timer) const
{
	if (state == executing)
	{
		const uint32_t whenDue = ((activeDMs != nullptr) ? activeDMs->nextStepTime
									: (clocksNeeded > DDA::WakeupTime) ? clocksNeeded - DDA::WakeupTime
										: 0)
								+ afterPrepare.moveStartTime;
		return timer.ScheduleCallbackFromIsr(whenDue);
	}
	return false;
}

// Insert a hiccup long enough to guarantee that we will exit the ISR
inline void DDA::InsertHiccup(uint32_t now)
{
	const uint32_t ticksDueAfterStart = (activeDMs != nullptr) ? activeDMs->nextStepTime
										: (clocksNeeded > DDA::WakeupTime) ? clocksNeeded - DDA::WakeupTime
											: 0;
	afterPrepare.moveStartTime = now + DDA::HiccupTime - ticksDueAfterStart;
}

#if HAS_SMART_DRIVERS

// Get the current full step interval for this axis or extruder
inline uint32_t DDA::GetStepInterval(size_t axis, uint32_t microstepShift) const
{
	const DriveMovement * const dm = FindDM(axis);
	return (dm != nullptr) ? dm->GetStepInterval(microstepShift) : 0;
}

#endif

#endif /* DDA_H_ */
