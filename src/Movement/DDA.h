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
#include "MoveSegment.h"
#include "ExtruderShaper.h"

#if SUPPORT_DRIVERS

# include "DriveMovement.h"
# include "StepTimer.h"

# if SUPPORT_CLOSED_LOOP
#  include <ClosedLoop/ClosedLoop.h>
# endif

struct CanMessageMovementLinearShaped;
struct CanMessageStopMovement;

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

// This defines a single coordinated movement of one or several motors
class DDA
{
	friend class DriveMovement;
	friend class ExtruderShaper;
	friend class PrepParams;
	friend class Move;

public:

	enum DDAState : uint8_t
	{
		empty,				// empty or being filled in
		provisional,		// ready, but could be subject to modifications
		committed			// has been converted into move segments already
	};

	DDA(DDA* n) noexcept;

	void* operator new(size_t count) { return Tasks::AllocPermanent(count); }
	void* operator new(size_t count, std::align_val_t align) { return Tasks::AllocPermanent(count, align); }
	void operator delete(void* ptr) noexcept {}
	void operator delete(void* ptr, std::align_val_t align) noexcept {}

	void Init() noexcept;															// Set up initial positions for machine startup
	bool Init(const CanMessageMovementLinearShaped& msg) noexcept SPEED_CRITICAL;	// Set up a move from a CAN message

#if 0
#if DEDICATED_STEP_TIMER
	bool ScheduleNextStepInterrupt() const noexcept SPEED_CRITICAL;					// Schedule the next interrupt, returning true if we can't because it is already due
#else
	bool ScheduleNextStepInterrupt(StepTimer& timer) const noexcept SPEED_CRITICAL;	// Schedule the next interrupt, returning true if we can't because it is already due
#endif
#endif

	void SetNext(DDA *n) noexcept { next = n; }
	void SetPrevious(DDA *p) noexcept { prev = p; }
	void Free() noexcept;
//unused?	bool IsPrintingMove() const noexcept { return flags.isPrintingMove; }

	DDAState GetState() const noexcept { return state; }
	DDA* GetNext() const noexcept { return next; }
	DDA* GetPrevious() const noexcept { return prev; }
	int32_t GetTimeLeft() const noexcept;

	uint32_t GetClocksNeeded() const noexcept { return clocksNeeded; }
	uint32_t GetMoveFinishTime() const noexcept { return afterPrepare.moveStartTime + clocksNeeded; }

	int32_t GetPosition(size_t driver) const noexcept { return endPoint[driver]; }

	void DebugPrint() const noexcept;																// print the DDA only

	static uint32_t GetAndClearMaxTicksOverdue() noexcept;
	static uint32_t GetAndClearMaxOverdueIncrement() noexcept;


#if USE_TC_FOR_STEP
	static uint32_t lastStepHighTime;									// when we last started a step pulse to a slow driver
#else
	static uint32_t lastStepLowTime;									// when we last completed a step pulse to a slow driver
#endif
	static uint32_t lastDirChangeTime;									// when we last change the DIR signal to a slow driver

private:
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

//	static unsigned int stepErrors;
//	static uint32_t maxTicksOverdue;
//	static uint32_t maxOverdueIncrement;
};

// Free up this DDA
inline void DDA::Free()
{
	state = empty;
}

#endif	// SUPPORT_DRIVERS

#endif /* DDA_H_ */
