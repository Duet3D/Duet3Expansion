/*
 * Move.cpp
 *
 *  Created on: 7 Dec 2014
 *      Author: David

 A note on bed levelling:

 As at version 1.21 we support two types of bed compensation:
 1. The old 3, 4 and 5-point compensation using a RandomProbePointSet. We will probably discontinue this soon.
 2. Mesh bed levelling

 There is an interaction between using G30 to home Z or set a precise Z=0 height just before a print, and bed compensation.
 Consider the following sequence:
 1. Home Z, using either G30 or an endstop.
 2. Run G29 to generate a height map. If the Z=0 point has drifted off, the height map may have a Z offset.
 3. Use G30 to get an accurate Z=0 point. We want to keep the shape of the height map, but get rid of the offset.
 4. Run G29 to generate a height map. This should generate a height map with on offset at the point we just probed.
 5. Cancel bed compensation. The height at the point we just probed should be zero.

 So as well as maintaining a height map, we maintain a Z offset from it. The procedure is:
 1. Whenever bed compensation is not being used, the Z offset should be zero.
 2. Whenever we run G29 to probe the bed, we have a choice:
 (a) accept that the map may have a height offset; and set the Z offset to zero. This is what we do currently.
 (b) normalise the height map to zero, adjust the Z=0 origin, and set the Z offset to zero.
 3. When we run G30 to reset the Z=0 height, and we have a height map loaded, we adjust the Z offset to be the negative of the
    height map indication of that point.
 4. If we now cancel the height map, we also clear the Z offset, and the height at the point we probed remains correct.
 5. If we now run G29 to probe again, the height map should have near zero offset at the point we probed, if there has been no drift.

 Before we introduced the Z offset, at step 4 we would have a potentially large Z error as if the G30 hadn't been run,
 and at step 5 the new height map would have an offset again.

 */

#include "Move.h"

#if SUPPORT_DRIVERS

#include "StepTimer.h"
#include "MoveTiming.h"
#include <Platform/Platform.h>
#include <CAN/CanInterface.h>
#include <CanMessageGenericParser.h>
#include <CanMessageGenericTables.h>
#include <CanMessageFormats.h>
#include <CanMessageBuffer.h>
#include <Platform/TaskPriorities.h>
#include <AppNotifyIndices.h>

#if HAS_SMART_DRIVERS
# include "StepperDrivers/TMC51xx.h"
# include "StepperDrivers/TMC22xx.h"
#endif

#if USE_TC_FOR_STEP
# if SAME5x
#  include <hri_tc_e54.h>
# elif SAMC21
#  include <hri_tc_c21.h>
# endif
#endif

#if 1	//debug
unsigned int getCanMoveTimeoutErrs;
#endif

#if SAMC21 || RP2040
constexpr size_t MoveTaskStackWords = 180;
#else
constexpr size_t MoveTaskStackWords = 220;
#endif

static Task<MoveTaskStackWords> *moveTask;

extern "C" [[noreturn]] void MoveLoop(void * param) noexcept
{
	static_cast<Move*>(param)->TaskLoop();
}

Move::Move() noexcept
{
#if !DEDICATED_STEP_TIMER
	timer.SetCallback(Move::TimerCallback, CallbackParameter(this));
#endif

	for (size_t i = 0; i < NumDrivers; ++i)
	{
		stepsPerMm[i] = DefaultStepsPerMm;
		directions[i] = true;
	}
}

void Move::Init() noexcept
{
#if HAS_SMART_DRIVERS
	SmartDrivers::Init();
# if SUPPORT_CLOSED_LOOP
	ClosedLoop::Init();						// this must be called AFTER SmartDrivers::Init()
# endif
	temperatureShutdownDrivers.Clear();
	temperatureWarningDrivers.Clear();
#endif

	// Initialise stepper driver structs
	for (size_t i = 0; i < NumDrivers; ++i)
	{
		dms[i].Init(i);
		{
			const uint32_t driverBit = 1u << (StepPins[i] & 31);
			dms[i].driversNormallyUsed = driverBit;
#if !SINGLE_DRIVER
			allDriverBits |= driverBit;
#endif
		}

#if HAS_SMART_DRIVERS
		SetMicrostepping(i, 16, true);
#endif
#if DIFFERENTIAL_STEPPER_OUTPUTS
		// Step pins
		IoPort::SetPinMode(StepPins[i], OUTPUT_LOW);
		SetDriveStrength(StepPins[i], 2);
		IoPort::SetPinMode(InvertedStepPins[i], OUTPUT_HIGH);
		SetDriveStrength(InvertedStepPins[i], 2);

		// Set up the CCL to invert the step output from PB10 to the inverted output on PA11
		MCLK->APBCMASK.reg |= MCLK_APBCMASK_CCL;

# if USE_TC_FOR_STEP
		// On the EXP1XD the step pin is also output TC1.0 and TC1 is not used for anything else
		// Use it to generate the step pulse
		EnableTcClock(StepGenTcNumber, GclkNum48MHz);

		hri_tc_set_CTRLA_SWRST_bit(StepGenTc);

		hri_tc_write_CTRLA_reg(StepGenTc, TC_CTRLA_MODE_COUNT16 | TcPrescalerRegVal);
		hri_tc_set_CTRLB_reg(StepGenTc, TC_CTRLBSET_ONESHOT);
		hri_tc_write_DBGCTRL_reg(StepGenTc, 0);
		hri_tc_write_EVCTRL_reg(StepGenTc, 0);
		hri_tc_write_WAVE_reg(StepGenTc, TC_WAVE_WAVEGEN_MPWM);

		StepGenTc->CC[0].reg = (uint16_t)slowDriverStepTimingClocks[0];
		StepGenTc->CCBUF[0].reg = (uint16_t)slowDriverStepTimingClocks[0];

		hri_tc_set_CTRLA_ENABLE_bit(StepGenTc);

		delayMicroseconds(2);												// this avoids a glitch on the step output

		SetPinFunction(StepPins[i], GpioPinFunction::E);					// TC1.0
# else
		SetPinFunction(StepPins[i], GpioPinFunction::I);					// CCL1in5
# endif
		CCL->CTRL.reg = 0;													// disable the CCL
		CCL->SEQCTRL[0].reg = CCL_SEQCTRL_SEQSEL_DISABLE;
		CCL->LUTCTRL[1].reg &= ~CCL_LUTCTRL_ENABLE;
		CCL->LUTCTRL[1].reg =
# if USE_TC_FOR_STEP
			  	  	  	  CCL_LUTCTRL_INSEL2(CCL_LUTCTRL_INSEL0_TC_Val)		// take input from TC1.0
# else
						  CCL_LUTCTRL_INSEL2(CCL_LUTCTRL_INSEL0_IO_Val)		// take input from CCL1 IN2
# endif
						| CCL_LUTCTRL_INSEL0(CCL_LUTCTRL_INSEL0_MASK_Val)
						| CCL_LUTCTRL_INSEL0(CCL_LUTCTRL_INSEL0_MASK_Val)
						| CCL_LUTCTRL_TRUTH(0b00001111);
		CCL->LUTCTRL[1].reg |= CCL_LUTCTRL_ENABLE;
		CCL->CTRL.reg = CCL_CTRL_ENABLE;
		SetPinFunction(InvertedStepPins[i], GpioPinFunction::I);			// CCL1out1 do this at the end to avoid a glitch on the output

		// Direction pins
		IoPort::SetPinMode(DirectionPins[i], OUTPUT_LOW);
		SetDriveStrength(DirectionPins[i], 2);
		IoPort::SetPinMode(InvertedDirectionPins[i], OUTPUT_HIGH);
		SetDriveStrength(InvertedDirectionPins[i], 2);

		// Enable pins
		IoPort::SetPinMode(EnablePins[i], OUTPUT_LOW);
		SetDriveStrength(EnablePins[i], 2);
		IoPort::SetPinMode(InvertedEnablePins[i], OUTPUT_HIGH);
		SetDriveStrength(InvertedEnablePins[i], 2);
		enableValues[i] = 1;
		driverIsEnabled[i] = false;
#else
		// Step pins
# if ACTIVE_HIGH_STEP
		IoPort::SetPinMode(StepPins[i], OUTPUT_LOW);
# else
		IoPort::SetPinMode(StepPins[i], OUTPUT_HIGH);
# endif
# if !HAS_SMART_DRIVERS
		SetDriveStrength(StepPins[i], 2);
# endif
# if RP2040
		SetPinFunction(StepPins[i], GpioPinFunction::Sio);			// enable fast stepping - must do this after the call to SetPinMode
# endif

		// Direction pins
# if ACTIVE_HIGH_DIR
		IoPort::SetPinMode(DirectionPins[i], OUTPUT_LOW);
# else
		IoPort::SetPinMode(DirectionPins[i], OUTPUT_HIGH);
# endif
# if !HAS_SMART_DRIVERS
		SetDriveStrength(DirectionPins[i], 2);
# endif

# if !HAS_SMART_DRIVERS
		// Enable pins
#  if ACTIVE_HIGH_ENABLE
		IoPort::SetPinMode(EnablePins[i], OUTPUT_LOW);
		enableValues[i] = 1;
#  else
		IoPort::SetPinMode(EnablePins[i], OUTPUT_HIGH);
		enableValues[i] = 0;
#  endif
		SetDriveStrength(EnablePins[i], 2);
		driverIsEnabled[i] = false;
# endif
#endif

		enableValues[i] = 0;
		driverAtIdleCurrent[i] = false;
		idleCurrentFactor[i] = 0.3;
		motorCurrents[i] = 0.0;
#if SUPPORT_BRAKE_PWM
		currentBrakePwm[i] = 0.0;
# if !defined(M23CL)
		brakeOnPins[i] = NoPin;
		brakeVoltages[i] = FullyOnBrakeVoltage;
# endif
#endif
		driverStates[i] = DriverStateControl::driverDisabled;
		brakeOffDelays[i] = 0;
		motorOffDelays[i] = DefaultDelayAfterBrakeOn;
	}

# if HAS_STALL_DETECT
	eventOnStallDrivers.Clear();
#endif

#if HAS_SMART_DRIVERS && HAS_VOLTAGE_MONITOR
	warnDriversNotPowered = false;
#endif

#ifdef M23CL
	// Set the brake control pins to outputs, leaving the brake engaged
	pinMode(BrakeOnPin, OUTPUT_LOW);
	pinMode(BrakePwmPin, OUTPUT_LOW);
#endif

	moveTask = new Task<MoveTaskStackWords>;
	moveTask->Create(MoveLoop, "Move", this, TaskPriority::MovePriority);
}

void Move::Exit() noexcept
{
	StepTimer::DisableTimerInterrupt();
	moveTask->TerminateAndUnlink();
}

#if HAS_VOLTAGE_MONITOR || HAS_12V_MONITOR
void Move::Spin(bool powered) noexcept
#else
void Move::Spin() noexcept
#endif
{
# if SUPPORT_BRAKE_PWM
	const float currentVinVoltage = Platform::GetCurrentVinVoltage();
# endif
	// Check whether we need to turn any brake solenoids or motors off, or adjust brake PWM
	for (size_t driver = 0; driver < NumDrivers; ++driver)
	{
		if (brakeOffTimers[driver].CheckAndStop(brakeOffDelays[driver]))
		{
			DisengageBrake(driver);
		}
		if (motorOffTimers[driver].CheckAndStop(motorOffDelays[driver]))
		{
			InternalDisableDrive(driver);
		}

#if SUPPORT_BRAKE_PWM
		// If the brake solenoid is activated, adjust the PWM if necessary
		if (currentBrakePwm[driver] != 0.0 && currentVinVoltage > 0.0)
		{
			const float requestedVoltage =
# ifdef M23CL
											M23CLBrakeVoltage;
# else
											brakeVoltages[driver];
# endif
			const float newBrakePwm = (currentVinVoltage < requestedVoltage) ? 1.0 : requestedVoltage/currentVinVoltage;
			if (fabsf(newBrakePwm - currentBrakePwm[driver]) >= 0.05)
			{
# ifdef M23CL
				IoPort::WriteAnalog(BrakePwmPin, newBrakePwm, BrakePwmFrequency);
# else
				brakePwmPorts[driver].WriteAnalog(newBrakePwm);
# endif
				currentBrakePwm[driver] = newBrakePwm;
			}
		}
#endif
	}

#if HAS_SMART_DRIVERS
# if HAS_VOLTAGE_MONITOR
	SmartDrivers::Spin(powered);
# else
	SmartDrivers::Spin(true);
# endif

	// Check one TMC driver for warnings and errors
	if (enableValues[nextDriveToPoll] >= 0)				// don't poll driver if it is flagged "no poll"
	{
		StandardDriverStatus stat = GetDriverStatus(nextDriveToPoll, true, true);
		const DriversBitmap mask = DriversBitmap::MakeFromBits(nextDriveToPoll);

		// First set the driver temperature status for the temperature sensor code
		if (stat.ot)
		{
			temperatureShutdownDrivers |= mask;
		}
		else
		{
			if (stat.otpw)
			{
				temperatureWarningDrivers |= mask;
			}
			else
			{
				temperatureWarningDrivers &= ~mask;
			}
			temperatureShutdownDrivers &= ~mask;
		}

		// Deal with the open load bits
		// The driver often produces a transient open-load error, especially in stealthchop mode, so we require the condition to persist before we report it.
		// So clear them unless they have been active for the minimum time.
		MillisTimer& timer = openLoadTimers[nextDriveToPoll];
		if (stat.IsAnyOpenLoadBitSet())
		{
			if (timer.IsRunning())
			{
				if (!timer.CheckNoStop(OpenLoadTimeout))
				{
					stat.ClearOpenLoadBits();
				}
			}
			else
			{
				timer.Start();
				stat.ClearOpenLoadBits();
			}
		}
		else
		{
			timer.Stop();
		}

		const StandardDriverStatus oldStatus = lastEventStatus[nextDriveToPoll];
		lastEventStatus[nextDriveToPoll] = stat;
		if (stat.HasNewErrorSince(oldStatus))
		{
			// It's a new error
			CanInterface::RaiseEvent(EventType::driver_error, stat.AsU16(), nextDriveToPoll, "", va_list());
		}
		else if (stat.HasNewWarningSince(oldStatus))
		{
			// It's a new warning
			CanInterface::RaiseEvent(EventType::driver_warning, stat.AsU16(), nextDriveToPoll, "", va_list());
		}

# if HAS_STALL_DETECT
		if (stat.HasNewStallSince(oldStatus) && eventOnStallDrivers.Intersects(mask))
		{
			CanInterface::RaiseEvent(EventType::driver_stall, 0, nextDriveToPoll, "", va_list());
		}
# endif
	}

	// Advance drive number ready for next time
	++nextDriveToPoll;
	if (nextDriveToPoll == MaxSmartDrivers)
	{
		nextDriveToPoll = 0;
	}
#endif
}

[[noreturn]] void Move::TaskLoop() noexcept
{
#if !DEDICATED_STEP_TIMER
	timer.SetCallback(Move::TimerCallback, CallbackParameter(this));
#endif
	while (true)
	{
		// Get another move and add it to the ring
#if 1	//debug
		CanMessageBuffer *buf;
		for (;;)
		{
			buf = CanInterface::GetCanMove(2000);
			if (buf != nullptr)
			{
				break;
			}
			buf = CanInterface::GetCanMove(0);
			if (buf != nullptr)
			{
				++getCanMoveTimeoutErrs;
				break;
			}
		}
#else
		CanMessageBuffer *buf = CanInterface::GetCanMove(TaskBase::TimeoutUnlimited);
#endif
		MicrosecondsTimer prepareTimer;
		const CanMessageType msgType = buf->id.MsgType();
		switch (msgType)
		{
		case CanMessageType::movementLinearShaped:
			{
				const bool moveAdded = AddMove(buf->msg.moveLinearShaped);
				if (moveAdded)
				{
					scheduledMoves++;
				}
				const uint32_t elapsedTime = prepareTimer.Read();
				if (elapsedTime > Move::maxPrepareTime)
				{
					Move::maxPrepareTime = elapsedTime;
				}
			}
			break;

		default:				// should not happen
			break;
		}

		CanMessageBuffer::Free(buf);
	}
}

// Movement error handling
void Move::LogStepError(uint8_t type) noexcept
{
	++numStepErrors;
	stepErrorTypesLogged.SetBit(type);
}

void Move::Diagnostics(const StringRef& reply) noexcept
{
	const float delayToReport = (float)StepTimer::GetMovementDelay() * (1000.0/(float)StepTimer::GetTickRate());
	reply.catf("Moves scheduled %" PRIu32 ", hiccups %u (%.2fms), segs %u, step errors %u (types 0x%x), maxLate %" PRIi32 " maxPrep %" PRIu32,
					scheduledMoves, numHiccups, (double)delayToReport, MoveSegment::NumCreated(),
					numStepErrors, stepErrorTypesLogged.GetRaw(), DriveMovement::GetAndClearMaxStepsLate(), maxPrepareTime);
	numHiccups = 0;
	maxPrepareTime = 0;
	numStepErrors = 0;
	stepErrorTypesLogged.Clear();
#if 1	//debug
	reply.catf(", ebfmin %.2f max %.2f", (double)minExtrusionPending, (double)maxExtrusionPending);
	minExtrusionPending = maxExtrusionPending = 0.0;
#endif
}

// Set up a remote move. Return true if it represents real movement, else false.
// All values have already been converted to step clocks and the total distance has been normalised to 1.0.
// The whenToExecute field of the movement message has already bee converted to local time
// This version handles the new movement message that includes the input shaping plan and passes extruder movement as distance, not steps
bool Move::AddMove(const CanMessageMovementLinearShaped& msg) noexcept
{
	// Prepare for movement
	PrepParams params;

	// Normalise the move to unit distance
	params.acceleration = msg.acceleration;
	params.deceleration = msg.deceleration;
	params.accelClocks = msg.accelerationClocks;
	params.steadyClocks = msg.steadyClocks;
	params.decelClocks = msg.decelClocks;

	// We occasionally receive a message with zero clocks needed. This messes up the calculations, so add one steady clock in this case.
	uint32_t clocksNeeded = params.accelClocks + params.steadyClocks + params.decelClocks;
	if (clocksNeeded == 0)
	{
		params.steadyClocks = clocksNeeded = 1;
	}

	const float accelDistanceExTopSpeed = -0.5 * params.acceleration * fsquare((float)params.accelClocks);
	const float decelDistanceExTopSpeed = -0.5 * params.deceleration * fsquare((float)params.decelClocks);
	const float topSpeed = (params.totalDistance - accelDistanceExTopSpeed - decelDistanceExTopSpeed)/clocksNeeded;

	params.accelDistance =      accelDistanceExTopSpeed + topSpeed * params.accelClocks;
	const float decelDistance = decelDistanceExTopSpeed + topSpeed * params.decelClocks;
	params.decelStartDistance =  1.0 - decelDistance;

	MovementFlags segFlags;
	segFlags.Clear();
	segFlags.nonPrintingMove = !msg.usePressureAdvance;
	segFlags.noShaping = !msg.useLateInputShaping;

	for (size_t drive = 0; drive < msg.numDrivers; drive++)
	{
		if (drive < NumDrivers)
		{
			if ((msg.extruderDrives & (1u << drive)) != 0)
			{
				// It's an extruder
				const float extrusionRequested = msg.perDrive[drive].extrusion;
				if (extrusionRequested != 0.0)
				{
					AddLinearSegments(drive, msg.whenToExecute, params, extrusionRequested, segFlags);
				}
			}
			else
			{
				const float delta = (float)msg.perDrive[drive].steps;
				if (delta != 0.0)
				{
					AddLinearSegments(drive, msg.whenToExecute, params, delta, segFlags);
				}
			}
		}
		else if (Platform::Debug(Module::Move))
		{
			debugPrintf("Ignored movement command for drive %u\n", drive);
		}
	}
	return true;
}

// Get the number of steps taken by the last move, if it was an isolated move
int32_t Move::GetLastMoveStepsTaken(size_t drive) const noexcept
{
	const DriveMovement& dm = dms[drive];
	return dm.currentMotorPosition - dm.positionAtMoveStart;
}

#if SINGLE_DRIVER

// This is called by the interrupt service routine to execute steps.
// It returns true if it needs to be called again on the DDA of the new current move, otherwise false.
// This must be as fast as possible, because it determines the maximum movement speed.
// This may occasionally get called prematurely, so it must check that a step is actually due before generating one.
#if SAMC21 || RP2040
__attribute__((section(".time_critical")))
#endif
void Move::StepDrivers(uint32_t now) noexcept
{
# if SUPPORT_CLOSED_LOOP
	if (dms[0].closedLoopControl.IsClosedLoopEnabled())
	{
		return;
	}
# endif

	// Determine whether the driver is due for stepping, overdue, or will be due very shortly
	if (dms[0].state >= DMState::firstMotionState && (int32_t)(dms[0].nextStepTime - now) <= (int32_t)MoveTiming::MinInterruptInterval)	// if the next step is due
	{
		// Step the driver

# if SUPPORT_SLOW_DRIVERS
		if (IsSlowDriver())									// if using a slow driver
		{
#  if USE_TC_FOR_STEP
			if (dms[0].driversCurrentlyUsed != 0)
			{
				const uint32_t lastStepPulseTime = lastStepHighTime;
				uint32_t rawNow;
				do
				{
					rawNow = StepTimer::GetTimerTicks();
				} while (rawNow - lastStepPulseTime < GetSlowDriverStepPeriodClocks() || rawNow - lastDirChangeTime < GetSlowDriverDirSetupClocks());
				StepGenTc->CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;
				lastStepHighTime = StepTimer::GetTimerTicks();
			}
			PrepareForNextSteps(now);
#  else
			uint32_t lastStepPulseTime = lastStepLowTime;
			uint32_t rawNow;
			do
			{
				rawNow = StepTimer::GetTimerTicks();
			} while (rawNow - lastStepPulseTime < GetSlowDriverStepLowClocks() || rawNow - lastDirChangeTime < GetSlowDriverDirSetupClocks());
			StepDriversHigh(dms[0].driversCurrentlyUsed);						// generate the step
			lastStepPulseTime = StepTimer::GetTimerTicks();
			PrepareForNextSteps(now);

			// 3a. Reset the step pin low
			while (StepTimer::GetTimerTicks() - lastStepPulseTime < GetSlowDriverStepHighClocks()) {}
			StepDriversLow();													// set all step pins low
			lastStepLowTime = StepTimer::GetTimerTicks();
#  endif
		}
		else
# endif
		{
# if USE_TC_FOR_STEP
			if (dms[0].driversCurrentlyUsed != 0)
			{
				StepGenTc->CTRLBSET.reg = TC_CTRLBSET_CMD_RETRIGGER;
				PrepareForNextSteps(now);
			}
# else
			StepDriversHigh(dms[0].driversCurrentlyUsed);						// generate the step
			PrepareForNextSteps(now);
			StepDriversLow();													// set the step pin low
# endif
		}

		if (dms[0].directionChanged)
		{
			dms[0].directionChanged = false;
			SetDirection(dms[0].direction);
		}
	}
}

#else

// This is called by the interrupt service routine to execute steps.
// It returns true if it needs to be called again on the DDA of the new current move, otherwise false.
// This must be as fast as possible, because it determines the maximum movement speed.
// This may occasionally get called prematurely, so it must check that a step is actually due before generating one.
void Move::StepDrivers(uint32_t now) noexcept
{
	uint32_t driversStepping = 0;
	DriveMovement* dm = activeDMs;
	while (dm != nullptr && (int32_t)(dm->nextStepTime - now) <= (int32_t)MoveTiming::MinInterruptInterval)		// if the next step is due
	{
		driversStepping |= dm->driversCurrentlyUsed;
		dm = dm->nextDM;
	}

# if SUPPORT_SLOW_DRIVERS
	if ((driversStepping & slowDriversBitmap != 0)					// if using any slow drivers
	{
		uint32_t lastStepPulseTime = lastStepLowTime;
		uint32_t rawNow;
		do
		{
			rawNow = StepTimer::GetTimerTicks();
		} while (rawNow - lastStepPulseTime < GetSlowDriverStepLowClocks() || rawNow - lastDirChangeTime < GetSlowDriverDirSetupClocks());
		StepDriversHigh(driversStepping);							// set the step pins high
		lastStepPulseTime = StepTimer::GetTimerTicks();

		PrepareForNextSteps(dm, now);

		while (StepTimer::GetTimerTicks() - lastStepPulseTime < GetSlowDriverStepHighClocks()) {}
		StepDriversLow();											// set all step pins low
		lastStepLowTime = StepTimer::GetTimerTicks();
	}
	else
# endif
	{
		StepDriversHigh(driversStepping);							// set the step pins high
# if SAME70
		__DSB();													// without this the step pulse can be far too short
# endif
		PrepareForNextSteps(dm, now);
		StepDriversLow();											// set all step pins low
	}

	// Remove those drives from the list, update the direction pins where necessary, and re-insert them so as to keep the list in step-time order.
	DriveMovement *dmToInsert = activeDMs;							// head of the chain we need to re-insert
	activeDMs = dm;													// remove the chain from the list
	while (dmToInsert != dm)										// note that both of these may be nullptr
	{
		DriveMovement * const nextToInsert = dmToInsert->nextDM;
		if (dmToInsert->state >= DMState::firstMotionState)
		{
			if (dmToInsert->directionChanged)
			{
				dmToInsert->directionChanged = false;
				SetDirection(dmToInsert->drive, dmToInsert->direction);
			}
			InsertDM(dmToInsert);
		}
		dmToInsert = nextToInsert;
	}
}

#endif

// Prepare each DM that we generated a step for for the next step
#if SINGLE_DRIVER

#if SAMC21 || RP2040
__attribute__((section(".time_critical")))
#endif
void Move::PrepareForNextSteps(uint32_t now) noexcept
{
	if (unlikely(dms[0].state == DMState::starting))
	{
		if (dms[0].NewSegment(now) != nullptr && dms[0].state != DMState::starting)
		{
			dms[0].driversCurrentlyUsed = dms[0].driversNormallyUsed;	// we previously set driversCurrentlyUsed to 0 to avoid generating a step, so restore it now
			(void)dms[0].CalcNextStepTimeFull(now);					// calculate next step time
			dms[0].directionChanged = true;							// force the direction to be set up
		}
	}
	else
	{
		(void)dms[0].CalcNextStepTime(now);							// calculate next step time, which may change the required direction
	}
}

#else

void Move::PrepareForNextSteps(DriveMovement *stopDm, uint32_t now) noexcept
{
	for (DriveMovement *dm2 = activeDMs; dm2 != stopDm; dm2 = dm2->nextDM)
	{
		if (unlikely(dm2->state == DMState::starting))
		{
			if (dm2->NewSegment(now) != nullptr && dm2->state != DMState::starting)
			{
				dm2->driversCurrentlyUsed = dm2->driversNormallyUsed;	// we previously set driversCurrentlyUsed to 0 to avoid generating a step, so restore it now
				(void)dm2->CalcNextStepTimeFull(now);					// calculate next step time
				dm2->directionChanged = true;							// force the direction to be set up
			}
		}
		else
		{
			(void)dm2->CalcNextStepTime(now);							// calculate next step time, which may change the required direction
		}
	}
}

#endif

// Stop some drivers and update the corresponding motor positions
void Move::StopDrivers(uint16_t whichDrives) noexcept
{
	DriversBitmap dr(whichDrives);
	dr.Iterate([this](size_t drive, unsigned int)
				{
					AtomicCriticalSectionLocker lock;
					dms[drive].StopDriverFromRemote();
#if !SINGLE_DRIVER
					DeactivateDM(&dms[drive]);
#endif
				}
			  );
}

#if !SINGLE_DRIVER

// Remove this drive from the list of drives with steps due and put it in the completed list
// Called with interrupts disabled.
void Move::DeactivateDM(DriveMovement *dmToRemove) noexcept
{
	DriveMovement **dmp = &activeDMs;
	while (*dmp != nullptr)
	{
		DriveMovement * const dm = *dmp;
		if (dm == dmToRemove)
		{
			(*dmp) = dm->nextDM;
			dm->state = DMState::idle;
			break;
		}
		dmp = &(dm->nextDM);
	}
}

#endif

// Add some linear segments to be executed by a driver, taking account of possible input shaping. This is used by linear axes and by extruders.
// We never add a segment that starts earlier than any existing segments, but we may add segments when there are none already.
void Move::AddLinearSegments(size_t drive, uint32_t startTime, const PrepParams& params, motioncalc_t steps, MovementFlags moveFlags) noexcept
{
	EnableDrive(drive);

	DriveMovement& dmp = dms[drive];
	const motioncalc_t stepsPerMm = steps/(motioncalc_t)1.0;

	const uint32_t steadyStartTime = startTime + params.accelClocks;
	const uint32_t decelStartTime = steadyStartTime + params.steadyClocks;

	// Phases with zero duration will not get executed and may lead to infinities in the calculations. Avoid introducing them. Keep the total distance correct.
	const motioncalc_t accelDistance = (params.accelClocks == 0) ? (motioncalc_t)0.0 : (motioncalc_t)params.accelDistance;
	const motioncalc_t decelDistance = (params.decelClocks == 0) ? (motioncalc_t)0.0 : (motioncalc_t)(1.0 - params.decelStartDistance);
	const motioncalc_t steadyDistance = (params.steadyClocks == 0) ? (motioncalc_t)0.0 : (motioncalc_t)1.0 - accelDistance - decelDistance;

#if SUPPORT_INPUT_SHAPING
	if (moveFlags.noShaping)
#endif
	{
		if (params.accelClocks != 0)
		{
			dmp.AddSegment(startTime, params.accelClocks, accelDistance * stepsPerMm, (motioncalc_t)params.acceleration * stepsPerMm, moveFlags);
		}
		if (params.steadyClocks != 0)
		{
			dmp.AddSegment(steadyStartTime, params.steadyClocks, steadyDistance * stepsPerMm, (motioncalc_t)0.0, moveFlags);
		}
		if (params.decelClocks != 0)
		{
			dmp.AddSegment(decelStartTime, params.decelClocks, decelDistance * stepsPerMm, -((motioncalc_t)params.deceleration * stepsPerMm), moveFlags);
		}
	}
#if SUPPORT_INPUT_SHAPING
	else
	{
		for (size_t index = 0; index < axisShaper.GetNumImpulses(); ++index)
		{
			const motioncalc_t factor = axisShaper.GetImpulseSize(index) * stepsPerMm;
			const uint32_t delay = axisShaper.GetImpulseDelay(index);
			if (params.accelClocks != 0)
			{
				dmp.AddSegment(startTime + delay, params.accelClocks, accelDistance * factor, (motioncalc_t)params.acceleration * factor, moveFlags);
			}
			if (params.steadyClocks != 0)
			{
				dmp.AddSegment(steadyStartTime + delay, params.steadyClocks, steadyDistance * factor, (motioncalc_t)0.0, moveFlags);
			}
			if (params.decelClocks != 0)
			{
				dmp.AddSegment(decelStartTime + delay, params.decelClocks, decelDistance * factor, -((motioncalc_t)params.deceleration * factor), moveFlags);
			}
		}
	}
#endif

	// If there were no segments attached to this DM initially, we need to schedule the interrupt for the new segment at the start of the list.
	// Don't do this until we have added all the segments for this move, because the first segment we added may have been modified and/or split when we added further segments to implement input shaping
#if SAMC21 || RP2040
	const uint32_t oldFlags = IrqSave();
#else
	const uint32_t oldPrio = ChangeBasePriority(NvicPriorityStep);					// shut out the step interrupt
#endif
	if (dmp.state == DMState::idle)
	{
		dmp.positionAtMoveStart = dmp.currentMotorPosition;							// needed for homing moves, which are always isolated moves
		if (dmp.ScheduleFirstSegment())
		{
			// Always set the direction when starting the first move
			dmp.directionChanged = false;
#if SINGLE_DRIVER
			SetDirection(dmp.direction);
#else
			SetDirection(dmp.drive, dmp.direction);
			InsertDM(&dmp);
			if (activeDMs == &dmp)													// if this is now the first DM in the active list
#endif
			{
				if (ScheduleNextStepInterrupt())
				{
					Interrupt();
				}
			}
		}
	}
#if SAMC21 || RP2040
	IrqRestore(oldFlags);
#else
	RestoreBasePriority(oldPrio);
#endif
}

// Filament monitor support
// Get the accumulated extruder motor steps taken by an extruder since the last call. Used by the filament monitoring code.
// Returns the number of motor steps moves since the last call, and isPrinting is true unless we are currently executing an extruding but non-printing move
int32_t Move::GetAccumulatedExtrusion(size_t driver, bool& isPrinting) noexcept
{
	DriveMovement& dm = dms[driver];
	AtomicCriticalSectionLocker lock;							// we don't want a move to complete and the ISR update the movement accumulators while we are doing this
	const int32_t ret = dm.movementAccumulator;
	const int32_t adjustment = dm.GetNetStepsTaken();
	dm.movementAccumulator = -adjustment;
	isPrinting = dms[driver].extruderPrinting;
	return ret + adjustment;
}

// Return when we started doing normal moves after the most recent extruder-only move, in millisecond ticks
uint32_t Move::ExtruderPrintingSince(size_t logicalDrive) const noexcept
{
	return dms[logicalDrive].extruderPrintingSince;
}

// This is the function that is called by the timer interrupt to step the motors. It is also called form Move::Spin() if the first step for a move is due immediately.
// This may occasionally get called prematurely.
#if SAMC21 || RP2040
__attribute__((section(".time_critical")))
#endif
void Move::Interrupt() noexcept
{
#if SINGLE_DRIVER
	if (dms[0].state >= DMState::firstMotionState)
#else
	if (activeDMs != nullptr)
#endif
	{
		uint32_t now = StepTimer::GetMovementTimerTicks();
		const uint32_t isrStartTime = now;
		for (;;)
		{
			// Generate steps for the current move segments
			StepDrivers(now);									// check endstops if necessary and step the drivers

			// Schedule a callback at the time when the next step is due, and quit unless it is due immediately
			if (!ScheduleNextStepInterrupt())
			{
				break;
			}

			// The next step is due immediately. Check whether we have been in this ISR for too long already and need to take a break
			now = StepTimer::GetMovementTimerTicks();
			const int32_t clocksTaken = (int32_t)(now - isrStartTime);
			if (clocksTaken >= (int32_t)MoveTiming::MaxStepInterruptTime)
			{
				// Force a break by updating the move start time.
				++numHiccups;
				uint32_t hiccupTimeInserted = 0;
				for (uint32_t hiccupTime = MoveTiming::HiccupTime; ; hiccupTime += MoveTiming::HiccupIncrement)
				{
					hiccupTimeInserted += hiccupTime;
					StepTimer::IncreaseMovementDelay(hiccupTime);

					// Reschedule the next step interrupt. This time it should succeed if the hiccup time was long enough.
					if (!ScheduleNextStepInterrupt())
					{
						//TODO tell the main board we are behind schedule
						(void)hiccupTimeInserted;
						return;
					}
					// The hiccup wasn't long enough, so go round the loop again
				}
			}
		}
	}
}

void Move::SetDriveStepsPerMm(size_t drive, float val)
{
	if (drive < NumDrivers)
	{
		stepsPerMm[drive] = val;
	}
}

void Move::SetDirectionValue(size_t drive, bool dVal) noexcept
{
	if (drive < NumDrivers)
	{
#if SUPPORT_CLOSED_LOOP
		// We must prevent the closed loop task fetching the current position while we are changing the direction
		if (directions[drive] != dVal)
		{
			TaskCriticalSectionLocker lock;
			directions[drive] = dVal;
			InvertCurrentMotorSteps(drive);
		}
#else
		directions[drive] = dVal;
#endif
	}
}

bool Move::GetDirectionValue(size_t driver) const noexcept
{
	return (driver >= NumDrivers) || directions[driver];
}

void Move::SetEnableValue(size_t driver, int8_t eVal) noexcept
{
	if (driver < NumDrivers)
	{
		enableValues[driver] = eVal;
# if !HAS_SMART_DRIVERS
		if (driverIsEnabled[driver])
		{
			EnableDrive(driver);
		}
		else
		{
			DisableDrive(driver);
		}
# endif
	}
}

int8_t Move::GetEnableValue(size_t driver) const noexcept
{
	return (driver < NumDrivers) ? enableValues[driver] : 0;
}

void Move::EnableDrive(size_t driver) noexcept
{
	if (driverStates[driver] != DriverStateControl::driverActive)
	{
		motorOffTimers[driver].Stop();
		driverStates[driver] = DriverStateControl::driverActive;

# if HAS_SMART_DRIVERS
		if (driverAtIdleCurrent[driver])
		{
			driverAtIdleCurrent[driver] = false;
#  if SUPPORT_CLOSED_LOOP
			dms[driver].closedLoopControl.ResetError();
#  endif
			UpdateMotorCurrent(driver);
		}
		SmartDrivers::EnableDrive(driver, true);
# else
		if (enableValues[driver] >= 0)
		{
			digitalWrite(EnablePins[driver], enableValues[driver] != 0);
#  if DIFFERENTIAL_STEPPER_OUTPUTS
			digitalWrite(InvertedEnablePins[driver], enableValues[driver] == 0);
#  endif
		}
# endif

		// If the brake is not already energised to disengage it, delay before disengaging it
# if SUPPORT_BRAKE_PWM
#  ifdef M23CL
		if (currentBrakePwm[driver] == 0.0)
#  else
		if (brakePwmPorts[driver].IsValid() && currentBrakePwm[driver] == 0.0)
#  endif
# else
		if (brakePorts[driver].IsValid() && !brakePorts[driver].ReadDigital())
# endif
		{
			if (brakeOffDelays[driver] != 0)
			{
				brakeOffTimers[driver].Start();
			}
			else
			{
				DisengageBrake(driver);
			}
		}
	}
}

void Move::DisableDrive(size_t driver) noexcept
{
	brakeOffTimers[driver].Stop();
	driverStates[driver] = DriverStateControl::driverDisabled;
# if SUPPORT_BRAKE_PWM
#  ifdef M23CL
	if (motorOffDelays[driver] != 0 && currentBrakePwm[driver] != 0.0)
#  else
	if (motorOffDelays[driver] != 0 && brakePwmPorts[driver].IsValid() && currentBrakePwm[driver] != 0.0)
#  endif
# else
	if (motorOffDelays[driver] != 0 && brakePorts[driver].IsValid() && brakePorts[driver].ReadDigital())
# endif
	{
		EngageBrake(driver);
		motorOffTimers[driver].Start();
	}
	else
	{
		EngageBrake(driver);
		InternalDisableDrive(driver);
	}
}

void Move::InternalDisableDrive(size_t driver) noexcept
{
# if HAS_SMART_DRIVERS
	SmartDrivers::EnableDrive(driver, false);
# else
	if (enableValues[driver] >= 0)
	{
		digitalWrite(EnablePins[driver], enableValues[driver] == 0);
#  if DIFFERENTIAL_STEPPER_OUTPUTS
		digitalWrite(InvertedEnablePins[driver], enableValues[driver] != 0);
#  endif
	}
# endif
}

void Move::SetDriverIdle(size_t driver, uint16_t idlePercent) noexcept
{
#if SUPPORT_CLOSED_LOOP
	if (dms[driver].closedLoopControl.OkayToSetDriverIdle())
#endif
	{
		idleCurrentFactor[driver] = (float)idlePercent * 0.01;
		driverStates[driver] = DriverStateControl::driverIdle;
		if (idleCurrentFactor[driver] == 0.0)
		{
			InternalDisableDrive(driver);
		}
#if HAS_SMART_DRIVERS
		else
		{
			driverAtIdleCurrent[driver] = true;
			UpdateMotorCurrent(driver);
		}
#endif
	}
}

void Move::DisableAllDrives() noexcept
{
	for (size_t driver = 0; driver < NumDrivers; ++driver)
	{
#if HAS_SMART_DRIVERS
		SmartDrivers::EnableDrive(driver, false);
#else
		DisableDrive(driver);
#endif
	}
}

#if HAS_SMART_DRIVERS

void Move::UpdateMotorCurrent(size_t driver) noexcept
{
	SmartDrivers::SetCurrent(driver, (driverAtIdleCurrent[driver]) ? motorCurrents[driver] * idleCurrentFactor[driver] : motorCurrents[driver]);
}
void Move::SetMotorCurrent(size_t driver, float current) noexcept
{
	motorCurrents[driver] = current;
	UpdateMotorCurrent(driver);
}

// TMC driver temperatures
float Move::GetTmcDriversTemperature()
{
	const DriversBitmap mask = DriversBitmap::MakeLowestNBits(MaxSmartDrivers);
	return (temperatureShutdownDrivers.Intersects(mask)) ? 150.0
			: (temperatureWarningDrivers.Intersects(mask)) ? 100.0
				: 0.0;
}

# if HAS_STALL_DETECT

void Move::SetOrResetEventOnStall(DriversBitmap drivers, bool enable) noexcept
{
	if (enable)
	{
		eventOnStallDrivers |= drivers;
	}
	else
	{
		eventOnStallDrivers &= ~drivers;
	}
}

bool Move::GetEventOnStall(unsigned int driver) noexcept
{
	return eventOnStallDrivers.IsBitSet(driver);
}

# endif

#else

StandardDriverStatus Move::GetStandardDriverStatus(size_t driver) noexcept
{
	//TODO implement alarm input
	StandardDriverStatus rslt;
	rslt.all = 0;
	return rslt;
}

#endif

#if SUPPORT_CLOSED_LOOP

bool Move::EnableIfIdle(size_t driver) noexcept
{
	if (driverStates[driver] == DriverStateControl::driverIdle)
	{
		driverStates[driver] = DriverStateControl::driverActive;
# if HAS_SMART_DRIVERS
		driverAtIdleCurrent[driver] = false;
		UpdateMotorCurrent(driver);
# endif
	}

	return driverStates[driver] == DriverStateControl::driverActive;
}

#endif

GCodeResult Move::ProcessM569(const CanMessageGeneric& msg, const StringRef& reply) noexcept
{
	CanMessageGenericParser parser(msg, M569Params);
	uint8_t drive;
	if (!parser.GetUintParam('P', drive))
	{
		reply.copy("Missing P parameter in CAN message");
		return GCodeResult::error;
	}

	if (drive >= NumDrivers)
	{
		reply.printf("Driver number %u.%u out of range", CanInterface::GetCanAddress(), drive);
		return GCodeResult::error;
	}

	bool seen = false;
	uint8_t direction;
	if (parser.GetUintParam('S', direction))
	{
		seen = true;
		SetDirectionValue(drive, direction != 0);
	}
	int8_t rValue;
	if (parser.GetIntParam('R', rValue))
	{
		seen = true;
		SetEnableValue(drive, rValue);
	}

#if SUPPORT_SLOW_DRIVERS
	float timings[4];
	size_t numTimings = 4;
	if (parser.GetFloatArrayParam('T', numTimings, timings))
	{
		seen = true;
		if (numTimings == 1)
		{
			timings[1] = timings[2] = timings[3] = timings[0];
		}
		else if (numTimings != 4)
		{
			reply.copy("bad timing parameter, expected 1 or 4 values");
			return GCodeResult::error;
		}
		SetDriverStepTiming(drive, timings);
	}
#endif

#if HAS_SMART_DRIVERS
	{
		uint32_t val;
		if (parser.GetUintParam('D', val))	// set driver mode
		{
			seen = true;
# if SUPPORT_CLOSED_LOOP
			// Enable/disabled closed loop control
			const ClosedLoopMode mode = (val == (uint32_t)DriverMode::direct) ? ClosedLoopMode::closed
										: (val == (uint32_t)DriverMode::direct + 1) ? ClosedLoopMode::assistedOpen
											: ClosedLoopMode::open;
			if (!dms[drive].closedLoopControl.SetClosedLoopEnabled(mode, reply))
			{
				// reply.printf is done in ClosedLoop::SetClosedLoopEnabled()
				return GCodeResult::error;
			}
# endif
			if (!SmartDrivers::SetDriverMode(drive, val))
			{
				reply.printf("Driver %u.%u does not support mode '%s'", CanInterface::GetCanAddress(), drive, TranslateDriverMode(val));
				return GCodeResult::error;
			}
# if SUPPORT_CLOSED_LOOP
			if (mode != ClosedLoopMode::open)
			{
				dms[drive].closedLoopControl.DriverSwitchedToClosedLoop();
			}
# endif
		}

		if (parser.GetUintParam('F', val))		// set off time
		{
			seen = true;
			if (!SmartDrivers::SetRegister(drive, SmartDriverRegister::toff, val))
			{
				reply.printf("Bad off time for driver %u", drive);
				return GCodeResult::error;
			}
		}

		if (parser.GetUintParam('B', val))		// set blanking time
		{
			seen = true;
			if (!SmartDrivers::SetRegister(drive, SmartDriverRegister::tblank, val))
			{
				reply.printf("Bad blanking time for driver %u", drive);
				return GCodeResult::error;
			}
		}

		if (parser.GetUintParam('V', val))		// set microstep interval for changing from stealthChop to spreadCycle
		{
			seen = true;
			if (!SmartDrivers::SetRegister(drive, SmartDriverRegister::tpwmthrs, val))
			{
				reply.printf("Bad mode change microstep interval for driver %u", drive);
				return GCodeResult::error;
			}
		}

#if SUPPORT_TMC51xx || SUPPORT_TMC2160
		if (parser.GetUintParam('H', val))		// set coolStep threshold
		{
			seen = true;
			if (!SmartDrivers::SetRegister(drive, SmartDriverRegister::thigh, val))
			{
				reply.printf("Bad high speed microstep interval for driver %u", drive);
				return GCodeResult::error;
			}
		}
#endif
	}

	size_t numHvalues = 3;
	const uint8_t *hvalues;
	if (parser.GetArrayParam('Y', ParamDescriptor::ParamType::uint8_array, numHvalues, hvalues))		// set spread cycle hysteresis
	{
		seen = true;
		if (numHvalues == 2 || numHvalues == 3)
		{
			// There is a constraint on the sum of HSTRT and HEND, so set HSTART then HEND then HSTART again because one may go up and the other down
			(void)SmartDrivers::SetRegister(drive, SmartDriverRegister::hstart, hvalues[0]);
			bool ok = SmartDrivers::SetRegister(drive, SmartDriverRegister::hend, hvalues[1]);
			if (ok)
			{
				ok = SmartDrivers::SetRegister(drive, SmartDriverRegister::hstart, hvalues[0]);
			}
			if (ok && numHvalues == 3)
			{
				ok = SmartDrivers::SetRegister(drive, SmartDriverRegister::hdec, hvalues[2]);
			}
			if (!ok)
			{
				reply.printf("Bad hysteresis setting for driver %u", drive);
				return GCodeResult::error;
			}
		}
		else
		{
			reply.copy("Expected 2 or 3 Y values");
			return GCodeResult::error;
		}
	}
#endif
	if (!seen)
	{
		reply.printf("Driver %u.%u runs %s, active %s enable",
						CanInterface::GetCanAddress(),
						drive,
						(GetDirectionValue(drive)) ? "forwards" : "in reverse",
						(GetEnableValue(drive)) ? "high" : "low");

#if SUPPORT_SLOW_DRIVERS
# if SINGLE_DRIVER
		if (IsSlowDriver())
# else
		if (IsSlowDriver(drive))
# endif
		{
			reply.catf(", step timing %.1f:%.1f:%.1f:%.1fus",
						(double)GetSlowDriverStepHighMicroseconds(),
						(double)GetSlowDriverStepLowMicroseconds(),
						(double)GetSlowDriverDirSetupMicroseconds(),
						(double)GetSlowDriverDirHoldMicroseconds());
		}
		else
		{
			reply.cat(", step timing fast");
		}
#endif

#if HAS_SMART_DRIVERS
		// It's a smart driver, so print the parameters common to all modes, except for the position
		const DriverMode dmode = SmartDrivers::GetDriverMode(drive);
		reply.catf(", mode %s", TranslateDriverMode(dmode));
# if SUPPORT_CLOSED_LOOP
		if (dmode == DriverMode::direct)
		{
			reply.catf(" (%s)", dms[drive].closedLoopControl.GetModeText());
		}
# endif
		reply.catf(", ccr 0x%05" PRIx32 ", toff %" PRIu32 ", tblank %" PRIu32,
					SmartDrivers::GetRegister(drive, SmartDriverRegister::chopperControl),
					SmartDrivers::GetRegister(drive, SmartDriverRegister::toff),
					SmartDrivers::GetRegister(drive, SmartDriverRegister::tblank)
				  );

# if SUPPORT_TMC51xx || SUPPORT_TMC2160
		{
			const uint32_t thigh = SmartDrivers::GetRegister(drive, SmartDriverRegister::thigh);
			bool bdummy;
			const float mmPerSec = (12000000.0 * SmartDrivers::GetMicrostepping(drive, bdummy))/(256 * thigh * DriveStepsPerMm(drive));
			reply.catf(", thigh %" PRIu32 " (%.1f mm/sec)", thigh, (double)mmPerSec);
		}
# endif

		// Print the additional parameters that are relevant in the current mode
		if (SmartDrivers::GetDriverMode(drive) == DriverMode::spreadCycle)
		{
			reply.catf(", hstart/hend/hdec %" PRIu32 "/%" PRIu32 "/%" PRIu32,
						SmartDrivers::GetRegister(drive, SmartDriverRegister::hstart),
						SmartDrivers::GetRegister(drive, SmartDriverRegister::hend),
						SmartDrivers::GetRegister(drive, SmartDriverRegister::hdec)
					  );
		}

# if SUPPORT_TMC22xx || SUPPORT_TMC51xx || SUPPORT_TMC2160
		if (SmartDrivers::GetDriverMode(drive) == DriverMode::stealthChop)
		{
			const uint32_t tpwmthrs = SmartDrivers::GetRegister(drive, SmartDriverRegister::tpwmthrs);
			bool bdummy;
			const float mmPerSec = (12000000.0 * SmartDrivers::GetMicrostepping(drive, bdummy))/(256 * tpwmthrs * DriveStepsPerMm(drive));
			const uint32_t pwmScale = SmartDrivers::GetRegister(drive, SmartDriverRegister::pwmScale);
			const uint32_t pwmAuto = SmartDrivers::GetRegister(drive, SmartDriverRegister::pwmAuto);
			const unsigned int pwmScaleSum = pwmScale & 0xFF;
			const int pwmScaleAuto = (int)((((pwmScale >> 16) & 0x01FF) ^ 0x0100) - 0x0100);
			const unsigned int pwmOfsAuto = pwmAuto & 0xFF;
			const unsigned int pwmGradAuto = (pwmAuto >> 16) & 0xFF;
			reply.catf(", tpwmthrs %" PRIu32 " (%.1f mm/sec), pwmScaleSum %u, pwmScaleAuto %d, pwmOfsAuto %u, pwmGradAuto %u",
						tpwmthrs, (double)mmPerSec, pwmScaleSum, pwmScaleAuto, pwmOfsAuto, pwmGradAuto);
		}
# endif
		// Finally, print the microstep position
		{
			const uint32_t mstepPos = SmartDrivers::GetRegister(drive, SmartDriverRegister::mstepPos);
			if (mstepPos < 1024)
			{
				reply.catf(", pos %" PRIu32, mstepPos);
			}
			else
			{
				reply.cat(", pos unknown");
			}
		}
#endif

	}
	return GCodeResult::ok;
}

GCodeResult Move::ProcessM569Point7(const CanMessageGeneric& msg, const StringRef& reply) noexcept
{
# ifdef M23CL
	reply.copy("Not supported by this board");
	return GCodeResult::error;
# else
	CanMessageGenericParser parser(msg, M569Point7Params);
	uint8_t drive;
	if (!parser.GetUintParam('P', drive))
	{
		reply.copy("Missing P parameter in CAN message");
		return GCodeResult::error;
	}

	if (drive >= NumDrivers)
	{
		reply.printf("Driver number %u.%u out of range", CanInterface::GetCanAddress(), drive);
		return GCodeResult::error;
	}

# if !SUPPORT_BRAKE_PWM
	if (parser.HasParameter('V'))
	{
		// Don't allow a brake port to be configured if the user specified a voltage and we don't support PWM
		reply.copy("Brake PWM not supported by this board");
		return GCodeResult::error;
	}
# endif

	bool seen = false;
	if (parser.HasParameter('C'))
	{
		String<StringLength20> portName;
		parser.GetStringParam('C', portName.GetRef());
# if SUPPORT_BRAKE_PWM
		if (!brakePwmPorts[drive].AssignPort(
# else
		if (!brakePorts[drive].AssignPort(
# endif
											portName.c_str(), reply, PinUsedBy::gpout,
											(driverStates[drive] == DriverStateControl::driverDisabled) ? PinAccess::write0 : PinAccess::write1
										 )
		   )
		{
			return GCodeResult::error;
		}
		seen = true;
		motorOffDelays[drive] = brakeOffDelays[drive] = DefaultDelayAfterBrakeOn;

# if SUPPORT_BRAKE_PWM
		brakePwmPorts[drive].SetFrequency(BrakePwmFrequency);
#  if defined(EXP1HCL)
		if (Platform::GetBoardVariant() >= 1)
		{
			brakeOnPins[drive] = (brakePwmPorts[drive].GetPin() == BrakePwmPin) ? BrakeOnPin : NoPin;
		}
#  endif
# endif
	}

	{
		uint16_t brakeDelay;
		if (parser.GetUintParam('S', brakeDelay))
		{
			seen = true;
			motorOffDelays[drive] = brakeOffDelays[drive] = brakeDelay;
		}
	}

# if SUPPORT_BRAKE_PWM
	if (parser.GetFloatParam('V', brakeVoltages[drive]))
	{
		seen = true;
	}
# endif

	if (!seen)
	{
		reply.printf("Driver %u.%u uses brake port ", CanInterface::GetCanAddress(), drive);
# if SUPPORT_BRAKE_PWM
		brakePwmPorts[drive].AppendPinName(reply);
		if (brakeVoltages[drive] < FullyOnBrakeVoltage)
		{
			reply.catf(" with voltage limited to %.1f by PWM", (double)brakeVoltages[drive]);
		}
# else
		brakePorts[drive].AppendPinName(reply);
# endif
		reply.catf(", brake delay %ums", motorOffDelays[drive]);
	}
	return GCodeResult::ok;
#endif
}

GCodeResult Move::SetMotorCurrents(const CanMessageMultipleDrivesRequest<float>& msg, size_t dataLength, const StringRef& reply) noexcept
{
# if HAS_SMART_DRIVERS
	const auto drivers = Bitmap<uint16_t>::MakeFromRaw(msg.driversToUpdate);
	if (dataLength < msg.GetActualDataLength(drivers.CountSetBits()))
	{
		reply.copy("bad data length");
		return GCodeResult::error;
	}

	GCodeResult rslt = GCodeResult::ok;
	drivers.Iterate([this, &msg, &reply, &rslt](unsigned int driver, unsigned int count) -> void
						{
							if (driver >= NumDrivers)
							{
								reply.lcatf("No such driver %u.%u", CanInterface::GetCanAddress(), driver);
								rslt = GCodeResult::error;
							}
							else
							{
								SetMotorCurrent(driver, msg.values[count]);
#if SUPPORT_CLOSED_LOOP
								dms[driver].closedLoopControl.UpdateStandstillCurrent();
#endif
							}
						}
				   );
	return rslt;
# else
	reply.copy("Setting not available for external drivers");
	return GCodeResult::error;
# endif
}

GCodeResult Move::SetStandstillCurrentFactor(const CanMessageMultipleDrivesRequest<float>& msg, size_t dataLength, const StringRef& reply) noexcept
{
# if HAS_SMART_DRIVERS
	const auto drivers = Bitmap<uint16_t>::MakeFromRaw(msg.driversToUpdate);
	if (dataLength < msg.GetActualDataLength(drivers.CountSetBits()))
	{
		reply.copy("bad data length");
		return GCodeResult::error;
	}

	GCodeResult rslt = GCodeResult::ok;
	drivers.Iterate([this, &msg, &reply, &rslt](unsigned int driver, unsigned int count) -> void
						{
							if (driver >= NumDrivers)
							{
								reply.lcatf("No such driver %u.%u", CanInterface::GetCanAddress(), driver);
								rslt = GCodeResult::error;
							}
							else
							{
								SmartDrivers::SetStandstillCurrentPercent(driver, msg.values[count]);
#if SUPPORT_CLOSED_LOOP
								dms[driver].closedLoopControl.UpdateStandstillCurrent();
#endif
							}
						}
				   );
	return rslt;
# else
	reply.copy("Setting not available for external drivers");
	return GCodeResult::error;
# endif
}

// Turn the brake solenoid on to disengage the brake
void Move::DisengageBrake(size_t driver) noexcept
{
#if SUPPORT_BRAKE_PWM
	// Set the PWM to deliver the requested voltage regardless of the VIN voltage
	const float requestedVoltage =
# ifdef M23CL
									M23CLBrakeVoltage;
# else
									brakeVoltages[driver];
# endif
	const float currentVinVoltage = Platform::GetCurrentVinVoltage();
	currentBrakePwm[driver] = (currentVinVoltage < requestedVoltage) ? 1.0 : requestedVoltage/currentVinVoltage;
# ifdef M23CL
	IoPort::WriteAnalog(BrakePwmPin, currentBrakePwm[driver], BrakePwmFrequency);
	digitalWrite(BrakeOnPin, true);
# else
	brakePwmPorts[driver].WriteAnalog(currentBrakePwm[driver]);
	digitalWrite(brakeOnPins[driver], true);
# endif
#else
	brakePorts[driver].WriteDigital(true);
#endif
}

// Turn the brake solenoid off to re-engage the brake
void Move::EngageBrake(size_t driver) noexcept
{
#if SUPPORT_BRAKE_PWM
	currentBrakePwm[driver] = 0.0;
# ifdef M23CL
	IoPort::WriteAnalog(BrakePwmPin, 0.0, BrakePwmFrequency);
	digitalWrite(BrakeOnPin, false);
# else
	brakePwmPorts[driver].WriteAnalog(0.0);
	digitalWrite(brakeOnPins[driver], false);
# endif
#else
	brakePorts[driver].WriteDigital(false);
#endif
}

#if HAS_SMART_DRIVERS

StandardDriverStatus Move::GetDriverStatus(size_t driver, bool accumulated, bool clearAccumulated) const noexcept
{
	StandardDriverStatus stat = SmartDrivers::GetStatus(driver, accumulated, clearAccumulated);
# if SUPPORT_CLOSED_LOOP
	stat = dms[driver].closedLoopControl.ModifyDriverStatus(stat);
# endif
	return stat;
}

#endif

// Function to broadcast the drivers status message. Called only by the Heat task.
void Move::SendDriversStatus(CanMessageBuffer& buf) noexcept
{
	CanMessageDriversStatus * const msg = buf.SetupStatusMessage<CanMessageDriversStatus>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress());
# if SUPPORT_CLOSED_LOOP
	msg->SetStandardFields(NumDrivers, true);
	for (size_t driver = 0; driver < NumDrivers; ++driver)
	{
		msg->closedLoopData[driver].status = GetDriverStatus(driver, false, false).AsU32();
		dms[driver].closedLoopControl.GetStatistics(msg->closedLoopData[driver]);
	}
# elif HAS_SMART_DRIVERS
	// We currently assume that all drivers on this board are smart drivers. This isn't true on a Duet 3 Mini with external drivers connected to the expansion slot.
	msg->SetStandardFields(MaxSmartDrivers, false);
	for (size_t driver = 0; driver < MaxSmartDrivers; ++driver)
	{
		msg->openLoopData[driver].status = SmartDrivers::GetStatus(driver, false, false).AsU32();
	}
# else
	msg->SetStandardFields(NumDrivers, false);
	for (size_t driver = 0; driver < NumDrivers; ++driver)
	{
		msg->openLoopData[driver].status = GetStandardDriverStatus(driver).AsU32();
	}
# endif
	buf.dataLength = msg->GetActualDataLength();
	CanInterface::Send(&buf);
}

#if SUPPORT_SLOW_DRIVERS

static inline void UpdateTiming(uint32_t& timing, uint32_t clocks) noexcept
{
# if SINGLE_DRIVER
		timing = clocks;
# else
		if (clocks > timing)
		{
			timing = clocks;
		}
# endif
}

void Move::SetDriverStepTiming(size_t drive, const float timings[4]) noexcept
{
	bool isSlow = false;

# if USE_TC_FOR_STEP

	// Step high time - must do this one first because it affects the conversion of some of the others
	if (timings[0] > MinimumStepHighMicroseconds)
	{
		isSlow = true;
		UpdateTiming(slowDriverStepTimingClocks[0], MicrosecondsToStepTCClocks(timings[0]));
	}
#  if SINGLE_DRIVER		// we can clear the value if we have only one driver
	else
	{
		slowDriverStepTimingClocks[0] = MicrosecondsToStepTCClocks(MinimumStepHighMicroseconds);
	}
#  endif

	// To get the new width to be applied to the step pulse, we need to update CCBUF[0] and then push it to CC[0]. Writing CC[0] directly doesn't work.
	StepGenTc->CCBUF[0].reg = (uint16_t)slowDriverStepTimingClocks[0];
	StepGenTc->CTRLBSET.reg = TC_CTRLBSET_CMD_UPDATE;

	// Step low time - must convert this to minimum period
	const float minimumPeriod = timings[1] + GetSlowDriverStepHighMicroseconds();		// use the actual rounded-up value
	if (minimumPeriod > 0.4)
	{
		isSlow = true;
		UpdateTiming(slowDriverStepTimingClocks[1], MicrosecondsToStepClocks(minimumPeriod));
	}
#  if SINGLE_DRIVER		// we can clear the value if we have only one driver
	else
	{
		slowDriverStepTimingClocks[1] = 1;
	}
#  endif

	// Direction setup time - we can just convert this one
	if (timings[2] > 0.2)
	{
		isSlow = true;
		UpdateTiming(slowDriverStepTimingClocks[2], MicrosecondsToStepClocks(timings[2]));
	}
#  if SINGLE_DRIVER		// we can clear the value if we have only one driver
	else
	{
		slowDriverStepTimingClocks[2] = 0;
	}
#  endif

	// Direction hold time - we need to convert hold time from trailing edge to hold time from leading edge
	const float holdTimeFromLeadingEdge = timings[3] + GetSlowDriverStepHighMicroseconds();		// use the actual rounded-up value
	if (holdTimeFromLeadingEdge > 0.4)
	{
		isSlow = true;
		const uint32_t clocks = MicrosecondsToStepClocks(holdTimeFromLeadingEdge);
		UpdateTiming(slowDriverStepTimingClocks[3], clocks);
	}
#  if SINGLE_DRIVER		// we can clear the value if we have only one driver
	else
	{
		slowDriverStepTimingClocks[3] = 1;
	}
#  endif

# else

	// Not using TC to generate step pulses
	for (size_t i = 0; i < 4; ++i)
	{
		if (timings[i] > 0.2)
		{
			isSlow = true;
			const uint32_t clocks = MicrosecondsToStepClocks(timings[i]);
			UpdateTiming(slowDriverStepTimingClocks[i], clocks);
		}
#  if SINGLE_DRIVER		// we can clear the value if we have only one driver
		else
		{
			slowDriverStepTimingClocks[i] = 0;
		}
#  endif
	}

# endif	// USE_TC_FOR_STEP

# if SINGLE_DRIVER
	isSlowDriver = isSlow;
# else
	slowDriversBitmap.SetOrClearBit(drive, isSlow);
# endif
}

float Move::GetSlowDriverStepHighMicroseconds() const noexcept
{
# if USE_TC_FOR_STEP
	return (float)slowDriverStepTimingClocks[0]/StepPulseClocksPerMicrosecond;
# else
	return StepTimer::TicksToFloatMicroseconds(slowDriverStepTimingClocks[0]);
# endif
}

float Move::GetSlowDriverStepLowMicroseconds() const noexcept
{
# if USE_TC_FOR_STEP
	const float period = StepTimer::TicksToFloatMicroseconds(slowDriverStepTimingClocks[1]);
	return period - GetSlowDriverStepHighMicroseconds();
# else
	return StepTimer::TicksToFloatMicroseconds(slowDriverStepTimingClocks[1]);
# endif
}

float Move::GetSlowDriverDirSetupMicroseconds() const noexcept
{
	return StepTimer::TicksToFloatMicroseconds(slowDriverStepTimingClocks[2]);
}

float Move::GetSlowDriverDirHoldMicroseconds() const noexcept
{
# if USE_TC_FOR_STEP
	const float dirHoldFromLeadingEdge = StepTimer::TicksToFloatMicroseconds(slowDriverStepTimingClocks[3]);
	return dirHoldFromLeadingEdge - GetSlowDriverStepHighMicroseconds();
# else
	return StepTimer::TicksToFloatMicroseconds(slowDriverStepTimingClocks[3]);
# endif
}

#endif		// SUPPORT_SLOW_DRIVERS

#if HAS_SMART_DRIVERS

bool Move::SetMicrostepping(size_t driver, unsigned int microsteps, bool interpolate) noexcept
{
	return SmartDrivers::SetMicrostepping(driver, microsteps, interpolate);
}

#endif

#if SUPPORT_CLOSED_LOOP

GCodeResult Move::ProcessM569Point1(const CanMessageGeneric &msg, const StringRef &reply) noexcept
{
	CanMessageGenericParser parser(msg, M569Point1Params);
	uint8_t drive;
	if (!parser.GetUintParam('P', drive))
	{
		reply.copy("missing P parameter in CAN message");
		return GCodeResult::error;
	}
	if (drive >= NumDrivers)
	{
		reply.copy("no such driver");
		return GCodeResult::error;
	}
	return dms[drive].closedLoopControl.ProcessM569Point1(parser, reply);
}

// M569.4 Set torque mode
GCodeResult Move::ProcessM569Point4(const CanMessageGeneric& msg, const StringRef& reply) noexcept
{
	CanMessageGenericParser parser(msg, M569Point4Params);
	uint8_t drive;
	if (!parser.GetUintParam('P', drive))
	{
		reply.copy("missing P parameter in CAN message");
		return GCodeResult::error;
	}
	if (drive >= NumDrivers)
	{
		reply.copy("no such driver");
		return GCodeResult::error;
	}
	return dms[drive].closedLoopControl.ProcessM569Point4(parser, reply);
}

GCodeResult Move::ProcessM569Point5(const CanMessageStartClosedLoopDataCollection& msg, const StringRef& reply) noexcept
{
	if (msg.deviceNumber >= NumDrivers)
	{
		reply.copy("no such driver");
		return GCodeResult::error;
	}
	return dms[msg.deviceNumber].closedLoopControl.ProcessM569Point5(msg, reply);
}

GCodeResult Move::ProcessM569Point6(const CanMessageGeneric &msg, const StringRef &reply) noexcept
{
	CanMessageGenericParser parser(msg, M569Point6Params);
	uint8_t drive;
	if (!parser.GetUintParam('P', drive))
	{
		reply.copy("missing P parameter in CAN message");
		return GCodeResult::error;
	}
	if (drive >= NumDrivers)
	{
		reply.copy("no such driver");
		return GCodeResult::error;
	}
	return dms[drive].closedLoopControl.ProcessM569Point6(parser, reply);
}

void Move::ClosedLoopControlLoop() noexcept
{
	for (DriveMovement& dm : dms)
	{
		dm.closedLoopControl.InstanceControlLoop();
	}
}

void Move::ClosedLoopDiagnostics(size_t driver, const StringRef& reply) noexcept
{
	dms[driver].closedLoopControl.InstanceDiagnostics(driver, reply);
}

#endif

#endif	//SUPPORT_DRIVERS

// End
