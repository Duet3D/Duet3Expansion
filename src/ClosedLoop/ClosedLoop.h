/*
 * ClosedLoop.h
 *
 *  Created on: 9 Jun 2020
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_CLOSEDLOOP_H_
#define SRC_CLOSEDLOOP_CLOSEDLOOP_H_

#include <RepRapFirmware.h>

#if SUPPORT_CLOSED_LOOP

# include <CanMessageFormats.h>
# include <General/NamedEnum.h>
# include <Movement/StepTimer.h>
# include <ClosedLoop/Trigonometry.h>
# include <Hardware/SharedSpiDevice.h>
# include <ClosedLoop/DerivativeAveragingFilter.h>
# include "TuningErrors.h"

class Encoder;
class SpiEncoder;

namespace ClosedLoop
{
	// Constants and variables that are used by both the ClosedLoop and the Tuning modules

	// Tuning manoeuvres
	constexpr uint8_t BASIC_TUNING_MANOEUVRE 				= 1u << 0;		// this measures the polarity, check that the CPR looks OK, and for relative encoders sets the zero position
	constexpr uint8_t ENCODER_CALIBRATION_MANOEUVRE 		= 1u << 1;		// this calibrates an absolute encoder
	constexpr uint8_t ENCODER_CALIBRATION_CHECK				= 1u << 2;		// this checks the calibration
	constexpr uint8_t STEP_MANOEUVRE 						= 1u << 6;		// this does a sudden step change in the requested position for PID tuning

#if 0	// The remainder are not currently implemented
	constexpr uint8_t CONTINUOUS_PHASE_INCREASE_MANOEUVRE 	= 1u << 5;
	constexpr uint8_t ZIEGLER_NICHOLS_MANOEUVRE 			= 1u << 7;
#endif

	//TODO reduce the number of these public variables, preferably to zero. Use a cleaner interface between the tuning module and the main closed loop module.
	extern Encoder *encoder;								// Pointer to the encoder object in use
	extern volatile uint8_t tuning;							// Bitmask of any tuning manoeuvres that have been requested
	extern TuningErrors tuningError;						// Flags for any tuning errors
	extern uint32_t currentMotorPhase;						// the phase (0 to 4095) that the driver is set to
	extern unsigned int basicTuningIterationMultiplier;		// we multiply the number of steps we do by this constant, default 1

	// Closed loop public methods
	void Init() noexcept;

	GCodeResult ProcessM569Point1(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	GCodeResult ProcessM569Point5(const CanMessageStartClosedLoopDataCollection&, const StringRef&) noexcept;
	GCodeResult ProcessM569Point6(const CanMessageGeneric& msg, const StringRef& reply) noexcept;

	void Diagnostics(const StringRef& reply) noexcept;

	// Methods called by the motion system
	void ControlLoop() noexcept;
	void TakeStep() noexcept;
	StandardDriverStatus ReadLiveStatus() noexcept;
	void SetStepDirection(bool) noexcept;
	bool GetClosedLoopEnabled() noexcept;
	bool SetClosedLoopEnabled(size_t driver, bool enabled, const StringRef &reply) noexcept;
	void DriverSwitchedToClosedLoop(size_t driver) noexcept;
	void ResetError(size_t driver) noexcept;
	StandardDriverStatus ModifyDriverStatus(size_t driver, StandardDriverStatus originalStatus) noexcept;

	// Methods called by the encoders
	void EnableEncodersSpi() noexcept;
	void DisableEncodersSpi() noexcept;

	// Methods used only by closed loop and by the tuning module
	void SetMotorPhase(uint16_t phase, float magnitude) noexcept;
	void FinishedBasicTuning() noexcept;
															// call this when we have stopped basic tuning movement and are ready to switch to closed loop control
	void ReadyToCalibrate(bool store) noexcept;				// call this when encoder calibration has finished collecting data
	void AdjustTargetMotorSteps(float amount) noexcept;		// called by tuning to execute a step

	// Methods in the tuning module
	void PerformTune() noexcept;
}

#  if defined(EXP1HCLv1_0) || defined(M23CL)
// The encoder uses the standard shared SPI device, so we don't need to enable/disable it
inline void ClosedLoop::EnableEncodersSpi() noexcept { }
inline void ClosedLoop::DisableEncodersSpi() noexcept { }
#  endif

# endif

#endif /* SRC_CLOSEDLOOP_CLOSEDLOOP_H_ */
