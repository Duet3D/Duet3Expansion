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

class Encoder;
class SpiEncoder;

namespace ClosedLoop
{
	// Constants and variables that are used by both the ClosedLoop and the Tuning modules

	// Possible tuning errors
	constexpr uint8_t TUNE_ERR_NOT_DONE_BASIC				= 1u << 0;
	constexpr uint8_t TUNE_ERR_NOT_CALIBRATED				= 1u << 1;
	constexpr uint8_t TUNE_ERR_TOO_LITTLE_MOTION			= 1u << 2;
	constexpr uint8_t TUNE_ERR_TOO_MUCH_MOTION				= 1u << 3;
	constexpr uint8_t TUNE_ERR_INCONSISTENT_MOTION			= 1u << 4;
	constexpr uint8_t TUNE_ERR_SYSTEM_ERROR					= 1u << 5;

	constexpr uint8_t TUNE_ERR_TUNING_FAILURE 				= TUNE_ERR_NOT_CALIBRATED | TUNE_ERR_SYSTEM_ERROR
															| TUNE_ERR_TOO_LITTLE_MOTION | TUNE_ERR_TOO_MUCH_MOTION | TUNE_ERR_INCONSISTENT_MOTION;

	// Tuning manoeuvres
	constexpr uint8_t BASIC_TUNING_MANOEUVRE 				= 1u << 0;		// this measures the polarity, check that the CPR looks OK, and for relative encoders sets the zero position
	constexpr uint8_t ENCODER_CALIBRATION_MANOEUVRE 		= 1u << 1;		// this calibrates an absolute encoder
	constexpr uint8_t STEP_MANOEUVRE 						= 1u << 6;		// this does a sudden step change in the requested position for PID tuning

#if 0	// The remainder are not currently implemented
	constexpr uint8_t CONTINUOUS_PHASE_INCREASE_MANOEUVRE 	= 1u << 5;
	constexpr uint8_t ZIEGLER_NICHOLS_MANOEUVRE 			= 1u << 7;
#endif

	//TODO reduce the number of these public variables, preferably to zero. Use a cleaner interface between the tuning module and the main closed loop module.
	extern Encoder *encoder;						// Pointer to the encoder object in use
	extern uint8_t tuning;							// Bitmask of any tuning manoeuvres that have been requested
	extern uint8_t tuningError;						// Flags for any tuning errors
	extern uint16_t measuredStepPhase;				// The measured position of the motor
	extern uint16_t desiredStepPhase;				// The desired position of the motor
	extern int32_t currentEncoderReading;			// The latest reading taken from the encoder
	extern float encoderPulsePerStep;				// How many encoder readings do we get per step?

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
	float PulsePerStepToExternalUnits(float pps, uint8_t encoderType) noexcept;
	float ExternalUnitsToPulsePerStep(float externalUnits, uint8_t encoderType) noexcept;
	void SetMotorPhase(uint16_t phase, float magnitude) noexcept;
	void SetForwardPolarity() noexcept;
	void SaveBasicTuningResult(float slope, float origin, float xMean, bool reverse) noexcept;
	void FinishedBasicTuning() noexcept;			// call this when we have stopped basic tuning movement and are ready to switch to closed loop control
	void AdjustTargetMotorSteps(float amount) noexcept;	// called by tuning to execute a step

	// Methods in the tuning module
	void PerformTune() noexcept;
}

#  if defined(EXP1HCLv0_3) || defined(EXP1HCLv1_0)
// The encoder uses the standard shared SPI device, so we don't need to enable/disable it
inline void ClosedLoop::EnableEncodersSpi() noexcept { }
inline void ClosedLoop::DisableEncodersSpi() noexcept { }
#  endif

# endif

#endif /* SRC_CLOSEDLOOP_CLOSEDLOOP_H_ */
