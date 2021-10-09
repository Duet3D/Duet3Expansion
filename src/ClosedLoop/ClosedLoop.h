/*
 * ClosedLoop.h
 *
 *  Created on: 9 Jun 2020
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_CLOSEDLOOP_H_
# define SRC_CLOSEDLOOP_CLOSEDLOOP_H_

# include <RepRapFirmware.h>

# if SUPPORT_CLOSED_LOOP

#  include <CanMessageFormats.h>
#  include <General/NamedEnum.h>
#  include <GCodes/GCodeResult.h>
#  include <Movement/StepTimer.h>
#  include <ClosedLoop/Trigonometry.h>
#  include <Hardware/SharedSpiDevice.h>
#  include <ClosedLoop/DerivativeAveragingFilter.h>

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
	constexpr uint8_t TUNE_ERR_SYSTEM_ERROR					= 1u << 7;
	constexpr uint8_t TUNE_ERR_TUNING_FAILURE 				= TUNE_ERR_NOT_DONE_BASIC | TUNE_ERR_NOT_CALIBRATED | TUNE_ERR_SYSTEM_ERROR | TUNE_ERR_TOO_LITTLE_MOTION | TUNE_ERR_TOO_MUCH_MOTION;

	// Tuning manoeuvres
	constexpr uint8_t BASIC_TUNING_MANOEUVRE 				= 1u << 0;		// this measures the polarity, check that the CPR looks OK, and for relative encoders sets the zero position
	constexpr uint8_t ENCODER_CALIBRATION_MANOEUVRE 		= 1u << 1;		// this calibrates an absolute encoder
	// The remainder are not currently implemented
	constexpr uint8_t CONTINUOUS_PHASE_INCREASE_MANOEUVRE 	= 1u << 5;
	constexpr uint8_t STEP_MANOEUVRE 						= 1u << 6;
	constexpr uint8_t ZIEGLER_NICHOLS_MANOEUVRE 			= 1u << 7;

	//TODO reduce the number of these public variables, preferably to zero. Use a cleaner interface between the tuning module and the main closed loop module.
	extern Encoder *encoder;						// Pointer to the encoder object in use
	extern uint8_t tuning;							// Bitmask of any tuning manoeuvres that have been requested
	extern uint8_t tuningError;						// Flags for any tuning errors
	extern uint16_t measuredStepPhase;				// The measured position of the motor
	extern uint16_t desiredStepPhase;				// The desired position of the motor
	extern std::atomic<int32_t> rawEncoderReading;	// The raw reading taken from the encoder
	extern std::atomic<float> targetMotorSteps;		// The number of steps the motor should have taken relative to it's zero position
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
	uint8_t ReadLiveStatus() noexcept;
	void SetStepDirection(bool) noexcept;
	bool GetClosedLoopEnabled() noexcept;
	bool SetClosedLoopEnabled(uint8_t drive, bool enabled, const StringRef &reply) noexcept;
	void DriverSwitchedToClosedLoop(uint8_t drive) noexcept;
	void SetHoldingCurrent(float percent);
	void ResetError(size_t driver) noexcept;

	// Methods called by the encoders
	void EnableEncodersSpi() noexcept;
	void DisableEncodersSpi() noexcept;

	// Methods used only by closed loop and by the tuning module
	float PulsePerStepToExternalUnits(float pps, uint8_t encoderType) noexcept;
	float ExternalUnitsToPulsePerStep(float externalUnits, uint8_t encoderType) noexcept;
	void SetMotorPhase(uint16_t phase, float magnitude) noexcept;
	void SetForwardPolarity() noexcept;
	void SetBasicTuningResults(float forwardCountsPerStep, float ReverseCountsPerStep, int32_t finalReading) noexcept;
	void ResetStepPosition(uint16_t motorPhase) noexcept;

	// Methods in the tuning module
	void PerformTune() noexcept;
}

#  ifdef EXP1HCL
// The encoder uses the standard shared SPI device, so we don't need to enable/disable it
inline void ClosedLoop::EnableEncodersSpi() noexcept { }
inline void ClosedLoop::DisableEncodersSpi() noexcept { }
#  endif

# endif

#endif /* SRC_CLOSEDLOOP_CLOSEDLOOP_H_ */
