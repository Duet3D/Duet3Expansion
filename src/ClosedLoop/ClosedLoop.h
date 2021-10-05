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

	// The size of the closed loop data buffer
	constexpr unsigned int DataBufferSize = 2000;		//  (2000 readings)

	// Possible tuning errors
	constexpr uint8_t TUNE_ERR_NOT_FOUND_POLARITY			= 1u << 0;
	constexpr uint8_t TUNE_ERR_NOT_ZEROED					= 1u << 1;
	constexpr uint8_t TUNE_ERR_NOT_CHECKED_POLARITY			= 1u << 2;
	constexpr uint8_t TUNE_ERR_NOT_CHECKED_CONTROL			= 1u << 3;
	constexpr uint8_t TUNE_ERR_NOT_CHECKED_ENCODER_STEPS 	= 1u << 4;
	constexpr uint8_t TUNE_ERR_INCORRECT_POLARITY 			= 1u << 5;
	constexpr uint8_t TUNE_ERR_CONTROL_FAILED 				= 1u << 6;
	constexpr uint8_t TUNE_ERR_SYSTEM_ERROR					= 1u << 7;
	constexpr uint8_t TUNE_ERR_TUNING_FAILURE 				= TUNE_ERR_INCORRECT_POLARITY | TUNE_ERR_CONTROL_FAILED | TUNE_ERR_SYSTEM_ERROR;

	// Tuning manoeuvres
	constexpr uint8_t POLARITY_DETECTION_MANOEUVRE 			= 1u << 0;
	constexpr uint8_t ZEROING_MANOEUVRE 					= 1u << 1;
	constexpr uint8_t POLARITY_CHECK 						= 1u << 2;
	constexpr uint8_t CONTROL_CHECK 						= 1u << 3;
	constexpr uint8_t ENCODER_STEPS_CHECK 					= 1u << 4;
	constexpr uint8_t CONTINUOUS_PHASE_INCREASE_MANOEUVRE 	= 1u << 5;
	constexpr uint8_t STEP_MANOEUVRE 						= 1u << 6;
	constexpr uint8_t ZIEGLER_NICHOLS_MANOEUVRE 			= 1u << 7;
	constexpr uint8_t FULL_TUNE 							= (1u << 8) - 1;

	// Enumeration of closed loop recording modes
	enum RecordingMode : uint8_t
	{
		Immediate = 0,
		OnNextMove = 1,
	};

	//TODO reduce the number of these public variables, preferably to zero. Use a cleaner interface between the tuning module and the mian closed loop module.
	extern Encoder *encoder;						// Pointer to the encoder object in use
	extern bool reversePolarity;					// Flag if the polarity on this motor is reversed
	extern uint8_t tuning;							// Bitmask of any tuning manoeuvres that have been requested
	extern uint8_t tuningError;						// Flags for any tuning errors
	extern uint16_t stepPhase;						// The current position of the motor
	extern uint16_t desiredStepPhase;				// The desired position of the motor
	extern std::atomic<int32_t> rawEncoderReading;	// The raw reading taken from the encoder
	extern std::atomic<float> targetMotorSteps;		// The number of steps the motor should have taken relative to it's zero position
	extern float encoderPulsePerStep;				// How many encoder readings do we get per step?
	extern float currentMotorSteps;					// The number of steps the motor has taken relative to it's zero position

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
	bool SetClosedLoopEnabled(bool enabled, const StringRef &reply) noexcept;
	void SetHoldingCurrent(float percent);
	void ResetError(size_t driver) noexcept;

	// Methods called by the encoders
	void EnableEncodersSpi() noexcept;
	void DisableEncodersSpi() noexcept;

	// Methods used only by closed loop and by the tuning module
	float PulsePerStepToExternalUnits(float pps, uint8_t encoderType) noexcept;
	float ExternalUnitsToPulsePerStep(float externalUnits, uint8_t encoderType) noexcept;
	void SetMotorPhase(uint16_t phase, float magnitude) noexcept;

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
