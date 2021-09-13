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
	// The size of the closed loop data buffer
#  if defined(EXP1HCE)
#   define CLOSED_LOOP_DATA_BUFFER_SIZE 50		//	(50 readings)
#  elif defined(EXP1HCL)
#   define CLOSED_LOOP_DATA_BUFFER_SIZE 2000		//  (2000 readings)
#  else
#   error Cannot support closed loop with the specified hardware
#  endif

	// Possible tuning errors
	constexpr uint8_t TUNE_ERR_NOT_FOUND_POLARITY			= 1 << 0;
	constexpr uint8_t TUNE_ERR_NOT_ZEROED					= 1 << 1;
	constexpr uint8_t TUNE_ERR_NOT_CHECKED_POLARITY			= 1 << 2;
	constexpr uint8_t TUNE_ERR_NOT_CHECKED_CONTROL			= 1 << 3;
	constexpr uint8_t TUNE_ERR_NOT_CHECKED_ENCODER_STEPS 	= 1 << 4;
	constexpr uint8_t TUNE_ERR_INCORRECT_POLARITY 			= 1 << 5;
	constexpr uint8_t TUNE_ERR_CONTROL_FAILED 				= 1 << 6;
	constexpr uint8_t TUNE_ERR_SYSTEM_ERROR					= 1 << 7;
	constexpr uint8_t TUNE_ERR_NOT_PERFORMED_MINIMAL_TUNE 	= TUNE_ERR_NOT_FOUND_POLARITY | TUNE_ERR_NOT_ZEROED | TUNE_ERR_NOT_CHECKED_POLARITY | TUNE_ERR_NOT_CHECKED_CONTROL | TUNE_ERR_NOT_CHECKED_ENCODER_STEPS;
	constexpr uint8_t TUNE_ERR_TUNING_FAILURE 				= TUNE_ERR_INCORRECT_POLARITY | TUNE_ERR_CONTROL_FAILED | TUNE_ERR_SYSTEM_ERROR;

	// Tuning manoeuvres
	constexpr uint8_t POLARITY_DETECTION_MANOEUVRE 			= 1 << 0;
	constexpr uint8_t ZEROING_MANOEUVRE 					= 1 << 1;
	constexpr uint8_t POLARITY_CHECK 						= 1 << 2;
	constexpr uint8_t CONTROL_CHECK 						= 1 << 3;
	constexpr uint8_t ENCODER_STEPS_CHECK 					= 1 << 4;
	constexpr uint8_t CONTINUOUS_PHASE_INCREASE_MANOEUVRE 	= 1 << 5;
	constexpr uint8_t STEP_MANOEUVRE 						= 1 << 6;
	constexpr uint8_t ZIEGLER_NICHOLS_MANOEUVRE 			= 1 << 7;
	constexpr uint8_t MINIMAL_TUNE 							= POLARITY_DETECTION_MANOEUVRE | ZEROING_MANOEUVRE | POLARITY_CHECK | CONTROL_CHECK | ENCODER_STEPS_CHECK;
	constexpr uint8_t FULL_TUNE 							= (1 << 8) - 1;

	// Enumeration of closed loop recording modes
	enum RecordingMode : uint8_t
	{
		Immediate = 0,
		OnNextMove = 1,
	};

	constexpr size_t TaskStackWords = 200;		// Size of the stack for all closed loop tasks

	// Control variables
	// Variables that can be set by the user to determine how the closed loop controller works

	extern bool closedLoopEnabled;			// Has closed loop been enabled by the user?
	extern uint8_t tuningError;						// Flags for any tuning errors
	extern uint8_t prevTuningError;					// Used to see what errors have been introduced by tuning

	extern bool reversePolarity;			// Flag if the polarity on this motor is reversed

	extern float holdCurrent;					// The minimum holding current when stationary

	extern float Kp;							// The proportional constant for the PID controller
	extern float Ki;							// The proportional constant for the PID controller
	extern float Kd;							// The proportional constant for the PID controller

	extern float errorThresholds[2];				// The error thresholds. [0] is pre-stall, [1] is stall

	extern uint8_t tuning;						// Bitmask of any tuning manoeuvres that have been requested
	extern float ultimateGain;					// The ultimate gain of the controller (used for tuning)
	extern float oscillationPeriod;				// The oscillation period when Kp = ultimate gain

	extern Encoder *encoder;				// Pointer to the encoder object in use
	extern float encoderPulsePerStep;				// How many encoder readings do we get per step?

	extern bool collectingData;				// Are we currently collecting data? If so:
	extern StepTimer::Ticks dataCollectionStartTicks;// - At what tick did data collection start?
	extern uint16_t rateRequested;					//	- What sample rate did they request?
	extern uint16_t filterRequested;				//	- What filter did they request?
	extern uint16_t samplesRequested;				//	- How many samples did they request?
	extern ClosedLoop::RecordingMode modeRequested;	//	- What mode did they request?
	extern uint8_t movementRequested;				//	- Which calibration movement did they request? 0=none, 1=polarity, 2=continuous
	extern float sampleBuffer[CLOSED_LOOP_DATA_BUFFER_SIZE * 14];	//	- Store the samples here (max. CLOSED_LOOP_DATA_BUFFER_SIZE samples of 12 variables)
	extern uint16_t sampleBufferReadPointer;	//  - Send this sample next to the mainboard
	extern uint16_t sampleBufferWritePointer;	//  - Store the next sample at this point in the buffer


	// Working variables
	// These variables are all used to calculate the required motor currents. They are declared here so they can be reported on by the data collection task

	extern std::atomic<int32_t> rawEncoderReading;		// The raw reading taken from the encoder
	extern bool stepDirection;				// The direction the motor is attempting to take steps in
	extern float currentMotorSteps;					// The number of steps the motor has taken relative to it's zero position
	extern std::atomic<float> targetMotorSteps;		// The number of steps the motor should have taken relative to it's zero position
	extern float currentError;						// The current error
	extern float lastError;						// The error from the previous iteration

	constexpr unsigned int derivativeFilterSize = 8;// The range of the derivative filter
	extern DerivativeAveragingFilter<derivativeFilterSize> *derivativeFilter;// An averaging filter to smooth the derivative of the error

	extern float PIDPTerm;							// Proportional term
	extern float PIDITerm;						// Integral term
	extern float PIDDTerm;							// Derivative term
	extern int16_t PIDControlSignal;				// The overall -255 to 255 signal from the PID controller

	extern int16_t phaseShift;						// The desired shift in the position of the motor
	extern uint16_t stepPhase;						// The current position of the motor
	extern uint16_t desiredStepPhase;			// The desired position of the motor

	extern int16_t coilA;							// The current to run through coil A
	extern int16_t coilB;							// The current to run through coil A

	extern bool stall;						// Has the closed loop error threshold been exceeded?
	extern bool preStall;					// Has the closed loop warning threshold been exceeded?

	extern int32_t tuneCounter;					// A counter for tuning tasks to use
	extern bool newTuningMove;				// Indicates if a tuning move has just finished
	extern float tuningVar1,
				 tuningVar2,
				 tuningVar3,
				 tuningVar4;						// Four general purpose variables for any tuning task to use


	// Monitoring variables
	// These variables monitor how fast the PID loop is running etc.

	extern StepTimer::Ticks minControlLoopRuntime;		// The minimum time the control loop has taken to run
	extern StepTimer::Ticks maxControlLoopRuntime;		// The maximum time the control loop has taken to run
	extern float ewmaControlLoopRuntime;				// The exponentially weighted moving average (ewma) time the control loop has taken

	extern StepTimer::Ticks prevControlLoopCallTime;	// The last time the control loop was called
	extern StepTimer::Ticks minControlLoopCallInterval;	// The minimum interval between the control loop being called
	extern StepTimer::Ticks maxControlLoopCallInterval;	// The maximum interval between the control loop being called
	extern float ewmaControlLoopCallInterval;			// An ewma of the frequency the control loop is called at

	// Closed loop methods

	void Init() noexcept;

	float PulsePerStepToExternalUnits(float pps, uint8_t encoderType) noexcept;
	float ExternalUnitsToPulsePerStep(float externalUnits, uint8_t encoderType) noexcept;

	GCodeResult ProcessM569Point1(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	GCodeResult ProcessM569Point5(const CanMessageStartClosedLoopDataCollection&, const StringRef&) noexcept;
	GCodeResult ProcessM569Point6(const CanMessageGeneric& msg, const StringRef& reply) noexcept;

	[[noreturn]] void DataCollectionLoop() noexcept;
	[[noreturn]] void DataTransmissionLoop() noexcept;

	void ReadState() noexcept;
	void ControlLoop() noexcept;
	void PerformTune() noexcept;
	void CollectSample() noexcept;
	void ControlMotorCurrents() noexcept;
	void SetMotorPhase(uint16_t phase, float magnitude) noexcept;

#  ifdef EXP1HCE
	void TurnAttinyOff() noexcept;
#  endif
	void EnableEncodersSpi() noexcept;
	void DisableEncodersSpi() noexcept;
	EncoderType GetEncoderType() noexcept;
	void Diagnostics(const StringRef& reply) noexcept;

	void TakeStep() noexcept;
	uint8_t ReadLiveStatus() noexcept;
	void SetStepDirection(bool) noexcept;
	bool GetClosedLoopEnabled() noexcept;
	void SetHoldingCurrent(float percent);
	void ResetError(size_t driver) noexcept;
	bool SetClosedLoopEnabled(bool, const StringRef&) noexcept;
}

#  ifdef EXP1HCL
// The encoder uses the standard shared SPI device, so we don't need to enable/disable it
inline void ClosedLoop::EnableEncodersSpi() noexcept { }
inline void ClosedLoop::DisableEncodersSpi() noexcept { }
#  endif

# endif

#endif /* SRC_CLOSEDLOOP_CLOSEDLOOP_H_ */
