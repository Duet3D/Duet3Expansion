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
# include "DerivativeAveragingFilter.h"
# include "TuningErrors.h"
# include "SampleBuffer.h"
# include "Encoders/Encoder.h"

constexpr float MaxSafeBacklash = 0.22;					// the maximum backlash in full steps that we can use - error if there is more
constexpr float MaxGoodBacklash = 0.15;					// the maximum backlash in full steps that we are happy with - warn if there is more
constexpr unsigned int LinearEncoderIncreaseFactor = 4;	// this should be a power of 2. Allowed backlash is increased by this amount for linear composite encoders.
constexpr float VelocityLimitGainFactor = 5.0;			// the gain of the P loop when in torque mode

class Encoder;
class SpiEncoder;
class CanMessageGenericParser;

// Struct to pass data back to the ClosedLoop module
struct MotionParameters
{
	float position = 0.0;
	float speed = 0.0;
	float acceleration = 0.0;
};

enum class ClosedLoopMode
{
	open = 0,
	closed,
	assistedOpen
};

class ClosedLoop
{
public:
	// Constants and variables that are used by both the ClosedLoop and the Tuning modules

	// Tuning manoeuvres
	static constexpr uint8_t BASIC_TUNING_MANOEUVRE 				= 1u << 0;		// this measures the polarity, check that the CPR looks OK, and for relative encoders sets the zero position
	static constexpr uint8_t ENCODER_CALIBRATION_MANOEUVRE 			= 1u << 1;		// this calibrates an absolute encoder
	static constexpr uint8_t ENCODER_CALIBRATION_CHECK				= 1u << 2;		// this checks the calibration
	static constexpr uint8_t STEP_MANOEUVRE 						= 1u << 6;		// this does a sudden step change in the requested position for PID tuning

#if 0	// The remainder are not currently implemented
	constexpr uint8_t CONTINUOUS_PHASE_INCREASE_MANOEUVRE 	= 1u << 5;
	constexpr uint8_t ZIEGLER_NICHOLS_MANOEUVRE 			= 1u << 7;
#endif

	// Closed loop public methods
	void InitInstance() noexcept;

	GCodeResult InstanceProcessM569Point1(CanMessageGenericParser& parser, const StringRef& reply) noexcept;
	GCodeResult InstanceProcessM569Point4(CanMessageGenericParser& parser, const StringRef& reply) noexcept;
	GCodeResult InstanceProcessM569Point5(const CanMessageStartClosedLoopDataCollection&, const StringRef& reply) noexcept;
	GCodeResult InstanceProcessM569Point6(CanMessageGenericParser& parser, const StringRef& reply) noexcept;
	void UpdateStandstillCurrent() noexcept;

	const char *_ecv_array GetModeText() const noexcept;
	void InstanceDiagnostics(size_t driver, const StringRef& reply) noexcept;

	// Methods called by the motion system
	void InstanceControlLoop() noexcept;
	StandardDriverStatus ReadLiveStatus() const noexcept;
	bool IsClosedLoopEnabled() const noexcept;
	bool SetClosedLoopEnabled(ClosedLoopMode mode, const StringRef &reply) noexcept;
	void DriverSwitchedToClosedLoop() noexcept;
	void ResetError() noexcept;
	bool OkayToSetDriverIdle() const noexcept;
	StandardDriverStatus ModifyDriverStatus(StandardDriverStatus originalStatus) noexcept;
	void GetStatistics(CanMessageDriversStatus::ClosedLoopStatus& stat) noexcept;

	// Methods called by the encoders
	static void EnableEncodersSpi() noexcept;
	static void DisableEncodersSpi() noexcept;

	static void Init() noexcept;
	static void ControlLoop() noexcept;
	static ClosedLoop *_ecv_null GetClosedLoopInstance(size_t driver) noexcept;
	static GCodeResult ProcessM569Point1(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	static GCodeResult ProcessM569Point4(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	static GCodeResult ProcessM569Point5(const CanMessageStartClosedLoopDataCollection&, const StringRef& reply) noexcept;
	static GCodeResult ProcessM569Point6(const CanMessageGeneric& msg, const StringRef& reply) noexcept;
	static bool OkayToSetDriverIdle(size_t driver) noexcept;
	static void Diagnostics(const StringRef& reply) noexcept;

	// Functions run by tasks
	[[noreturn]] void DataTransmissionTaskLoop() noexcept;
	[[noreturn]] void EncoderCalibrationTaskLoop() noexcept;

private:
	// Constants private to this module
	static constexpr unsigned int DerivativeFilterSize = 8;			// The range of the error derivative filter (use a power of 2 for efficiency)
	static constexpr unsigned int SpeedFilterSize = 8;				// The range of the speed filter (use a power of 2 for efficiency)
	static constexpr unsigned int tuningStepsPerSecond = 2000;		// the rate at which we send 1/256 microsteps during tuning, slow enough for high-inertia motors
	static constexpr StepTimer::Ticks stepTicksPerTuningStep = StepTimer::StepClockRate/tuningStepsPerSecond;
	static constexpr StepTimer::Ticks stepTicksBeforeTuning = StepTimer::StepClockRate/10;
																	// 1/10 sec delay between enabling the driver and starting tuning, to allow for brake release and current buildup
	static constexpr StepTimer::Ticks DataCollectionIdleStepTicks = StepTimer::StepClockRate/200;
																	// start collecting tuning data 5ms before the start of the tuning move
	static constexpr float DefaultHoldCurrentFraction = 0.25;		// the minimum fraction of the requested current that we apply when holding position
	static constexpr float DefaultTorquePerAmp = 1.0;				// the torque per amp of motor current

	static constexpr float PIDIlimit = 80.0;

	// Methods used only by closed loop and by the tuning module
	void SetMotorPhase(uint16_t phase, float magnitude) noexcept;
	void FinishedBasicTuning() noexcept;
																// call this when we have stopped basic tuning movement and are ready to switch to closed loop control
	void ReadyToCalibrate(bool store) noexcept;					// call this when encoder calibration has finished collecting data
	void AdjustTargetMotorSteps(float amount) noexcept;			// called by tuning to execute a step
	void ExitTorqueMode() noexcept;

	// Methods in the tuning module
	void PerformTune() noexcept;

	// Enumeration of closed loop recording modes
	enum RecordingMode : uint8_t
	{
		None = 0,			// not collecting data
		Immediate,			// collecting data now
		OnNextMove,			// collect data when the next movement command starts executing
		SendingData			// finished collecting data but still sending it to the main board
	};

	static ClosedLoop *closedLoopInstances[NumDrivers];

	Encoder *encoder = nullptr;									// Pointer to the encoder object in use
	volatile uint8_t tuning = 0;								// Bitmask of any tuning manoeuvres that have been requested
	TuningErrors tuningError;									// Flags for any tuning errors

	// Control variables, set by the user to determine how the closed loop controller works
	ClosedLoopMode currentMode = ClosedLoopMode::open;			// which mode the driver is in

	// Holding current, and variables derived from it
	float 	holdCurrentFraction = DefaultHoldCurrentFraction;	// The minimum holding current when stationary
	float	torquePerAmp = DefaultTorquePerAmp;					// the torque per amp of configured current
	float 	Kp = 30.0;											// The proportional constant for the PID controller
	float 	Ki = 0.0;											// The proportional constant for the PID controller
	float 	Kd = 0.0;											// The proportional constant for the PID controller
	float	Kv = 1000.0;										// The velocity feedforward constant
	float	Ka = 0.0;											// The acceleration feedforward constant

	float 	errorThresholds[2];									// The error thresholds. [0] is pre-stall, [1] is stall

	float torqueModeCommandedCurrentFraction = 0.0;		// when in torque mode, the requested torque
	float torqueModeMaxSpeed = 0.0;						// when in torque mode, the maximum speed. Zero or negative means no limit.

	// Working variables
	// These variables are all used to calculate the required motor currents. They are declared here so they can be reported on by the data collection task
	MotionParameters mParams;							// the target position, speed and acceleration
	float currentPositionError;							// the current position error in full steps
	float periodMaxAbsPositionError = 0.0;				// the maximum value of the absolute position error
	float periodSumOfPositionErrorSquares = 0.0;		// used to calculate the RMS error
	float periodMaxCurrentFraction = 0.0;				// the maximum current fraction over this period
	float periodSumOfCurrentFractions = 0.0;			// used to calculate the average current fraction
	unsigned int periodNumSamples = 0;					// how many samples are in sumOfPositionErrorSquares

	float 	PIDPTerm;									// Proportional term
	float 	PIDITerm = 0.0;								// Integral accumulator
	float 	PIDDTerm;									// Derivative term
	float	PIDVTerm;									// Velocity feedforward term
	float	PIDATerm;									// Acceleration feedforward term
	float	PIDControlSignal;							// The overall signal from the PID controller


	uint16_t desiredStepPhase = 0;						// The desired position of the motor
	uint16_t phaseOffset = 0;							// The amount by which the phase should be offset when in semi-open-loop mode
	int16_t coilA;										// The current to run through coil A
	int16_t coilB;										// The current to run through coil A

	bool	hasMovementCommand = false;					// true if a regular movement command is being executed
	bool	inTorqueMode = false;
	bool	torqueModeDirection;
	bool 	stall = false;								// Has the closed loop error threshold been exceeded?
	bool 	preStall = false;							// Has the closed loop warning threshold been exceeded?

	// Basic tuning synchronisation
	volatile bool basicTuningDataReady = false;

	// Encoder calibration synchronisation
	enum class CalibrationState : uint8_t { notReady = 0, dataReady, complete };
	volatile CalibrationState calibrationState = CalibrationState::notReady;
	volatile bool calibrateNotCheck = false;
	volatile TuningErrors calibrationErrors;

	StepTimer::Ticks whenLastTuningStepTaken;			// when the control loop last called the tuning code

	// Monitoring variables
	// These variables monitor how fast the PID loop is running etc.
	StepTimer::Ticks prevControlLoopCallTime;			// The last time the control loop was called
	StepTimer::Ticks minControlLoopRuntime;				// The minimum time the control loop has taken to run
	StepTimer::Ticks maxControlLoopRuntime;				// The maximum time the control loop has taken to run
	StepTimer::Ticks minControlLoopCallInterval;		// The minimum interval between the control loop being called
	StepTimer::Ticks maxControlLoopCallInterval;		// The maximum interval between the control loop being called

	// Data collection variables
	// Input variables
	volatile RecordingMode samplingMode = RecordingMode::None;	// What mode did they request? Volatile because we care about when it is written.
	uint8_t  movementRequested;									// Which calibration movement did they request? 0=none, 1=polarity, 2=continuous
	uint16_t filterRequested;									// What filter did they request?
	volatile uint16_t samplesRequested;							// The number of samples requested

	// Derived variables
	volatile uint16_t samplesCollected = 0;
	volatile uint16_t samplesSent = 0;
	bool sampleBufferOverflowed = false;						// set if the buffer is full when we need to store a sample
	StepTimer::Ticks dataCollectionStartTicks;					// At what tick did data collection start?
	StepTimer::Ticks dataCollectionIntervalTicks;				// the requested interval between samples
	StepTimer::Ticks whenNextSampleDue;							// when it will be time to take the next sample

	DerivativeAveragingFilter<DerivativeFilterSize> errorDerivativeFilter;	// An averaging filter to smooth the derivative of the error
	DerivativeAveragingFilter<SpeedFilterSize> speedFilter;		// An averaging filter to smooth the actual speed
	SampleBuffer sampleBuffer;									// buffer for collecting samples - declare this last because it is large

	// Functions private to this module
	EncoderType GetEncoderType() noexcept
	{
		return (encoder == nullptr) ? EncoderType::none : encoder->GetType();
	}

	// Return true if we are currently collecting data or primed to collect data or finishing sending data
	inline bool CollectingData() noexcept { return samplingMode != RecordingMode::None; }

	void CollectSample() noexcept;
	float ControlMotorCurrents(StepTimer::Ticks ticksSinceLastCall) noexcept;
	void StartTuning(uint8_t tuningType) noexcept;
	GCodeResult ProcessBasicTuningResult(const StringRef& reply) noexcept;
	GCodeResult ProcessCalibrationResult(const StringRef& reply) noexcept;
	void ReportTuningErrors(TuningErrors tuningErrorBitmask, const StringRef& reply) noexcept;
	void ResetMonitoringVariables() noexcept;
	void SetTargetToCurrentPosition() noexcept;
	void CreateCalibrationTask() noexcept;

	// Tuning methods
	bool BasicTuning(bool firstIteration) noexcept;
	bool EncoderCalibration(bool firstIteration) noexcept;
	bool Step(bool firstIteration) noexcept;
};

inline bool ClosedLoop::IsClosedLoopEnabled() const noexcept
{
	return currentMode != ClosedLoopMode::open;
}

inline ClosedLoop *ClosedLoop::GetClosedLoopInstance(size_t driver) noexcept
{
	return (driver < NumDrivers) ? closedLoopInstances[driver] : nullptr;
}

inline void ClosedLoop::ControlLoop() noexcept
{
	for (ClosedLoop* instance : closedLoopInstances)
	{
		instance->InstanceControlLoop();
	}
}

// The encoder uses the standard shared SPI device, so we don't need to enable/disable it
inline void ClosedLoop::EnableEncodersSpi() noexcept { }
inline void ClosedLoop::DisableEncodersSpi() noexcept { }

# endif

#endif /* SRC_CLOSEDLOOP_CLOSEDLOOP_H_ */
