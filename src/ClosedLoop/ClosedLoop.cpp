/*
 * ClosedLoop.cpp
 *
 *  Created on: 9 Jun 2020
 *      Author: David
 */

/*
 * Some observations on closed loop control of motors:
 * 1. We can generate steps in a step ISR as we do in open loop ode, then add each microstep to the desired position.
 *    But it is probably better to calculate the position directly from the move parameters.
 *    We can calculate the current velocity and acceleration as well, which should enable more accurate positioning.
 * 2. Currently we allow the user to set microstepping as usual, because that works in both open and closed loop mode.
 *    If we required that the users uses microstepping equal to the encoder resolution, then it would be difficult to operate in open loop mode.
 * 3. We can write the control loop to aim for a particular encoder reading (by first converting the required step position to an encoder reading),
 *    or we can convert encoder readings to step positions and aim for a particular step position. If we do the latter then the control loop
 *    will typically hunt between two encoder positions that straddle the required step position. We could perhaps avoid that by treating
 *    a motor position error that is less than about 0.6 encoder steps as zero error. However, targeting encoder count has the advantage that
 *    we could use mostly integer maths, which would be better if we want to implement this on SAMC21 or RP2040 platforms.
 * 3. We need to be sure that we can count encoder steps and motor positions without loss of accuracy when they get high.
 *    The encoder with greatest resolution is currently the AS5047, which has 16384 counts/rev = 81.92 counts/full step. TLI5012B has the same resolution.
 *    If the machine has 100 microsteps/mm at x16 microstepping then this corresponds to 512 counts/mm. If stored as a 32-bit signed integer
 *    it will overflow at about +/-4.2km which should be OK for axes but perhaps not for extruders.
 * 4. If we represent motor step position as a float then to be accurate to the nearest encoder count we have only 24 bits available,
 *    so the highest we can go without loss of resolution is +/- 32.768 metres.
 * 5. Since all motion commands sent to the board are relative, we could reset the step count and encoder count periodically by a whole number of revolutions,
 *    as long as we are careful to do it atomically. If/when we support linear encoders, this will not be necessary for those.
 * 6. When using an absolute encoder, although it cold be treated like a relative encoder fir the purposes of motor control,
 *    the angle information may be useful to compensate for leadscrew nut irregularities; so we should preserve the angle information.
 */

#include "ClosedLoop.h"

#if SUPPORT_CLOSED_LOOP

using std::atomic;
using std::numeric_limits;

# include "AS5047D.h"
# include "TLI5012B.h"
# include "SpiEncoder.h"
# include <ClosedLoop/DerivativeAveragingFilter.h>

# include <math.h>
# include <Platform/Platform.h>
# include <General/Bitmap.h>
# include <Platform/TaskPriorities.h>
# include <CAN/CanInterface.h>
# include <CanMessageBuffer.h>
# include <CanMessageFormats.h>
# include <CanMessageGenericParser.h>
# include <CanMessageGenericTables.h>

# include "QuadratureEncoderPdec.h"

# if SUPPORT_TMC2160
#  include "Movement/StepperDrivers/TMC51xx.h"
# else
#  error Cannot support closed loop with the specified hardware
# endif

#define BASIC_TUNING_DEBUG	0

// Variables that are used by both the ClosedLoop and the Tuning modules

Encoder*	ClosedLoop::encoder = nullptr;				// Pointer to the encoder object in use
volatile uint8_t ClosedLoop::tuning = 0;				// Bitmask of any tuning manoeuvres that have been requested
TuningErrors ClosedLoop::tuningError;					// Flags for any tuning errors
uint32_t	ClosedLoop::currentMotorPhase;				// the phase (0 to 4095) that the driver is set to

namespace ClosedLoop
{
	// Constants private to this module
	constexpr size_t DataCollectionTaskStackWords = 200;		// Size of the stack for all closed loop tasks
	constexpr size_t EncoderCalibrationTaskStackWords = 500;	// Size of the stack for all closed loop tasks

	constexpr unsigned int derivativeFilterSize = 8;	// The range of the derivative filter (use a power of 2 for efficiency)
	constexpr unsigned int DataBufferSize = 2000 * 14;	// When collecting samples we can accommodate 2000 readings of up to 13 variables + timestamp
	constexpr unsigned int tuningStepsPerSecond = 2000;	// the rate at which we send 1/256 microsteps during tuning, slow enough for high-inertia motors
	constexpr StepTimer::Ticks stepTicksPerTuningStep = StepTimer::StepClockRate/tuningStepsPerSecond;
	constexpr StepTimer::Ticks stepTicksBeforeTuning = StepTimer::StepClockRate/10;
														// 1/10 sec delay between enabling the driver and starting tuning, to allow for brake release and current buildup
	constexpr StepTimer::Ticks DataCollectionIdleStepTicks = StepTimer::StepClockRate/200;
														// start collecting tuning data 5ms before the start of the tuning move
	constexpr float DefaultHoldCurrentFraction = 0.25;	// the minimum fraction of the requested current that we apply when holding position
	constexpr float MinimumDegreesPhaseShift = 15.0;	// the phase shift at which we start reducing current instead of reducing the phase shift, where 90deg is one full step
	constexpr float MinimumPhaseShift = (MinimumDegreesPhaseShift/360.0) * 4096;	// the same in units where 4096 is a complete circle

	constexpr float MaxSafeHysteresis = 0.2;			// the maximum hysteresis in full steps that we can use - error if there is more
	constexpr float MaxGoodHysteresis = 0.1;			// the maximum hysteresis in full steps that we are happy with - warn if there is more

	constexpr float PIDIlimit = 80.0;

	// Enumeration of closed loop recording modes
	enum RecordingMode : uint8_t
	{
		None = 0,			// not collecting data
		Immediate,			// collecting data now
		OnNextMove,			// collect data when the next movement command starts executing
		SendingData			// finished collecting data but still sending it to the main board
	};

	// Control variables, set by the user to determine how the closed loop controller works
	bool 	closedLoopEnabled = false;					// Has closed loop been enabled by the user?

	// Holding current, and variables derived from it
	float 	holdCurrentFraction = DefaultHoldCurrentFraction;			// The minimum holding current when stationary
	float	recipHoldCurrentFraction = 1.0/DefaultHoldCurrentFraction;	// The reciprocal of the minimum holding current
	float	holdCurrentFractionTimesMinPhaseShift = MinimumPhaseShift * DefaultHoldCurrentFraction;

	float 	Kp = 100;									// The proportional constant for the PID controller
	float 	Ki = 0;										// The proportional constant for the PID controller
	float 	Kd = 0;										// The proportional constant for the PID controller

	float 	errorThresholds[2];							// The error thresholds. [0] is pre-stall, [1] is stall

	float 	ultimateGain = 0;							// The ultimate gain of the controller (used for tuning)
	float 	oscillationPeriod = 0;						// The oscillation period when Kp = ultimate gain

	// Data collection variables
	// Input variables
	volatile RecordingMode samplingMode = RecordingMode::None;	// What mode did they request? Volatile because we care about when it is written.
	uint8_t 	movementRequested;						// Which calibration movement did they request? 0=none, 1=polarity, 2=continuous
	uint16_t	filterRequested;						// What filter did they request?
	volatile uint16_t samplesRequested;

	// Derived variables
	volatile unsigned int variableCount;
	volatile uint16_t samplesCollected = 0;
	volatile uint16_t samplesSent = 0;;
	StepTimer::Ticks dataCollectionStartTicks;			// At what tick did data collection start?
	StepTimer::Ticks dataCollectionIntervalTicks;		// the requested interval between samples
	StepTimer::Ticks whenNextSampleDue;					// when it will be time to take the next sample

	// Data collection buffer and related variables
	float 	sampleBuffer[DataBufferSize];				// Ring buffer to store the samples in
	volatile size_t sampleBufferReadPointer = 0;		// Send this sample next to the main board
	volatile size_t sampleBufferWritePointer = 0;		// Store the next sample at this point in the buffer
	volatile size_t	sampleBufferLimit;					// the limit for the read/write pointers, to avoid wrapping within a single set of sampled variables
	volatile bool	sampleBufferOverflowed;				// true if we collected data faster than we could send it

	// Working variables
	// These variables are all used to calculate the required motor currents. They are declared here so they can be reported on by the data collection task
	bool 	stepDirection = true;						// The direction the motor is attempting to take steps in
	float	targetMotorSteps;							// The number of steps the motor should have taken relative to it's zero position
	int32_t targetEncoderReading;						// The encoder reading we want, calculated from targetMotorSteps
	float 	currentError;								// The current error

	DerivativeAveragingFilter<derivativeFilterSize> derivativeFilter;	// An averaging filter to smooth the derivative of the error

	float 	PIDPTerm;									// Proportional term
	float 	PIDITerm = 0.0;								// Integral accumulator
	float 	PIDDTerm;									// Derivative term
	float	PIDControlSignal;							// The overall signal from the PID controller

	float	phaseShift;									// The desired shift in the position of the motor, where 1024 = 1 full step

	uint16_t desiredStepPhase = 0;						// The desired position of the motor
	int16_t coilA;										// The current to run through coil A
	int16_t coilB;										// The current to run through coil A

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

	// Functions private to this module
	EncoderType GetEncoderType() noexcept
	{
		return (encoder == nullptr) ? EncoderType::none : encoder->GetType();
	}

	// Return true if we are currently collecting data or primed to collect data or finishing sending data
	inline bool CollectingData() noexcept { return samplingMode != RecordingMode::None; }

	void ReadState() noexcept;
	void CollectSample() noexcept;
	void ControlMotorCurrents(StepTimer::Ticks loopStartTime) noexcept;
	void StartTuning(uint8_t tuningType) noexcept;
	GCodeResult ProcessBasicTuningResult(const StringRef& reply) noexcept;
	GCodeResult ProcessCalibrationResult(const StringRef& reply) noexcept;
	void ReportTuningErrors(TuningErrors tuningErrorBitmask, const StringRef& reply) noexcept;

	extern "C" [[noreturn]] void DataTransmissionLoop(void *param) noexcept;
	extern "C" [[noreturn]] void EncoderCalibrationLoop(void *param) noexcept;

}	// end namespace

// Tasks and task loops
static Task<ClosedLoop::DataCollectionTaskStackWords> *dataTransmissionTask = nullptr;		// Data transmission task - handles sending back the buffered sample data
static Task<ClosedLoop::EncoderCalibrationTaskStackWords> *encoderCalibrationTask = nullptr;		// Encoder calibration task - handles calibrating the encoder in the background

// Helper function to count the number of variables being collected by a given filter
static inline unsigned int CountVariablesCollected(uint16_t filter)
{
	return (new Bitmap<uint16_t>(filter))->CountSetBits() + 1;
}

// Helper function to convert a time period (expressed in StepTimer::Ticks) to ms
static inline float TickPeriodToTimePeriod(StepTimer::Ticks tickPeriod)
{
	return tickPeriod * StepTimer::StepClocksToMillis;
}

// Helper function to convert a time period (expressed in StepTimer::Ticks) to a frequency in Hz
static inline float TickPeriodToFreq(StepTimer::Ticks tickPeriod)
{
	return 1000.0l / TickPeriodToTimePeriod(tickPeriod);
}

// Helper function to reset the 'monitoring variables' as defined above
static void ResetMonitoringVariables()
{
	ClosedLoop::minControlLoopRuntime = numeric_limits<StepTimer::Ticks>::max();
	ClosedLoop::maxControlLoopRuntime = numeric_limits<StepTimer::Ticks>::min();
	ClosedLoop::minControlLoopCallInterval = numeric_limits<StepTimer::Ticks>::max();
	ClosedLoop::maxControlLoopCallInterval = numeric_limits<StepTimer::Ticks>::min();
}

// Helper function to cat all the current tuning errors onto a reply in human-readable form
void ClosedLoop::ReportTuningErrors(TuningErrors tuningErrorBitmask, const StringRef &reply)
{
	if (tuningErrorBitmask & TuningError::NeedsBasicTuning) 	{ reply.cat(", the drive has not had basic tuning done"); }
	if (tuningErrorBitmask & TuningError::NotCalibrated) 		{ reply.cat(", the drive has not been calibrated"); }
	if (tuningErrorBitmask & TuningError::SystemError) 			{ reply.cat(", a system error occurred while tuning"); }
	if (tuningErrorBitmask & TuningError::CalibrationInProgress){ reply.cat(", encoder calibration is in progress"); }
	if (tuningErrorBitmask & TuningError::InconsistentMotion)	{ reply.cat(", the measured motion was inconsistent"); }
	if (tuningErrorBitmask & TuningError::TooLittleMotion)		{ reply.cat(", the measured motion was less than expected"); }
	if (tuningErrorBitmask & TuningError::TooMuchMotion)		{ reply.cat(", the measured motion was more than expected"); }
}

// Helper function to set the motor to a given phase and magnitude
// The phase is normally in the range 0 to 4095 but when tuning it can be 0 to somewhat over 8192.
// We must take it modulo 4096 when computing the currents. Funtion Trigonometry::FastSinCos does that.
void ClosedLoop::SetMotorPhase(uint16_t phase, float magnitude) noexcept
{
	currentMotorPhase = phase;
	float sine, cosine;
	Trigonometry::FastSinCos(phase, sine, cosine);
	coilA = (int16_t)lrintf(cosine * magnitude);
	coilB = (int16_t)lrintf(sine * magnitude);

# if SUPPORT_TMC2160 && SINGLE_DRIVER
	SmartDrivers::SetRegister(0, SmartDriverRegister::xDirect, (((uint32_t)(uint16_t)coilB << 16) | (uint32_t)(uint16_t)coilA) & 0x01FF01FF);
# else
#  error Cannot support closed loop with the specified hardware
# endif
}

static void GenerateTmcClock()
{
	// Currently we program DPLL0 to generate 120MHz output, so to get 15MHz we divide by 8
	ConfigureGclk(ClockGenGclkNumber, GclkSource::dpll0, 8, true);
	SetPinFunction(ClockGenPin, ClockGenPinPeriphMode);
	SmartDrivers::SetTmcExternalClock(15000000);
}

void ClosedLoop::Init() noexcept
{
	pinMode(EncoderCsPin, OUTPUT_HIGH);											// make sure that any attached SPI encoder is not selected

	GenerateTmcClock();															// generate the clock for the TMC2160A

	// Initialise to default error thresholds
	errorThresholds[0] = DefaultClosedLoopPositionWarningThreshold;
	errorThresholds[1] = DefaultClosedLoopPositionErrorThreshold;

	// Initialise the monitoring variables
	ResetMonitoringVariables();

	PIDITerm = 0.0;
	derivativeFilter.Reset();

	// Set up the data transmission task
	dataTransmissionTask = new Task<ClosedLoop::DataCollectionTaskStackWords>;
	dataTransmissionTask->Create(DataTransmissionLoop, "CLSend", nullptr, TaskPriority::ClosedLoopDataTransmission);
}

GCodeResult ClosedLoop::ProcessM569Point1(const CanMessageGeneric &msg, const StringRef &reply) noexcept
{
	CanMessageGenericParser parser(msg, M569Point1Params);

	// Set default parameters
	uint8_t tempEncoderType = GetEncoderType().ToBaseType();
	float tempCPR;
	float tempKp = Kp;
	float tempKi = Ki;
	float tempKd = Kd;
	uint16_t tempStepsPerRev = 200;
	size_t numThresholds = 2;
	float tempErrorThresholds[numThresholds];
	float holdingCurrentPercent;

	// Pull changed parameters
	const bool seenT = parser.GetUintParam('T', tempEncoderType);
	const bool seenC = parser.GetFloatParam('C', tempCPR);
	const bool seenPid = parser.GetFloatParam('R', tempKp) | parser.GetFloatParam('I', tempKi)  | parser.GetFloatParam('D', tempKd);
	const bool seenE = parser.GetFloatArrayParam('E', numThresholds, tempErrorThresholds);
	const bool seenH = parser.GetFloatParam('H', holdingCurrentPercent);
	const bool seenS = parser.GetUintParam('S', tempStepsPerRev);

	// Report back if no parameters to change
	if (!(seenT || seenC || seenPid || seenE || seenH))
	{
		if (encoder == nullptr)
		{
			reply.cat("No encoder configured");
		}
		else
		{
			reply.catf("Encoder type: %s", GetEncoderType().ToString());
			encoder->AppendStatus(reply);
			reply.lcatf("PID parameters P=%.3f I=%.3f D=%.3f, min. current %" PRIi32 "%%", (double) Kp, (double) Ki, (double) Kd, lrintf(holdCurrentFraction * 100.0));
		}
		return GCodeResult::ok;
	}

	if (seenT && !seenC && (tempEncoderType == EncoderType::rotaryQuadrature || tempEncoderType == EncoderType::linearQuadrature))
	{
		reply.copy("Missing C parameter");
		return GCodeResult::error;
	}

	if ((seenC || seenS) && !seenT)
	{
		reply.copy("M569.1 C and S parameters not permitted without T parameter");
		return GCodeResult::error;
	}

	// Validate the new params
	if (tempEncoderType > EncoderType::NumValues)
	{
		reply.copy("Invalid T value. Valid values are 2 and 3");
		return GCodeResult::error;
	}
	if (seenE && (tempErrorThresholds[0] < 0 || tempErrorThresholds[1] < 0))
	{
		reply.copy("Error threshold value must nor be less than zero");
		return GCodeResult::error;
	}
	if (seenC && tempCPR < 4 * tempStepsPerRev)
	{
		reply.copy("Encoder counts/rev must be at least four times steps/rev");
		return GCodeResult::error;
	}

	// Set the new params
	TaskCriticalSectionLocker lock;			// don't allow the closed loop task to see an inconsistent combination of these values

	if (seenPid)
	{
		Kp = tempKp;
		Ki = tempKi;
		Kd = tempKd;
		PIDITerm = 0.0;
		derivativeFilter.Reset();
	}

	if (seenH)
	{
		holdCurrentFraction = constrain<float>(holdingCurrentPercent, 10.0, 100.0) / 100.0;
		holdCurrentFractionTimesMinPhaseShift = MinimumPhaseShift * holdCurrentFraction;
		recipHoldCurrentFraction = 1.0/holdCurrentFraction;
	}

	if (seenE)
	{
		errorThresholds[0] = tempErrorThresholds[0];
		errorThresholds[1] = tempErrorThresholds[1];
	}

	if (seenT)
	{
		//TODO do we need to get a lock here in case there is any movement?
		SetClosedLoopEnabled(0, false, reply);
		DeleteObject(encoder);

		switch (tempEncoderType)
		{
		case EncoderType::none:
		default:
			// encoder is already nullptr
			break;

		case EncoderType::AS5047:
			encoder = new AS5047D(tempStepsPerRev, *Platform::sharedSpi, EncoderCsPin);
			if (encoderCalibrationTask == nullptr)
			{
				encoderCalibrationTask = new Task<ClosedLoop::EncoderCalibrationTaskStackWords>;
				encoderCalibrationTask->Create(EncoderCalibrationLoop, "EncCal", nullptr, TaskPriority::SpinPriority);		// must be same priority as main task
			}
			break;

		case EncoderType::TLI5012:
			encoder = new TLI5012B(tempStepsPerRev, *Platform::sharedSpi, EncoderCsPin);
			if (encoderCalibrationTask == nullptr)
			{
				encoderCalibrationTask = new Task<ClosedLoop::EncoderCalibrationTaskStackWords>;
				encoderCalibrationTask->Create(EncoderCalibrationLoop, "EncCal", nullptr, TaskPriority::SpinPriority);		// must be same priority as main task
			}
			break;

#if defined(EXP1HCLv1_0)
		case EncoderType::linearQuadrature:
			encoder = new QuadratureEncoderPdec(tempStepsPerRev, tempCPR);
			break;

		case EncoderType::rotaryQuadrature:
			encoder = new QuadratureEncoderPdec(tempStepsPerRev, tempCPR);
			break;
#endif
		}

		if (encoder != nullptr)
		{
			tuningError = encoder->MinimalTuningNeeded();
			const GCodeResult rslt = encoder->Init(reply);
			if (rslt == GCodeResult::ok && encoder->LoadLUT())
			{
				tuningError &= ~TuningError::NotCalibrated;
			}
			return rslt;
		}
	}

	return GCodeResult::ok;
}

GCodeResult ClosedLoop::ProcessM569Point5(const CanMessageStartClosedLoopDataCollection& msg, const StringRef& reply) noexcept
{
	if (encoder == nullptr || msg.deviceNumber != 0)
	{
		reply.copy("No encoder has been configured");
		return GCodeResult::error;
	}

	if (CollectingData())
	{
		reply.copy("Driver is already collecting data");
		return GCodeResult::error;
	}

	uint8_t requestedMode;
	if (msg.movement != 0)
	{
		requestedMode = (uint8_t)RecordingMode::OnNextMove;		// when recording a tuning move we ignore the mode and start 5ms before it
	}
	else
	{
		requestedMode = msg.mode + 1;							// the A parameter is out of step with the enumeration by 1
		if (requestedMode != (uint8_t)RecordingMode::Immediate && requestedMode != (uint8_t)RecordingMode::OnNextMove)
		{
			reply.copy("Invalid recording mode");
			return GCodeResult::error;
		}
	}

	if (msg.movement != 0 && tuning != 0)
	{
		reply.copy("Driver is already performing tuning");
		return GCodeResult::error;
	}

	// Calculate how many samples will fit in the buffer
	variableCount = CountVariablesCollected(msg.filter);
	const unsigned int samplesPerBuffer =  ARRAY_SIZE(sampleBuffer) / variableCount;
	sampleBufferLimit = samplesPerBuffer * variableCount;		// wrap the read/write pointers round when they reach this value

	// Set up the recording vars
	sampleBufferWritePointer = sampleBufferReadPointer = 0;
	samplesCollected = samplesSent = 0;
	sampleBufferOverflowed = false;
	filterRequested = msg.filter;
	samplesRequested = msg.numSamples;
	dataCollectionIntervalTicks = (msg.rate == 0) ? 1 : StepTimer::StepClockRate/msg.rate;
	dataCollectionStartTicks = whenNextSampleDue = StepTimer::GetTimerTicks();
	samplingMode = (RecordingMode)requestedMode;				// do this one last, it triggers data collection

	StartTuning(msg.movement);
	return GCodeResult::ok;
}

// This is called when tuning has finished and we have a relative encoder
GCodeResult ClosedLoop::ProcessBasicTuningResult(const StringRef& reply) noexcept
{
	// Tuning has finished - there are now 3 scenarios
	// 1. No tuning errors exist (!tuningError)							= OK
	// 2. No new tuning errors exist !(~prevTuningError & tuningError)	= WARNING
	// 3. A new tuning error has been introduced (else)					= WARNING
	reply.printf("Driver %u.0 basic tuning ", CanInterface::GetCanAddress());
	if (!basicTuningDataReady)
	{
		tuningError &= ~TuningError::CalibrationInProgress;
		reply.cat("failed (no reason available)");
		return GCodeResult::error;
	}

	basicTuningDataReady = false;

	const TuningErrors newTuningErrors = encoder->ProcessTuningData();
	if (newTuningErrors != 0)
	{
		tuningError &= ~TuningError::CalibrationInProgress;
		reply.cat("failed");
		tuningError = (tuningError & ~(TuningError::TooMuchMotion | TuningError::TooLittleMotion | TuningError::InconsistentMotion)) | newTuningErrors;
		ReportTuningErrors(newTuningErrors, reply);
		if (newTuningErrors & (TuningError::TooMuchMotion | TuningError::TooLittleMotion))
		{
			reply.catf(", measured counts/rev is about %.1f", (double)((encoder->GetMeasuredCountsPerStep() * encoder->GetStepsPerRev()) * 0.25));
		}
		return GCodeResult::error;
	}

#ifdef DEBUG
	debugPrintf("commanded phase %" PRIu32 " measured phase %" PRIu32 "\n", currentMotorPhase, encoder->GetCurrentPhasePosition());
	encoder->TakeReading();
	debugPrintf("commanded phase %" PRIu32 " measured phase %" PRIu32 "\n", currentMotorPhase, encoder->GetCurrentPhasePosition());
#endif
	PIDITerm = 0.0;
	derivativeFilter.Reset();
	targetEncoderReading = encoder->GetCurrentCount();
	targetMotorSteps = (float)targetEncoderReading / encoder->GetCountsPerStep();
	tuningError &= ~TuningError::CalibrationInProgress;

	const float hyst = encoder->GetMeasuredHysteresis();
	if (hyst >= MaxSafeHysteresis)
	{
		tuningError = TuningError::HysteresisTooHigh;
		reply.catf("failed, measured hysteresis (%.3f step) is too high", (double)hyst);
		return GCodeResult::error;
	}
	else if (hyst >= MaxGoodHysteresis)
	{
		reply.catf("succeeded but measured hysteresis (%.3f step) is high", (double)hyst);
		tuningError = 0;
		return GCodeResult::warning;
	}
	else
	{
		reply.catf("succeeded, measured hysteresis %.3f step", (double)hyst);
		tuningError = 0;
		return GCodeResult::ok;
	}
}

// This function is run by the encoder calibration task.
// Its purpose is to wait for encoder calibration data to become available and process it.
// Processing to takes several seconds, so we need to do it in a separate task to avoid the main board timing out awaiting CAN responses.
void ClosedLoop::EncoderCalibrationLoop(void *param) noexcept
{
	for (;;)
	{
		TaskBase::Take();
		if (encoder != nullptr && encoder->IsAbsolute() && calibrationState == CalibrationState::dataReady)
		{
			calibrationErrors = ((AbsoluteRotaryEncoder*)encoder)->Calibrate(calibrateNotCheck);
			calibrationState = CalibrationState::complete;
		}
	}
}

GCodeResult ClosedLoop::ProcessCalibrationResult(const StringRef& reply) noexcept
{
	if (calibrationState == CalibrationState::dataReady)
	{
		// Waiting for the calibration task to finish processing the calibration via our Spin() function
		return GCodeResult::notFinished;
	}

	reply.printf("Driver %u.0 calibration ", CanInterface::GetCanAddress());
	if (calibrationState != CalibrationState::complete)
	{
		reply.cat("failed (no reason available)");
		return GCodeResult::error;
	}

	// Must have calibrationState == CalibrationState::complete
	calibrationState = CalibrationState::notReady;
	if (!calibrateNotCheck)
	{
		reply.cat("check ");
	}
	if (calibrationErrors != 0)
	{
		reply.cat("failed");
		if (calibrateNotCheck)
		{
			tuningError = (tuningError & ~(TuningError::TooMuchMotion | TuningError::TooLittleMotion | TuningError::InconsistentMotion | TuningError::CalibrationInProgress)) | calibrationErrors;
		}
		else
		{
			tuningError &= ~TuningError::CalibrationInProgress;
		}
		ReportTuningErrors(calibrationErrors, reply);
		if (calibrationErrors & (TuningError::TooMuchMotion | TuningError::TooLittleMotion))
		{
			reply.catf(", measured counts/step is about %.1f", (double)encoder->GetMeasuredCountsPerStep());
		}
		return GCodeResult::error;
	}

	PIDITerm = 0.0;
	derivativeFilter.Reset();
	targetEncoderReading = encoder->GetCurrentCount();
	targetMotorSteps = (float)targetEncoderReading / encoder->GetCountsPerStep();

	const float hyst = encoder->GetMeasuredHysteresis();
	if (hyst >= MaxSafeHysteresis)
	{
		if (calibrateNotCheck) { tuningError = TuningError::HysteresisTooHigh; }
		else { tuningError &= ~TuningError::CalibrationInProgress; }
		reply.catf("failed, measured hysteresis (%.3f step) is too high", (double)hyst);
		return GCodeResult::error;
	}
	else if (hyst >= MaxGoodHysteresis)
	{
		if (calibrateNotCheck) { tuningError = 0; }
		else { tuningError &= ~TuningError::CalibrationInProgress; }
		reply.catf("succeeded but measured hysteresis (%.3f step) is high", (double)hyst);
	}
	else
	{
		if (calibrateNotCheck) { tuningError = 0; }
		else { tuningError &= ~TuningError::CalibrationInProgress; }
		reply.catf("succeeded, measured hysteresis is %.3f step", (double)hyst);
	}

	// Report the calibration errors and corrections
	reply.lcatf("%s encoder reading errors: ", (calibrateNotCheck) ? "Original" : "Residual");
	((AbsoluteRotaryEncoder*)encoder)->AppendCalibrationErrors(reply);
	if (calibrateNotCheck)
	{
		reply.lcatf("Corrections made: ");
		((AbsoluteRotaryEncoder*)encoder)->AppendLUTCorrections(reply);
	}

	return GCodeResult::ok;
}

GCodeResult ClosedLoop::ProcessM569Point6(const CanMessageGeneric &msg, const StringRef &reply) noexcept
{
	if (encoder == nullptr)
	{
		reply.copy("no encoder configured");
		return GCodeResult::error;
	}

	CanMessageGenericParser parser(msg, M569Point6Params);

	uint8_t desiredTuning;
	if (!parser.GetUintParam('V', desiredTuning))
	{
		// We have been called to return the status after the previous call returned "not finished"
		if (tuning != 0)
		{
			return GCodeResult::notFinished;
		}

		// If we were checking the calibration, report the result
		return (encoder->IsAbsolute()) ? ProcessCalibrationResult(reply) : ProcessBasicTuningResult(reply);
	}

	switch (desiredTuning)
	{
	default:
		reply.copy("invalid tuning mode");
		return GCodeResult::error;

	case 1:		// basic calibration
		if (encoder->IsAbsolute())
		{
			reply.copy("basic tuning is not applicable to absolute encoders");
			return GCodeResult::error;
		}
		break;

	case 2:
	case 3:
	case 4:
		if (!encoder->IsAbsolute())
		{
			reply.copy("calibration is not applicable to relative encoders");
			return GCodeResult::error;
		}
		if (desiredTuning == 4)
		{
			// Tuning move 4 just clears the lookup table
			((AbsoluteRotaryEncoder*)encoder)->ScrubLUT();
			reply.copy("Encoder calibration cleared");
			tuningError |= TuningError::NotCalibrated;
			return GCodeResult::ok;
		}
		break;

	case 64:
		break;
	}

	// Here if this is a new command to start a tuning move
	// Check we are in direct drive mode
	if (SmartDrivers::GetDriverMode(0) != DriverMode::direct)
	{
		reply.copy("Drive is not in closed loop mode");
		return GCodeResult::error;
	}

	if (!Platform::EnableIfIdle(0))
	{
		reply.copy("Drive is not enabled");
		return GCodeResult::error;
	}

	StartTuning(desiredTuning);
	return GCodeResult::notFinished;
}

void ClosedLoop::StartTuning(uint8_t tuningMode) noexcept
{
	if (tuningMode != 0)
	{
		whenLastTuningStepTaken = StepTimer::GetTimerTicks() + stepTicksBeforeTuning;	// delay the start to allow brake release and motor current buildup
		tuning = (tuningMode == 1) ? BASIC_TUNING_MANOEUVRE
					: (tuningMode == 2) ? ENCODER_CALIBRATION_MANOEUVRE
						: (tuningMode == 3) ? ENCODER_CALIBRATION_CHECK
							: (tuningMode == 64) ? STEP_MANOEUVRE
								: 0;
	}
}

// Call this when we have stopped basic tuning movement and are ready to switch to closed loop control
void ClosedLoop::FinishedBasicTuning() noexcept
{
	basicTuningDataReady = true;
	tuningError |= TuningError::CalibrationInProgress;				// to prevent movement until we are dne tuning
}

// Call this when encoder calibration has finished collecting data
void ClosedLoop::ReadyToCalibrate(bool store) noexcept
{
	calibrateNotCheck = store;
	if (encoderCalibrationTask != nullptr)
	{
		calibrationState = CalibrationState::dataReady;
		tuningError |= TuningError::CalibrationInProgress;			// to prevent movement in case we are re-calibrating
		encoderCalibrationTask->Give();
	}
}

// This is called by tuning to execute a step
void ClosedLoop::AdjustTargetMotorSteps(float amount) noexcept
{
	targetMotorSteps += amount;
	targetEncoderReading = lrintf(targetMotorSteps * encoder->GetCountsPerStep());
}

void ClosedLoop::ControlLoop() noexcept
{
	// Record the control loop call interval
	const StepTimer::Ticks loopCallTime = StepTimer::GetTimerTicks();
	const StepTimer::Ticks timeElapsed = loopCallTime - prevControlLoopCallTime;
	minControlLoopCallInterval = min<StepTimer::Ticks>(minControlLoopCallInterval, timeElapsed);
	maxControlLoopCallInterval = max<StepTimer::Ticks>(maxControlLoopCallInterval, timeElapsed);

	// Read the current state of the drive. Do this even if we are not in closed loop mode.
	ReadState();

	// Calculate and store the current error in full steps
	currentError = (float)(targetEncoderReading - encoder->GetCurrentCount()) * encoder->GetStepsPerCount();
	derivativeFilter.ProcessReading(currentError, loopCallTime);

	if (!closedLoopEnabled)
	{
		// If closed loop disabled, do nothing
	}
	else if (tuning != 0)											// if we need to tune, do it
	{
		// Limit the rate at which we command tuning steps Need to do signed comparison because initially, whenLastTuningStepTaken is in the future.
		const int32_t timeSinceLastTuningStep = (int32_t)(loopCallTime - whenLastTuningStepTaken);
		if (timeSinceLastTuningStep >= (int32_t)stepTicksPerTuningStep)
		{
			whenLastTuningStepTaken = loopCallTime;
			PerformTune();
		}
		else if (samplingMode == RecordingMode::OnNextMove && timeSinceLastTuningStep + (int32_t)DataCollectionIdleStepTicks >= 0)
		{
			dataCollectionStartTicks = whenNextSampleDue = loopCallTime;
			samplingMode = RecordingMode::Immediate;
		}
	}
	else if (tuningError)
	{
		// Don't do anything if there is a tuning error, or calibration is in progress after tuning
	}
	else
	{
		ControlMotorCurrents(loopCallTime);							// otherwise control those motor currents!

		// Look for a stall or pre-stall
		preStall = errorThresholds[0] > 0 && fabsf(currentError) > errorThresholds[0];
		const bool alreadyStalled = stall;
		stall 	 = errorThresholds[1] > 0 && fabsf(currentError) > errorThresholds[1];
		if (stall && !alreadyStalled)
		{
			Platform::NewDriverFault();
		}
	}

	// Collect a sample, if we need to
	if (samplingMode == RecordingMode::Immediate && (int32_t)(loopCallTime - whenNextSampleDue) >= 0)
	{
		// It's time to take a sample
		CollectSample();
		whenNextSampleDue += dataCollectionIntervalTicks;
	}

	// Record how long this has taken to run
	prevControlLoopCallTime = loopCallTime;
	const StepTimer::Ticks loopRuntime = StepTimer::GetTimerTicks() - loopCallTime;
	minControlLoopRuntime = min<StepTimer::Ticks>(minControlLoopRuntime, loopRuntime);
	maxControlLoopRuntime = max<StepTimer::Ticks>(maxControlLoopRuntime, loopRuntime);
}

// Send data from the buffer to the main board over CAN
[[noreturn]] void ClosedLoop::DataTransmissionLoop(void *param) noexcept
{
	while (true)
	{
		RecordingMode locMode;										// to capture the volatile variable
		if ((locMode = samplingMode) == RecordingMode::Immediate || locMode == RecordingMode::SendingData)
		{
			// Started a new data collection
			samplesSent = 0;

			// Loop until everything has been read. Stop when either we have sent the requested number of samples, or the state is SendingData and we have sent all the data in the buffer.
			// Note, this may mean that the last packet contains no data and has the "last" flag set.
			bool finished;
			do
			{
				// Set up a CAN message
				CanMessageBuffer buf(nullptr);
				CanMessageClosedLoopData& msg = *(buf.SetupStatusMessage<CanMessageClosedLoopData>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress()));

				// Populate the control fields
				msg.firstSampleNumber = samplesSent;
				msg.filter = filterRequested;
				msg.zero = msg.zero2 = 0;

				unsigned int numCopied = 0;
				unsigned int numSamplesInMessage = 0;
				size_t copyReadPointer = sampleBufferReadPointer;	// capture volatile variable
				do
				{
					while (samplesSent == samplesCollected && samplingMode == RecordingMode::Immediate)
					{
						TaskBase::Take();							// wait for data to be available
					}

					if (samplesSent < samplesCollected)
					{
						memcpy(msg.data + numCopied, sampleBuffer + copyReadPointer, variableCount * sizeof(float));
						numCopied += variableCount;
						copyReadPointer += variableCount;
						if (copyReadPointer >= sampleBufferLimit)
						{
							copyReadPointer = 0;
						}
						++samplesSent;								// update this one first to avoid a race condition
						sampleBufferReadPointer = copyReadPointer;	// now it's safe to update this one
						++numSamplesInMessage;
					}
					finished = (samplingMode != RecordingMode::Immediate && samplesSent == samplesCollected);
				} while (!finished && numCopied + variableCount <= CanMessageClosedLoopData::MaxDataItems);

				msg.numSamples = numSamplesInMessage;
				msg.lastPacket = finished;
				msg.overflowed = sampleBufferOverflowed;

				// Send the CAN message
				buf.dataLength = msg.GetActualDataLength();
				CanInterface::Send(&buf);
			} while (!finished);
			samplingMode = RecordingMode::None;
		}
		else
		{
			TaskBase::Take();									// wait for a new data collection to start
		}
	}
}

// Store a sample in the buffer
void ClosedLoop::CollectSample() noexcept
{
	size_t wp = sampleBufferWritePointer;						// capture volatile variable and don't update it until all data has been written
	if (wp == sampleBufferReadPointer && samplesSent != samplesCollected)
	{
		sampleBufferOverflowed = true;							// the buffer is full so tell the sending task about it
		samplingMode = RecordingMode::SendingData;				// stop collecting data
	}
	else
	{
		sampleBuffer[wp++] = TickPeriodToTimePeriod(StepTimer::GetTimerTicks() - dataCollectionStartTicks);		// always collect this

		if (filterRequested & CL_RECORD_RAW_ENCODER_READING) 	{ sampleBuffer[wp++] = (float)encoder->GetCurrentCount(); }
		if (filterRequested & CL_RECORD_CURRENT_MOTOR_STEPS) 	{ sampleBuffer[wp++] = (float)encoder->GetCurrentCount() * encoder->GetStepsPerCount(); }
		if (filterRequested & CL_RECORD_TARGET_MOTOR_STEPS)  	{ sampleBuffer[wp++] = targetMotorSteps; }
		if (filterRequested & CL_RECORD_CURRENT_ERROR) 			{ sampleBuffer[wp++] = currentError; }
		if (filterRequested & CL_RECORD_PID_CONTROL_SIGNAL)  	{ sampleBuffer[wp++] = PIDControlSignal; }
		if (filterRequested & CL_RECORD_PID_P_TERM)  			{ sampleBuffer[wp++] = PIDPTerm; }
		if (filterRequested & CL_RECORD_PID_I_TERM)  			{ sampleBuffer[wp++] = PIDITerm; }
		if (filterRequested & CL_RECORD_PID_D_TERM)  			{ sampleBuffer[wp++] = PIDDTerm; }
		if (filterRequested & CL_RECORD_STEP_PHASE)  			{ sampleBuffer[wp++] = (float)encoder->GetCurrentPhasePosition(); }
		if (filterRequested & CL_RECORD_DESIRED_STEP_PHASE)  	{ sampleBuffer[wp++] = (float)desiredStepPhase; }
		if (filterRequested & CL_RECORD_PHASE_SHIFT)  			{ sampleBuffer[wp++] = phaseShift; }
		if (filterRequested & CL_RECORD_COIL_A_CURRENT) 		{ sampleBuffer[wp++] = (float)coilA; }
		if (filterRequested & CL_RECORD_COIL_B_CURRENT) 		{ sampleBuffer[wp++] = (float)coilB; }

		sampleBufferWritePointer = (wp >= sampleBufferLimit) ? 0 : wp;
		++samplesCollected;
		if (samplesCollected == samplesRequested)
		{
			samplingMode = RecordingMode::SendingData;			// stop collecting data
		}
	}

	dataTransmissionTask->Give();
}

inline void ClosedLoop::ReadState() noexcept
{
	if (encoder == nullptr) { return; }							// we can't read anything if there is no encoder

	// Calculate the current position & phase from the encoder reading
	const bool err = encoder->TakeReading();
	(void)err;		//TODO handle error
}

void ClosedLoop::ControlMotorCurrents(StepTimer::Ticks loopStartTime) noexcept
{
	// Get the time delta in seconds
	const float timeDelta = (float)(loopStartTime - prevControlLoopCallTime) * (1.0/(float)StepTimer::StepClockRate);

	// Use a PID controller to calculate the required 'torque' - the control signal
	// We choose to use a PID control signal in the range -256 to +256. This is rather arbitrary.
	PIDPTerm = Kp * currentError;
	PIDITerm = constrain<float>(PIDITerm + Ki * currentError * timeDelta, -PIDIlimit, PIDIlimit);	// constrain I to prevent it running away
	PIDDTerm = constrain<float>(Kd * derivativeFilter.GetDerivative(), -256.0, 256.0);		// constrain D so that we can graph it more sensibly after a sudden step input
	PIDControlSignal = constrain<float>(PIDPTerm + PIDITerm + PIDDTerm, -256.0, 256.0);		// clamp the sum between +/- 256

	// Calculate the offset required to produce the torque in the correct direction
	// i.e. if we are moving in the positive direction, we must apply currents with a positive phase shift
	// The max abs value of phase shift we want is 1 full step i.e. 25%.
	// Given that PIDControlSignal is -255 .. 255 and phase is 0 .. 4095
	// and that 25% of 4095 ~= 1024, our max phase shift ~= 4 * PIDControlSignal
	phaseShift = PIDControlSignal * 4.0;

	// New control algorithm:
	// - if the required phase shift is greater than about 15 degrees, apply it at maximum current
	// - below 15 degrees, keep the phase shift at +/- 15 degrees and reduce the current, but not below the minimum holding current
	// - after that, keep the current at the holding current and reduce the phase shift.
	float currentFraction;
	const float absPhaseShift = fabsf(phaseShift);
	if (absPhaseShift >= MinimumPhaseShift)
	{
		// Use the requested phase shift at full current
		currentFraction = 1.0;
	}
	else if (absPhaseShift >= holdCurrentFractionTimesMinPhaseShift)
	{
		// Use the minimum phase shift but reduce the current
		currentFraction = absPhaseShift * (1.0/MinimumPhaseShift);
		phaseShift = (phaseShift >= 0) ? MinimumPhaseShift : -MinimumPhaseShift;
	}
	else
	{
		// Reduce the phase shift and keep the current the same
		currentFraction = holdCurrentFraction;
		phaseShift *= recipHoldCurrentFraction;
	}

	// Calculate the required motor currents to induce that torque and reduce it modulo 4096
	// The following assumes that signed arithmetic is 2's complement
	const uint32_t measuredStepPhase = encoder->GetCurrentPhasePosition();
	desiredStepPhase = (uint16_t)((int32_t)measuredStepPhase + lrintf(phaseShift)) % 4096;

	// Assert the required motor currents
	SetMotorPhase(desiredStepPhase, currentFraction);
}

void ClosedLoop::Diagnostics(const StringRef& reply) noexcept
{
	reply.printf("Closed loop enabled: %s", closedLoopEnabled ? "yes" : "no");
	reply.catf(", pre-error threshold: %.2f, error threshold: %.2f", (double) errorThresholds[0], (double) errorThresholds[1]);
	reply.catf(", encoder type %s", GetEncoderType().ToString());
	if (encoder != nullptr)
	{
		reply.catf(", reverse polarity: %s", (encoder->IsBackwards()) ? "yes" : "no");
		if (encoder->TakeReading())
		{
			reply.cat(", error reading encoder\n");
		}
		else
		{
			reply.catf(", position %" PRIi32 "\n", encoder->GetCurrentCount());
		}
		encoder->AppendDiagnostics(reply);
	}

	// The rest is only relevant if we are in closed loop mode
	if (closedLoopEnabled)
	{
		reply.catf(", tuning mode: %#x, tuning error: %#x", tuning, tuningError);
		reply.catf(", collecting data: %s", CollectingData() ? "yes" : "no");
		if (CollectingData())
		{
			reply.catf(" (filter: %#x, mode: %u, rate: %u, movement: %u)", filterRequested, samplingMode, (unsigned int)(StepTimer::StepClockRate/dataCollectionIntervalTicks), movementRequested);
		}

#if 0	// DC disabled this because it doesn't work yet and the driver diagnostics were too long for the reply buffer
		reply.catf(", ultimateGain=%f, oscillationPeriod=%f", (double) ultimateGain, (double) oscillationPeriod);
#endif
		reply.lcatf("Control loop runtime (ms): min=%.3f, max=%.3f, frequency (Hz): min=%ld, max=%ld",
					(double) TickPeriodToTimePeriod(minControlLoopRuntime), (double)TickPeriodToTimePeriod(maxControlLoopRuntime),
					lrintf(TickPeriodToFreq(maxControlLoopCallInterval)), lrintf(TickPeriodToFreq(minControlLoopCallInterval)));

		ResetMonitoringVariables();
	}

	//DEBUG
	//reply.catf(", event status 0x%08" PRIx32 ", TCC2 CTRLA 0x%08" PRIx32 ", TCC2 EVCTRL 0x%08" PRIx32, EVSYS->CHSTATUS.reg, QuadratureTcc->CTRLA.reg, QuadratureTcc->EVCTRL.reg);
}

// TODO: Instead of having this take step, why not use the current DDA to calculate where we need to be?
void ClosedLoop::TakeStep() noexcept
{
# if SUPPORT_TMC2160 && SINGLE_DRIVER
	bool dummy;			// this receives the interpolation, but we don't use it
	const unsigned int microsteps = SmartDrivers::GetMicrostepping(0, dummy);
	const float microstepAngle = (microsteps == 0) ? 1.0 : 1.0/microsteps;
	targetMotorSteps += (stepDirection ? -microstepAngle : +microstepAngle);
	targetEncoderReading = lrintf(targetMotorSteps * encoder->GetCountsPerStep());

	if (samplingMode == RecordingMode::OnNextMove)
	{
		dataCollectionStartTicks = whenNextSampleDue = StepTimer::GetTimerTicks();
		samplingMode = RecordingMode::Immediate;
	}
# else
#  error Cannot support closed loop with the specified hardware
# endif
}

StandardDriverStatus ClosedLoop::ReadLiveStatus() noexcept
{
	StandardDriverStatus result;
	result.all = 0;
	result.closedLoopPositionNotMaintained = stall;
	result.closedLoopPositionWarning = preStall;
	result.closedLoopNotTuned = ((tuningError & encoder->MinimalTuningNeeded()) != 0);
	result.closedLoopTuningError = ((tuningError & TuningError::AnyTuningFailure) != 0);
	return result;
}

void ClosedLoop::SetStepDirection(bool dir) noexcept
{
	stepDirection = dir;
}

bool ClosedLoop::GetClosedLoopEnabled() noexcept
{
	return closedLoopEnabled;
}

void ClosedLoop::ResetError(size_t driver) noexcept
{
# if SINGLE_DRIVER
	if (driver == 0)
	{
		// Set the target position to the current position
		ReadState();
		derivativeFilter.Reset();
		targetEncoderReading = encoder->GetCurrentCount();
		targetMotorSteps = targetEncoderReading * encoder->GetStepsPerCount();
	}
# else
#  error Cannot support closed loop with the specified hardware
# endif
}

// This is called before the driver mode is changed. Return true if success. Always succeeds if we are disabling closed loop.
bool ClosedLoop::SetClosedLoopEnabled(size_t driver, bool enabled, const StringRef &reply) noexcept
{
	// Trying to enable closed loop
	if (driver != 0)
	{
		reply.copy("Invalid driver number");
		return false;
	}

	if (enabled && !closedLoopEnabled)
	{
		if (encoder == nullptr)
		{
			reply.copy("No encoder specified for closed loop drive mode");
			return false;
		}

		delay(10);														// delay long enough for the TMC driver to have read the microstep counter since the end of the last movement
		const uint16_t initialStepPhase = SmartDrivers::GetMicrostepPosition(0) * 4;	// get the current coil A microstep position as 0..4095

		// Temporarily calibrate the encoder zero position
		// We assume that the motor is at the position given by its microstep counter. This may not be true e.g. if it has a brake that has not been disengaged.
		ReadState();													// SetBackwards changes the reading
		if (!encoder->IsAbsolute())
		{
			encoder->SetBackwards(false);
			encoder->SetKnownPhaseAtCurrentCount(initialStepPhase);

			// Reset the tuning
			tuningError = encoder->MinimalTuningNeeded();
		}

		desiredStepPhase = initialStepPhase;							// set this to be picked up later in DriverSwitchedToClosedLoop
		PIDITerm = 0.0;
		derivativeFilter.Reset();
		targetEncoderReading = encoder->GetCurrentCount();
		targetMotorSteps = (float)targetEncoderReading / encoder->GetCountsPerStep();

		// Set the target position to the current position
		ResetError(0);													// this calls ReadState again and sets up targetMotorSteps

		ResetMonitoringVariables();										// to avoid getting stupid values
		prevControlLoopCallTime = StepTimer::GetTimerTicks();			// to avoid huge integral term windup
	}

	// If we are disabling closed loop mode, we should ideally send steps to get the microstep counter to match the current phase here
	closedLoopEnabled = enabled;

	return true;
}

// This is called just after the driver has switched into closed loop mode (it may have been in closed loop mode already)
void ClosedLoop::DriverSwitchedToClosedLoop(size_t driver) noexcept
{
	if (driver == 0)
	{
		delay(3);														// allow time for the switch to complete and a few control loop iterations to be done
		SetMotorPhase(desiredStepPhase, SmartDrivers::GetStandstillCurrentPercent(0) * 0.01);	// set the motor currents to match the initial position using the open loop standstill current
		PIDITerm = 0.0;													// clear the integral term accumulator
		derivativeFilter.Reset();
		ResetMonitoringVariables();										// the first loop iteration will have recorded a higher than normal loop call interval, so start again
	}
}

// If we are in closed loop modify the driver status appropriately
StandardDriverStatus ClosedLoop::ModifyDriverStatus(size_t driver, StandardDriverStatus originalStatus) noexcept
{
	if (closedLoopEnabled && driver == 0)
	{
		originalStatus.stall = 0;										// ignore stall detection in open loop mode
		originalStatus.standstill = 0;									// ignore standstill detection in closed loop mode
		originalStatus.closedLoopNotTuned = ((tuningError & encoder->MinimalTuningNeeded()) != 0);
		originalStatus.closedLoopIllegalMove = 0;						//TODO implement this or remove it
		originalStatus.closedLoopTuningError = ((tuningError & TuningError::AnyTuningFailure) != 0);
	}

	if (!originalStatus.closedLoopNotTuned)
	{
		// Report position warnings and errors even in open loop mode, if tuning has been done
		originalStatus.closedLoopPositionWarning = preStall;
		originalStatus.closedLoopPositionNotMaintained = stall;
	}

	return originalStatus;
}

#endif

// End
