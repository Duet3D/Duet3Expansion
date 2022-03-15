/*
 * ClosedLoop.cpp
 *
 *  Created on: 9 Jun 2020
 *      Author: David
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
# include <Platform.h>
# include <General/Bitmap.h>
# include <TaskPriorities.h>
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
uint8_t		ClosedLoop::tuning = 0;						// Bitmask of any tuning manoeuvres that have been requested
uint8_t		ClosedLoop::tuningError;					// Flags for any tuning errors
uint16_t	ClosedLoop::measuredStepPhase;				// The measured position of the motor
uint16_t	ClosedLoop::desiredStepPhase = 0;			// The desired position of the motor
int32_t		ClosedLoop::currentEncoderReading;			// The raw reading taken from the encoder
														// TODO no good for an extruder, use int32_t instead to count 1/256 microsteps and handle overflow
float 		ClosedLoop::encoderPulsePerStep;			// How many encoder readings do we get per step?

namespace ClosedLoop
{
	// Constants private to this module
	constexpr size_t TaskStackWords = 200;				// Size of the stack for all closed loop tasks
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

	constexpr float MaxSlopeMismatch = 0.1;				// we want the forward and reverse measured CPS each to be within 10% of the average
	constexpr float MinimumSlope = 7.5/1024;			// we require at least 8 transitions (8 pulses) per step, but we may measure slightly lower than the true value
	constexpr float MaxSafeHysteresis = 0.2;			// the maximum hysteresis in full steps that we are happy with - warn if there is more

	constexpr float PIDIlimit = 80.0;

	// Enumeration of closed loop recording modes
	enum RecordingMode : uint8_t
	{
		None = 0,			// not collecting data
		Immediate,			// collecting data now
		OnNextMove,			// collect data when the next movement command starts executing
		SendingData			// finished collecting data but still sending it to the main board
	};

	// Variables private to this module
	float	recipEncoderPulsesPerStep;					// Reciprocal of the encoder pulses per step, to avoid FP division when calculating the error

	// Control variables, set by the user to determine how the closed loop controller works
	bool 	closedLoopEnabled = false;					// Has closed loop been enabled by the user?
	uint8_t prevTuningError;							// Used to see what errors have been introduced by tuning

	// Holding current, and variables derived from it
	float 	holdCurrentFraction = DefaultHoldCurrentFraction;			// The minimum holding current when stationary
	float	recipHoldCurrentFraction = 1.0/DefaultHoldCurrentFraction;	// The reciprocal of the minimum holding current
	float	holdCurrentFractionTimesMinPhaseShift = MinimumPhaseShift * DefaultHoldCurrentFraction;

	int32_t	reversePolarityMultiplier = 1;				// +1 if encoder direction is forwards, -1 if it is reverse

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
	float	currentMotorSteps;							// The number of steps the motor has taken relative to it's zero position
	int32_t targetEncoderReading;						// The encoder reading we want, calculated from targetMotorSteps
	float 	currentError;								// The current error

	DerivativeAveragingFilter<derivativeFilterSize> derivativeFilter;	// An averaging filter to smooth the derivative of the error

	float 	PIDPTerm;									// Proportional term
	float 	PIDITerm = 0.0;								// Integral accumulator
	float 	PIDDTerm;									// Derivative term
	float	PIDControlSignal;							// The overall signal from the PID controller

	float	phaseShift;									// The desired shift in the position of the motor, where 1024 = 1 full step

	int16_t coilA;										// The current to run through coil A
	int16_t coilB;										// The current to run through coil A

	bool 	stall = false;								// Has the closed loop error threshold been exceeded?
	bool 	preStall = false;							// Has the closed loop warning threshold been exceeded?

	// Tuning variables
	struct TuningResults
	{
		float slope;
		float origin;
		float xMean;
		float revisedOrigin;

		// Calculate the revised origin based on a different slope, using the mid point of the move as the reference point
		void CalcRevisedOrigin(float slopeToUse) noexcept
		{
			revisedOrigin = origin + (slope - slopeToUse) * xMean;
		}

		// Print the values - used for debugging only
		void Print(const char *s, const StringRef& reply) const noexcept
		{
			reply.catf("%s: slope %.4f meanX %.1f origin %.2f revised origin %.2f\n", s, (double)slope, (double)xMean, (double)origin, (double)revisedOrigin);
		}
	};

	TuningResults forwardTuningResults, reverseTuningResults;
	float measuredCountsPerStep;
	float tuningHysteresis;
	StepTimer::Ticks whenLastTuningStepTaken;			// when the control loop last called the tuning code

#if BASIC_TUNING_DEBUG
	int32_t originalRawEncoderReading = 0;
	uint16_t originalDesiredStepPhase = 0, originalMeasuredStepPhase = 0;
	float originalCurrentMotorSteps = 0.0;
	float originalAssumedEncoderReading = 0.0, originalDesiredEncoderReading = 0.0;
	int32_t offsetCorrectionMade = 0;
	int32_t finalRawEncoderReading = 0;
	uint16_t finalMeasuredStepPhase = 0;
	float finalCurrentMotorSteps = 0.0;
#endif

	// The bitmask of a minimal tuning error for each encoder type
	// This is an array so that ZEROING_MANOEUVRE can be removed from the magnetic encoders if the LUT is in NVM
	uint8_t minimalTunes[5] =
	{
		// None
		0,
		// linearQuadrature
		TUNE_ERR_NOT_DONE_BASIC,
		// rotaryQuadrature
		TUNE_ERR_NOT_DONE_BASIC,
		// AS5047
		TUNE_ERR_NOT_DONE_BASIC | TUNE_ERR_NOT_CALIBRATED,
		// TLI5012
		TUNE_ERR_NOT_DONE_BASIC | TUNE_ERR_NOT_CALIBRATED,
	};

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
	void ReportTuningErrors(uint8_t tuningErrorBitmask, const StringRef& reply) noexcept;

	extern "C" [[noreturn]] void DataTransmissionLoop(void *param) noexcept;

}	// end namespace

// Tasks and task loops
static Task<ClosedLoop::TaskStackWords> *dataTransmissionTask;		// Data transmission task - handles sending back the buffered sample data

// Helper function to count the number of variables being collected by a given filter
static inline unsigned int CountVariablesCollected(uint16_t filter)
{
	return (new Bitmap<uint16_t>(filter))->CountSetBits() + 1;
}

// Helper function to convert a time period (expressed in StepTimer::Ticks) to ms
static inline float TickPeriodToTimePeriod(StepTimer::Ticks tickPeriod) {
	return tickPeriod * StepTimer::StepClocksToMillis;
}

// Helper function to convert a time period (expressed in StepTimer::Ticks) to a frequency in Hz
static inline float TickPeriodToFreq(StepTimer::Ticks tickPeriod) {
	return 1000.0l / TickPeriodToTimePeriod(tickPeriod);
}

// Helper function to reset the 'monitoring variables' as defined above
static void ResetMonitoringVariables() {
	ClosedLoop::minControlLoopRuntime = numeric_limits<StepTimer::Ticks>::max();
	ClosedLoop::maxControlLoopRuntime = numeric_limits<StepTimer::Ticks>::min();
	ClosedLoop::minControlLoopCallInterval = numeric_limits<StepTimer::Ticks>::max();
	ClosedLoop::maxControlLoopCallInterval = numeric_limits<StepTimer::Ticks>::min();
}

// Helper function to convert between the internal representation of encoderCountPerStep, and the appropriate external representation (e.g. CPR)
//TODO make this a virtual function member of the encoder?
float ClosedLoop::PulsePerStepToExternalUnits(float pps, uint8_t encoderType) noexcept {
	switch (encoderType)
	{
	case EncoderType::rotaryQuadrature:
		return pps / 4;															// Output count per step
	case EncoderType::AS5047:
		return (360.0 / ((AS5047D*) ClosedLoop::encoder)->GetMaxValue()) * pps;	// Output degree per step
	default:
		return pps;																// Output pulse per step
	}
}

//TODO make this a virtual function member of the encoder?
float ClosedLoop::ExternalUnitsToPulsePerStep(float externalUnits, uint8_t encoderType) noexcept {
	switch (encoderType)
	{
	case EncoderType::rotaryQuadrature:
		return externalUnits * 4;												// Input is count per step
	case EncoderType::AS5047:
		return (((AS5047D*) ClosedLoop::encoder)->GetMaxValue() / 360.0) * externalUnits;	// Input is degree per step
	default:
		return externalUnits;													// Input is pulse per step
	}
}

// Helper function to cat all the current tuning errors onto a reply in human-readable form
void ClosedLoop::ReportTuningErrors(uint8_t tuningErrorBitmask, const StringRef &reply)
{
	if (tuningErrorBitmask & TUNE_ERR_NOT_DONE_BASIC) 		{ reply.cat(" The drive has not had basic tuning done."); }
	if (tuningErrorBitmask & TUNE_ERR_NOT_CALIBRATED) 		{ reply.cat(" The drive has not been calibrated."); }
	if (tuningErrorBitmask & TUNE_ERR_SYSTEM_ERROR) 		{ reply.cat(" A system error occurred while tuning."); }
	if (tuningErrorBitmask & TUNE_ERR_INCONSISTENT_MOTION)	{ reply.cat(" The measured motion was inconsistent."); }
	if (tuningErrorBitmask & TUNE_ERR_TOO_LITTLE_MOTION)	{ reply.catf(" The measured motion was less than expected; counts/step is about %.2f.", (double)(measuredCountsPerStep * 0.25)); }
	if (tuningErrorBitmask & TUNE_ERR_TOO_MUCH_MOTION)		{ reply.catf(" The measured motion was more than expected; counts/step is about %.2f.", (double)(measuredCountsPerStep * 0.25)); }
}

// Helper function to set the motor to a given phase and magnitude
// The phase is normally in the range 0 to 4095 but when tuning it can be 0 to somewhat over 8192. We must take it modulo 4096 when computing the currents.
void ClosedLoop::SetMotorPhase(uint16_t phase, float magnitude) noexcept
{
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

	// Initialise to no error thresholds
	errorThresholds[0] = 0;
	errorThresholds[1] = 0;

	// Initialise the monitoring variables
	ResetMonitoringVariables();

	derivativeFilter.Reset();

	// Set up the data transmission task
	dataTransmissionTask = new Task<ClosedLoop::TaskStackWords>;
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
	size_t numThresholds = 2;
	float tempErrorThresholds[numThresholds];
	float holdingCurrentPercent;

	// Pull changed parameters
	uint8_t seen = 0;
	seen |= parser.GetUintParam('T', tempEncoderType) 			<< 0;
	seen |= parser.GetFloatParam('C', tempCPR) 					<< 1;
	seen |= parser.GetFloatParam('R', tempKp) 					<< 2;
	seen |= parser.GetFloatParam('I', tempKi) 					<< 3;
	seen |= parser.GetFloatParam('D', tempKd) 					<< 4;
	seen |= parser.GetFloatArrayParam('E', numThresholds, tempErrorThresholds) << 5;
	seen |= parser.GetFloatParam('H', holdingCurrentPercent)	<< 6;

	// Report back if !seen
	if (seen == 0)
	{
		reply.catf("Encoder type: %s", GetEncoderType().ToString());
		//TODO add a virtual function member to the encoder, to return the units string?
		const char* const units = (tempEncoderType == EncoderType::rotaryQuadrature) ? "encoder pulses/step"
									: (tempEncoderType == EncoderType::AS5047) ? "motor degrees/step"
										: "encoder CPR";
		reply.catf(", %s: %.2f", units, (double)PulsePerStepToExternalUnits(encoderPulsePerStep, tempEncoderType));

		if (encoder != nullptr)
		{
			encoder->AppendStatus(reply);
		}
		reply.catf(", PID parameters P=%.3f I=%.3f D=%.3f, min. current %.1f%%", (double) Kp, (double) Ki, (double) Kd, (double)(holdCurrentFraction * 100.0));
		return GCodeResult::ok;
	}

	// Validate the new params
	if (tempEncoderType > EncoderType::NumValues)
	{
		reply.copy("Invalid T value. Valid values are 0 and 1");
		return GCodeResult::error;
	}
	if ((seen & (0x1 << 5)) && (tempErrorThresholds[0] < 0 || tempErrorThresholds[1] < 0))
	{
		reply.copy("Error threshold value must be greater than zero");
		return GCodeResult::error;
	}
	if ((seen & (0x1 << 1)) && tempCPR < 1.0)
	{
		reply.copy("Encoder counts per step most be positive");
		return GCodeResult::error;
	}

	// Set the new params
	TaskCriticalSectionLocker lock;			// don't allow the closed loop task to see an inconsistent combination of these values

	if (seen & (0x1 << 1))
	{
		// Convert external units to internal units
		encoderPulsePerStep = ExternalUnitsToPulsePerStep(tempCPR, tempEncoderType);
		recipEncoderPulsesPerStep = 1.0/encoderPulsePerStep;
	}

	Kp = tempKp;
	Ki = tempKi;
	Kd = tempKd;
	PIDITerm = 0;

	if (seen & (1u << 6))
	{
		holdCurrentFraction = constrain<float>(holdingCurrentPercent, 10.0, 100.0) / 100.0;
		holdCurrentFractionTimesMinPhaseShift = MinimumPhaseShift * holdCurrentFraction;
		recipHoldCurrentFraction = 1.0/holdCurrentFraction;
	}

	if (seen & (0x1 << 5)) {
		errorThresholds[0] = tempErrorThresholds[0];
		errorThresholds[1] = tempErrorThresholds[1];
	}

	// If encoder count per steps has changed, we need to re-tune
	if ((seen & (0x1 << 1)) && encoder != nullptr) {
		tuningError = minimalTunes[encoder->GetType().ToBaseType()];
	}

	//TODO need to get a lock here in case there is any movement
	if (seen & (0x1 << 0)) {
		if (tempEncoderType == GetEncoderType().ToBaseType())
		{
			if (encoder != nullptr)
			{
				encoder->Disable();
				return encoder->Init(reply);
			}
		}
		else
		{
			DeleteObject(encoder);
			switch (tempEncoderType)
			{
			case EncoderType::none:
			default:
				encoder = nullptr;
				break;

			case EncoderType::AS5047:
				encoder = new AS5047D(*Platform::sharedSpi, EncoderCsPin);
				if (((AS5047D*) encoder)->LoadLUT()) {
					minimalTunes[EncoderType::AS5047] &= ~TUNE_ERR_NOT_CALIBRATED;
				}
				break;

			case EncoderType::TLI5012:
				encoder = new TLI5012B(*Platform::sharedSpi, EncoderCsPin);
				break;

			case EncoderType::linearQuadrature:
				encoder = new QuadratureEncoderPdec();
				break;

			case EncoderType::rotaryQuadrature:
				// TODO: Debug why this can't be set to rotary mode
				encoder = new QuadratureEncoderPdec();
				break;
			}
		}

		if (encoder != nullptr)
		{
			tuningError = minimalTunes[encoder->GetType().ToBaseType()];
			return encoder->Init(reply);
		}
	}

	return GCodeResult::ok;
}

static float startRecordingTrigger;

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
	startRecordingTrigger = targetMotorSteps;					// mark the current number of steps in case we are using RecordingMode::OnNextMove
	dataCollectionIntervalTicks = (msg.rate == 0) ? 1 : StepTimer::StepClockRate/msg.rate;
	dataCollectionStartTicks = whenNextSampleDue = StepTimer::GetTimerTicks();
	samplingMode = (RecordingMode)requestedMode;				// do this one last, it triggers data collection

	StartTuning(msg.movement);
	return GCodeResult::ok;
}

GCodeResult ClosedLoop::ProcessM569Point6(const CanMessageGeneric &msg, const StringRef &reply) noexcept
{
	CanMessageGenericParser parser(msg, M569Point6Params);

	uint8_t desiredTuning;
	if (!parser.GetUintParam('V', desiredTuning))
	{
		// We have been called to return the status after the previous call returned "not finished"
		if (tuning != 0)
		{
			return GCodeResult::notFinished;
		}

#if BASIC_TUNING_DEBUG
		forwardTuningResults.Print("Forward", reply);
		reverseTuningResults.Print("Reverse", reply);
#endif

		// Tuning has finished - there are now 3 scenarios
		// 1. No tuning errors exist (!tuningError)							= OK
		// 2. No new tuning errors exist !(~prevTuningError & tuningError)	= WARNING
		// 3. A new tuning error has been introduced (else)					= WARNING
		if (tuningError == 0)
		{
#if BASIC_TUNING_DEBUG
			reply.catf("OER %" PRIi32 " AER %.1f DER %.1f DSP %u OMSP %u OCMS %.3f\n",
						originalRawEncoderReading, (double)originalAssumedEncoderReading, (double)originalDesiredEncoderReading,
							originalDesiredStepPhase, originalMeasuredStepPhase, (double)originalCurrentMotorSteps);
			reply.catf("OCM %" PRIi32 " FER %" PRIi32 " FMSP %u FCMS %.3f\n",
						offsetCorrectionMade, finalRawEncoderReading, finalMeasuredStepPhase, (double)finalCurrentMotorSteps);
#endif
			reply.catf("Driver %u.0 tuned successfully, measured hysteresis %.2f step", CanInterface::GetCanAddress(), (double)tuningHysteresis);
			if (tuningHysteresis <= MaxSafeHysteresis)
			{
				return GCodeResult::ok;
			}
			reply.lcat("Measured hysteresis is high");
			return GCodeResult::warning;
		}

		// Tuning failed so report the errors
		if ((~prevTuningError & tuningError) != 0)
		{
			reply.catf("Driver %u.0 new tuning error(s):", CanInterface::GetCanAddress());
			ReportTuningErrors(~prevTuningError & tuningError, reply);
		}
		if ((prevTuningError & tuningError) != 0)
		{
			reply.lcatf("Driver %u.0 un-cleared previous tuning error(s):", CanInterface::GetCanAddress());
			ReportTuningErrors(prevTuningError & tuningError, reply);
		}

		return GCodeResult::warning;
	}

	// Here if this is a new command to start a tuning move
	// Check we are in direct drive mode
	if (SmartDrivers::GetDriverMode(0) != DriverMode::direct)
	{
		reply.copy("Drive is not in closed loop mode");
		return GCodeResult::error;
	}

#if 0	//TODO when we have redefined the V parameter values, validate the parameter
	// Validate the V parameter
	if (desiredTuning > FULL_TUNE)
	{
		reply.printf("Invalid 'V' parameter value. V may be 0-%d", FULL_TUNE);
		return GCodeResult::error;
	}
#endif

	prevTuningError = tuningError;
	StartTuning(desiredTuning);

	return GCodeResult::notFinished;
}

void ClosedLoop::StartTuning(uint8_t tuningMode) noexcept
{
	if (tuningMode != 0)
	{
		Platform::DriveEnableOverride(0, true);					// enable the motor and prevent it becoming idle
		whenLastTuningStepTaken = StepTimer::GetTimerTicks() + stepTicksBeforeTuning;	// delay the start to allow brake release and motor current buildup
		if (tuningMode & ENCODER_CALIBRATION_MANOEUVRE)
		{
			tuningMode |= BASIC_TUNING_MANOEUVRE;				// always run basic tuning before encoder calibration
		}
		tuning = tuningMode;
	}
}

void ClosedLoop::SetForwardPolarity() noexcept
{
	reversePolarityMultiplier = 1;
}

void ClosedLoop::SaveBasicTuningResult(float slope, float origin, float xMean, bool reverse) noexcept
{
	TuningResults& r = (reverse) ? reverseTuningResults : forwardTuningResults;
	r.slope = slope;
	r.origin = origin;
	r.xMean = xMean;
}

// Call this when we have stopped basic tuning movement and are ready to switch to closed loop control
void ClosedLoop::FinishedBasicTuning() noexcept
{
	// Check that the forward and reverse slopes are similar and a good match to the configured counts per step
	const float averageSlope = (forwardTuningResults.slope + reverseTuningResults.slope) * 0.5;
	// We sometimes read different forwards and reverse counts, so instead of taking an average of the origin, average the origin w.r.t. the mid points of the tuning moves
	forwardTuningResults.CalcRevisedOrigin(averageSlope);
	reverseTuningResults.CalcRevisedOrigin(averageSlope);

	measuredCountsPerStep = fabsf(averageSlope) * 1024;
	tuningHysteresis = fabsf(forwardTuningResults.revisedOrigin - reverseTuningResults.revisedOrigin)/measuredCountsPerStep;

	if (fabsf(averageSlope) < MinimumSlope || fabsf(forwardTuningResults.slope - reverseTuningResults.slope) > MaxSlopeMismatch * 2 * fabsf(averageSlope))
	{
		tuningError |= TUNE_ERR_INCONSISTENT_MOTION;
	}
	else if (measuredCountsPerStep > encoderPulsePerStep * 1.05)
	{
		tuningError |= TUNE_ERR_TOO_MUCH_MOTION;
	}
	else if (measuredCountsPerStep < encoderPulsePerStep * 0.95)
	{
		tuningError |= TUNE_ERR_TOO_LITTLE_MOTION;
	}
	else
	{
		// Tuning succeeded
#if BASIC_TUNING_DEBUG
		ReadState();
		originalRawEncoderReading = rawEncoderReading;
		originalDesiredStepPhase = desiredStepPhase;
		originalMeasuredStepPhase = measuredStepPhase;
		originalCurrentMotorSteps = currentMotorSteps;
#endif
		reversePolarityMultiplier = (averageSlope < 0.0) ? -1 : 1;

		if (encoder->GetPositioningType() == EncoderPositioningType::relative)
		{
			const float averageOffset = (forwardTuningResults.revisedOrigin + reverseTuningResults.revisedOrigin) * 0.5;
			((RelativeEncoder*)encoder)->SetOffset(-lrintf(averageOffset));					// in future, subtract this offset so that zero reading means zero phase
			targetEncoderReading = lrintf(desiredStepPhase * fabsf(averageSlope));			// the encoder reading we ought to be getting now if there is no hysteresis
			targetMotorSteps = currentMotorSteps = targetEncoderReading / encoderPulsePerStep;
#if BASIC_TUNING_DEBUG
			originalAssumedEncoderReading = desiredStepPhase * averageSlope + averageOffset;
			originalDesiredEncoderReading = desiredEncoderReading;
			offsetCorrectionMade = -lrintf(averageOffset);
			ReadState();
			finalRawEncoderReading = rawEncoderReading;
			finalMeasuredStepPhase = measuredStepPhase;
			finalCurrentMotorSteps = currentMotorSteps;
#endif
		}

		tuningError &= ~(TUNE_ERR_TOO_MUCH_MOTION | TUNE_ERR_TOO_LITTLE_MOTION | TUNE_ERR_INCONSISTENT_MOTION);
	}

	tuningError &= ~TUNE_ERR_NOT_DONE_BASIC;
}

// This is called by tuning to execute a step
void ClosedLoop::AdjustTargetMotorSteps(float amount) noexcept
{
	targetMotorSteps += amount;
	targetEncoderReading = lrintf(targetMotorSteps * encoderPulsePerStep);
}

void ClosedLoop::ControlLoop() noexcept
{
	// Record the control loop call interval
	const StepTimer::Ticks loopCallTime = StepTimer::GetTimerTicks();
	const StepTimer::Ticks timeElapsed = loopCallTime - prevControlLoopCallTime;
	minControlLoopCallInterval = min<StepTimer::Ticks>(minControlLoopCallInterval, timeElapsed);
	maxControlLoopCallInterval = max<StepTimer::Ticks>(maxControlLoopCallInterval, timeElapsed);

	// Read the current state of the drive
	ReadState();

	// Calculate and store the current error
	currentError = (float)(targetEncoderReading - currentEncoderReading) * recipEncoderPulsesPerStep;
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
			if (tuning == 0)
			{
				Platform::DriveEnableOverride(0, false);			// If that was the last tuning move, release the override
			}
		}
		else if (samplingMode == RecordingMode::OnNextMove && timeSinceLastTuningStep + (int32_t)DataCollectionIdleStepTicks >= 0)
		{
			dataCollectionStartTicks = whenNextSampleDue = loopCallTime;
			samplingMode = RecordingMode::Immediate;
		}
	}
	else if (tuningError)
	{
		// Don't do anything if there is a tuning error
	}
	else
	{
		ControlMotorCurrents(loopCallTime);							// otherwise control those motor currents!
	}

	// Look for a stall or pre-stall
	preStall = errorThresholds[0] > 0 && fabsf(currentError) > errorThresholds[0];
	const bool alreadyStalled = stall;
	stall 	 = errorThresholds[1] > 0 && fabsf(currentError) > errorThresholds[1];
	if (stall && !alreadyStalled)
	{
		Platform::NewDriverFault();
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

		if (filterRequested & CL_RECORD_RAW_ENCODER_READING) 	{sampleBuffer[wp++] = (float)currentEncoderReading;}
		if (filterRequested & CL_RECORD_CURRENT_MOTOR_STEPS) 	{sampleBuffer[wp++] = currentMotorSteps;}
		if (filterRequested & CL_RECORD_TARGET_MOTOR_STEPS)  	{sampleBuffer[wp++] = targetMotorSteps;}
		if (filterRequested & CL_RECORD_CURRENT_ERROR) 			{sampleBuffer[wp++] = currentError;}
		if (filterRequested & CL_RECORD_PID_CONTROL_SIGNAL)  	{sampleBuffer[wp++] = PIDControlSignal;}
		if (filterRequested & CL_RECORD_PID_P_TERM)  			{sampleBuffer[wp++] = PIDPTerm;}
		if (filterRequested & CL_RECORD_PID_I_TERM)  			{sampleBuffer[wp++] = PIDITerm;}
		if (filterRequested & CL_RECORD_PID_D_TERM)  			{sampleBuffer[wp++] = PIDDTerm;}
		if (filterRequested & CL_RECORD_STEP_PHASE)  			{sampleBuffer[wp++] = (float)measuredStepPhase;}
		if (filterRequested & CL_RECORD_DESIRED_STEP_PHASE)  	{sampleBuffer[wp++] = (float)desiredStepPhase;}
		if (filterRequested & CL_RECORD_PHASE_SHIFT)  			{sampleBuffer[wp++] = phaseShift;}
		if (filterRequested & CL_RECORD_COIL_A_CURRENT) 		{sampleBuffer[wp++] = (float)coilA;}
		if (filterRequested & CL_RECORD_COIL_B_CURRENT) 		{sampleBuffer[wp++] = (float)coilB;}

		sampleBufferWritePointer = (wp >= sampleBufferLimit) ? 0 : wp;
		++samplesCollected;
		if (samplesCollected == samplesRequested)
		{
			samplingMode = RecordingMode::SendingData;			// stop collecting data
		}
	}

	dataTransmissionTask->Give();
}

void ClosedLoop::ReadState() noexcept
{
	if (encoder == nullptr) { return; }							// we can't read anything if there is no encoder

	// Calculate the current position & phase from the encoder reading
	currentEncoderReading = encoder->GetReading() * reversePolarityMultiplier;
	currentMotorSteps = (float)currentEncoderReading / encoderPulsePerStep;

	// Calculate stepPhase - a 0-4095 value representing the phase *within* the current 4 full steps
	const float tmp = currentMotorSteps * 0.25;
	measuredStepPhase = (uint16_t)((tmp - floorf(tmp)) * 4095.9);
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

	// Calculate the required motor currents to induce that torque and reduce it module 4096
	// The following assumes that signed arithmetic is 2's complement
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
		reply.catf(", reverse polarity: %s", (reversePolarityMultiplier < 0) ? "yes" : "no");
		reply.catf(", position %" PRIi32, encoder->GetReading());
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
	targetEncoderReading = lrintf(targetMotorSteps * encoderPulsePerStep);

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
	result.closedLoopNotTuned = ((tuningError & minimalTunes[encoder->GetType().ToBaseType()]) != 0);
	result.closedLoopTuningError = ((tuningError & TUNE_ERR_TUNING_FAILURE) != 0);
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
	if (driver == 0) {
		// Set the target position to the current position
		ReadState();
		derivativeFilter.Reset();
		targetMotorSteps = currentMotorSteps;
		targetEncoderReading = currentEncoderReading;
	}
# else
#  error Cannot support closed loop with the specified hardware
# endif
}

// This is called before the driver mode is changed. Return true if success.
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

		delay(3);														// delay long enough for the TMC driver to have read the microstep counter since the end of the last movement
		const uint16_t initialStepPhase = SmartDrivers::GetMicrostepPosition(0) * 4;	// get the current coil A microstep position as 0..4095
		reversePolarityMultiplier = 1;									// assume the encoder reads forwards
		if (encoder->GetPositioningType() == EncoderPositioningType::relative)
		{
			// Temporarily calibrate the encoder zero position
			// We assume that the motor is at the position given by its microstep counter. This may not be true e.g. if it has a brake.
			ReadState();												// set up currentMotorSteps and measuredStepPhase
			((RelativeEncoder*)encoder)->SetOffset(lrintf(((int32_t)initialStepPhase - (int32_t)measuredStepPhase) * encoderPulsePerStep / 1024.0));	// set the new zero position
		}

		desiredStepPhase = initialStepPhase;							// set this to be picked up later in DriverSwitchedToClosedLoop

		// Set the target position to the current position
		ResetError(0);													// this calls ReadState again and sets up targetMotorSteps

		// Reset the tuning (We have already checked encoder != nullptr)
		tuningError = minimalTunes[encoder->GetType().ToBaseType()];

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
		originalStatus.closedLoopNotTuned = ((tuningError & minimalTunes[encoder->GetType().ToBaseType()]) != 0);
		originalStatus.closedLoopIllegalMove = 0;						//TODO implement this or remove it
		originalStatus.closedLoopTuningError = ((tuningError & TUNE_ERR_TUNING_FAILURE) != 0);
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
