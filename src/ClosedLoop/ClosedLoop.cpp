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

// Variables that are used by both the ClosedLoop and the Tuning modules

Encoder*	ClosedLoop::encoder = nullptr;				// Pointer to the encoder object in use
uint8_t		ClosedLoop::tuning = 0;						// Bitmask of any tuning manoeuvres that have been requested
uint8_t		ClosedLoop::tuningError;					// Flags for any tuning errors
uint16_t	ClosedLoop::stepPhase;						// The current position of the motor
uint16_t	ClosedLoop::desiredStepPhase = 0;			// The desired position of the motor
atomic<int32_t> ClosedLoop::rawEncoderReading;			// The raw reading taken from the encoder
atomic<float> 	ClosedLoop::targetMotorSteps = 0;		// The number of steps the motor should have taken relative to it's zero position
														// TODO no good for an extruder, use int32_t instead to count 1/256 microsteps and handle overflow
float 		ClosedLoop::encoderPulsePerStep;			// How many encoder readings do we get per step?
float 		ClosedLoop::currentMotorSteps;				// The number of steps the motor has taken relative to it's zero position

namespace ClosedLoop
{
	// Constants private to this module
	constexpr size_t TaskStackWords = 200;				// Size of the stack for all closed loop tasks
	constexpr unsigned int derivativeFilterSize = 16;	// The range of the derivative filter (use a power of 2 for efficiency)
	constexpr unsigned int DataBufferSize = 2000 * 14;	// When collecting samples we can accommodate 2000 readings of up to 13 variables + timestamp
	constexpr unsigned int tuningStepsPerSecond = 2000;	// the rate at which we send 1/256 microsteps during tuning, slow enough for high-inertia motors
	constexpr StepTimer::Ticks stepTicksPerTuningStep = StepTimer::StepClockRate/tuningStepsPerSecond;
	constexpr StepTimer::Ticks stepTicksBeforeTuning = StepTimer::StepClockRate/10;
														// 1/10 sec delay between enabling the driver and starting tuning, to allow for brake release and current buildup
	constexpr StepTimer::Ticks DataCollectionIdleStepTicks = StepTimer::StepClockRate/200;
														// start collecting tuning data 5ms before the start of the tuning move

	// Enumeration of closed loop recording modes
	enum RecordingMode : uint8_t
	{
		None = 0,			// not collecting data
		Immediate,			// collecting data now
		OnNextMove,			// collect data when the next movement command starts executing
		SendingData			// finished collecting data but still sending it to the main board
	};

	// Variables private to this module

	// Control variables, set by the user to determine how the closed loop controller works
	bool 	closedLoopEnabled = false;					// Has closed loop been enabled by the user?
	uint8_t prevTuningError;							// Used to see what errors have been introduced by tuning

	float 	holdCurrent = 0;							// The minimum holding current when stationary
	float	reversePolarityMultiplier = 1.0;			// +1 if encoder direction is forwards, -1 if it is reverse

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
	float 	currentError;								// The current error
	float 	lastError = 0;								// The error from the previous iteration

	DerivativeAveragingFilter<derivativeFilterSize> derivativeFilter;	// An averaging filter to smooth the derivative of the error

	float 	PIDPTerm;									// Proportional term
	float 	PIDITerm = 0;								// Integral accumulator
	float 	PIDDTerm;									// Derivative term
	int16_t PIDControlSignal;							// The overall -255 to 255 signal from the PID controller

	int16_t phaseShift;									// The desired shift in the position of the motor

	int16_t coilA;										// The current to run through coil A
	int16_t coilB;										// The current to run through coil A

	bool 	stall = false;								// Has the closed loop error threshold been exceeded?
	bool 	preStall = false;							// Has the closed loop warning threshold been exceeded?

	// The bitmask of a minimal tuning error for each encoder type
	// This is an array so that ZEROING_MANOEUVRE can be removed from the magnetic encoders if the LUT is in NVM
	uint8_t minimalTunes[5] = {
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
	StepTimer::Ticks minControlLoopRuntime;				// The minimum time the control loop has taken to run
	StepTimer::Ticks maxControlLoopRuntime;				// The maximum time the control loop has taken to run
	float 			 ewmaControlLoopRuntime;			// The exponentially weighted moving average (ewma) time the control loop has taken

	StepTimer::Ticks prevControlLoopCallTime;			// The last time the control loop was called
	StepTimer::Ticks minControlLoopCallInterval;		// The minimum interval between the control loop being called
	StepTimer::Ticks maxControlLoopCallInterval;		// The maximum interval between the control loop being called
	float 			 ewmaControlLoopCallInterval;		// An ewma of the frequency the control loop is called at

	StepTimer::Ticks whenLastTuningStepTaken;			// when we last called the tuning code

	// Functions private to this module
	EncoderType GetEncoderType() noexcept
	{
		return (encoder == nullptr) ? EncoderType::none : encoder->GetType();
	}

	// Return true if we are currently collecting data or primed to collect data or finishing sending data
	inline bool CollectingData() noexcept { return samplingMode != RecordingMode::None; }

	void ReadState(StepTimer::Ticks now) noexcept;
	void CollectSample() noexcept;
	void ControlMotorCurrents(StepTimer::Ticks loopStartTime) noexcept;
	void StartTuning(uint8_t tuningType) noexcept;

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
	ClosedLoop::ewmaControlLoopRuntime = 0;
	ClosedLoop::minControlLoopCallInterval = numeric_limits<StepTimer::Ticks>::max();
	ClosedLoop::maxControlLoopCallInterval = numeric_limits<StepTimer::Ticks>::min();
	ClosedLoop::ewmaControlLoopCallInterval = 0;
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
static void ReportTuningErrors(uint8_t tuningErrorBitmask, const StringRef &reply)
{
	if (tuningErrorBitmask & ClosedLoop::TUNE_ERR_NOT_DONE_BASIC) 		{reply.catf(" The drive has not had basic tuning done.");}
	if (tuningErrorBitmask & ClosedLoop::TUNE_ERR_NOT_CALIBRATED) 		{reply.catf(" The drive has not been calibrated.");}
	if (tuningErrorBitmask & ClosedLoop::TUNE_ERR_SYSTEM_ERROR) 		{reply.catf(" A system error occurred while tuning.");}
	if (tuningErrorBitmask & ClosedLoop::TUNE_ERR_TOO_LITTLE_MOTION)	{reply.catf(" The measured motion was less than expected.");}
	if (tuningErrorBitmask & ClosedLoop::TUNE_ERR_TOO_MUCH_MOTION)		{reply.catf(" The measured motion was more than expected.");}
}

// Helper function to set the motor to a given phase and magnitude
void ClosedLoop::SetMotorPhase(uint16_t phase, float magnitude) noexcept
{
	magnitude = constrain<float>(magnitude, holdCurrent, 1.0);
	coilA = 255 * Trigonometry::FastCos(phase) * magnitude;
	coilB = 255 * Trigonometry::FastSin(phase) * reversePolarityMultiplier * magnitude;

# if SUPPORT_TMC2160 && SINGLE_DRIVER
	SmartDrivers::SetRegister(0,
			SmartDriverRegister::xDirect,
			((coilB << 16) & 0x01FF0000) | (coilA & 0x000001FF));
# else
#  error Cannot support closed loop with the specified hardware
# endif
}

static void GenerateTmcClock()
{
	// Currently we program DPLL0 to generate 120MHz output, so to get 15MHz we divide by 8
	ConfigureGclk(ClockGenGclkNumber, GclkSource::dpll0, 8, true);
	SetPinFunction(ClockGenPin, ClockGenPinPeriphMode);
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
	float tempCPR = encoderPulsePerStep;		// TODO: Use countPerStepToExternalUnits() here
	float tempKp = Kp;
	float tempKi = Ki;
	float tempKd = Kd;
	size_t numThresholds = 2;
	float tempErrorThresholds[numThresholds];

	// Pull changed parameters
	uint8_t seen = 0;
	seen |= parser.GetUintParam('T', tempEncoderType) 	<< 0;
	seen |= parser.GetFloatParam('C', tempCPR) 			<< 1;
	seen |= parser.GetFloatParam('R', tempKp) 			<< 2;
	seen |= parser.GetFloatParam('I', tempKi) 			<< 3;
	seen |= parser.GetFloatParam('D', tempKd) 			<< 4;
	seen |= parser.GetFloatArrayParam('E',
				numThresholds,
				tempErrorThresholds) 					<< 5;

	// Report back if !seen
	if (!seen) {
		reply.catf("Encoder type: %s", GetEncoderType().ToString());
		//TODO add a virtual function member to the encoder, to return the units string?
		const char* const units = (tempEncoderType == EncoderType::rotaryQuadrature) ? "encoder pulses/step"
									: (tempEncoderType == EncoderType::AS5047) ? "motor degrees/step"
										: "encoder CPR";
		reply.catf(", %s: %.1f", units, (double)PulsePerStepToExternalUnits(tempCPR, tempEncoderType));

		if (encoder != nullptr)
		{
			encoder->AppendStatus(reply);
		}
		reply.catf(", PID parameters: P=%.3f, I=%.3f, D=%.3f", (double) Kp, (double) Ki, (double) Kd);
		return GCodeResult::ok;
	}

	// Validate the new params
	if (tempEncoderType > EncoderType::NumValues) {reply.copy("Invalid T value. Valid values are 0 and 1"); return GCodeResult::error;}
	if ((seen & (0x1 << 5)) && tempErrorThresholds[0] < 0) {reply.copy("Error threshold value must be greater than zero"); return GCodeResult::error;}
	if ((seen & (0x1 << 5)) && tempErrorThresholds[1] < 0) {reply.copy("Error threshold value must be greater than zero"); return GCodeResult::error;}

	// Convert external units to internal units
	if (seen & (0x1 << 1)) {
		tempCPR = ExternalUnitsToPulsePerStep(tempCPR, tempEncoderType);
	}

	// Set the new params
	encoderPulsePerStep = tempCPR;
	Kp = tempKp;
	Ki = tempKi;
	Kd = tempKd;
	PIDITerm = 0;

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

#if 0	//TODO when we have redefined the V parameter values, validate the parameter
	// Validate the V parameter
	if (msg.movement > FULL_TUNE)
	{
		reply.printf("Maximum value for V is %u. V%u is invalid", FULL_TUNE, msg.movement);
		return GCodeResult::error;
	}
#endif

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

		// Tuning has finished - there are now 3 scenarios
		// 1. No tuning errors exist (!tuningError)							= OK
		// 2. No new tuning errors exist !(~prevTuningError & tuningError)	= WARNING
		// 3. A new tuning error has been introduced (else)					= WARNING
		if (tuningError == 0)
		{
			reply.copy("Tuning completed successfully");
			return GCodeResult::ok;
		}

		if ((~prevTuningError & tuningError) == 0)
		{
			reply.copy("No new tuning errors have been found, but some previous tuning errors were not cleared");
			ReportTuningErrors(tuningError, reply);
		}
		else
		{
			reply.copy("One or more tuning errors occurred");
			ReportTuningErrors(~prevTuningError & tuningError, reply);
			if (prevTuningError & tuningError)
			{
				reply.catf(" In addition, the following tuning errors were already present");
				ReportTuningErrors(prevTuningError & tuningError, reply);
			}
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
	reversePolarityMultiplier = 1.0;
}

void ClosedLoop::SetBasicTuningResults(float forwardCountsPerStep, float ReverseCountsPerStep, int32_t finalReading) noexcept
{
	float averageCountsPerStep = (forwardCountsPerStep + ReverseCountsPerStep) * 0.5;
	if (averageCountsPerStep < 0.0)
	{
		averageCountsPerStep = -averageCountsPerStep;
		reversePolarityMultiplier = -1.0;
	}
	else
	{
		reversePolarityMultiplier = 1.0;
	}

	if (averageCountsPerStep > encoderPulsePerStep * 1.1)
	{
		tuningError |= TUNE_ERR_TOO_MUCH_MOTION;
	}
	else if (averageCountsPerStep < encoderPulsePerStep * 0.9)
	{
		tuningError |= TUNE_ERR_TOO_LITTLE_MOTION;
	}
	else
	{
		tuningError &= ~(TUNE_ERR_TOO_MUCH_MOTION | TUNE_ERR_TOO_LITTLE_MOTION);
	}

	if (encoder->GetPositioningType() == EncoderPositioningType::relative)
	{
		// Set the new zero position
		((RelativeEncoder*)encoder)->SetOffset(-finalReading);
	}

	tuningError &= ~TUNE_ERR_NOT_DONE_BASIC;
}

void ClosedLoop::ResetStepPosition(uint16_t motorPhase) noexcept
{
	currentMotorSteps = targetMotorSteps = (float)motorPhase/4096;
}

void ClosedLoop::ControlLoop() noexcept
{
	// Record the control loop call interval
	const StepTimer::Ticks loopCallTime = StepTimer::GetTimerTicks();
	if (prevControlLoopCallTime != 0) {
		const StepTimer::Ticks timeElapsed = loopCallTime - prevControlLoopCallTime;
		ewmaControlLoopCallInterval = ewmaControlLoopCallInterval == 0
				? timeElapsed
				: timeElapsed * 0.5 + ewmaControlLoopCallInterval * 0.5;
		minControlLoopCallInterval = min<StepTimer::Ticks>(minControlLoopCallInterval, timeElapsed);
		maxControlLoopCallInterval = max<StepTimer::Ticks>(maxControlLoopCallInterval, timeElapsed);
	}

	// Read the current state of the drive
	ReadState(loopCallTime);

	if (!closedLoopEnabled) {
		// If closed loop disabled, do nothing
	} else if (tuning != 0) {									// if we need to tune, tune
		// Limit the rate at which we command tuning steps Need to do signed comparison because initially, whenLastTuningStepTaken is in the future.
		const int32_t timeSinceLastTuningStep = (int32_t)(loopCallTime - whenLastTuningStepTaken);
		if (timeSinceLastTuningStep >= (int32_t)stepTicksPerTuningStep)
		{
			whenLastTuningStepTaken = loopCallTime;
			PerformTune();
			if (tuning == 0) {
				Platform::DriveEnableOverride(0, false);
			}		// If that was the last tuning move, release the override
		}
		else if (samplingMode == RecordingMode::OnNextMove && timeSinceLastTuningStep + (int32_t)DataCollectionIdleStepTicks >= 0) {
			dataCollectionStartTicks = loopCallTime;
			samplingMode = RecordingMode::Immediate;
		}
	} else if (tuningError) {
		// Don't do anything if there is a tuning error
		SetMotorPhase(0, 0);
	} else {
		ControlMotorCurrents(loopCallTime);							// otherwise control those motor currents!
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
	StepTimer::Ticks loopRuntime = StepTimer::GetTimerTicks() - loopCallTime;
	ewmaControlLoopRuntime = ewmaControlLoopRuntime == 0
			? loopRuntime
			: loopRuntime * 0.5 + ewmaControlLoopRuntime * 0.5;
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

		if (filterRequested & CL_RECORD_RAW_ENCODER_READING) 	{sampleBuffer[wp++] = (float)rawEncoderReading;}
		if (filterRequested & CL_RECORD_CURRENT_MOTOR_STEPS) 	{sampleBuffer[wp++] = currentMotorSteps;}
		if (filterRequested & CL_RECORD_TARGET_MOTOR_STEPS)  	{sampleBuffer[wp++] = targetMotorSteps;}
		if (filterRequested & CL_RECORD_CURRENT_ERROR) 			{sampleBuffer[wp++] = currentError;}
		if (filterRequested & CL_RECORD_PID_CONTROL_SIGNAL)  	{sampleBuffer[wp++] = (float)PIDControlSignal;}
		if (filterRequested & CL_RECORD_PID_P_TERM)  			{sampleBuffer[wp++] = PIDPTerm;}
		if (filterRequested & CL_RECORD_PID_I_TERM)  			{sampleBuffer[wp++] = PIDITerm;}
		if (filterRequested & CL_RECORD_PID_D_TERM)  			{sampleBuffer[wp++] = PIDDTerm;}
		if (filterRequested & CL_RECORD_STEP_PHASE)  			{sampleBuffer[wp++] = (float)stepPhase;}
		if (filterRequested & CL_RECORD_DESIRED_STEP_PHASE)  	{sampleBuffer[wp++] = (float)desiredStepPhase;}
		if (filterRequested & CL_RECORD_PHASE_SHIFT)  			{sampleBuffer[wp++] = (float)phaseShift;}
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

void ClosedLoop::ReadState(StepTimer::Ticks now) noexcept
{
	if (encoder == nullptr) { return; }							// we can't read anything if there is no encoder

	// Calculate the current position & phase from the encoder reading
	rawEncoderReading = encoder->GetReading();
	currentMotorSteps = rawEncoderReading / encoderPulsePerStep;

	// Calculate and store the current error
	currentError = targetMotorSteps - currentMotorSteps;
	derivativeFilter.ProcessReading(currentError, now);

	// Calculate stepPhase - a 0-4095 value representing the phase *within* the current step
	const float tmp = currentMotorSteps * 0.25;
	stepPhase = (tmp - floorf(tmp)) * 4095;
}

void ClosedLoop::ControlMotorCurrents(StepTimer::Ticks loopStartTime) noexcept
{
	// Get the time delta in seconds
	const float timeDelta = (loopStartTime - prevControlLoopCallTime) * (1.0/(float)StepTimer::StepClockRate);

	// Look for a stall or pre-stall
	preStall = errorThresholds[0] > 0 && abs(currentError) > errorThresholds[0];
	const bool alreadyStalled = stall;
	stall 	 = errorThresholds[1] > 0 && abs(currentError) > errorThresholds[1];
	if (stall && !alreadyStalled) {
		Platform::NewDriverFault();
	}

	// If the current error is zero, we don't need to do anything!
	if (currentError == 0 && !CollectingData()) { return; }

	// Use a PID controller to calculate the required 'torque' - the control signal
	PIDPTerm = Kp * currentError;
	PIDITerm = constrain<float>(PIDITerm + Ki * currentError * timeDelta, -255.0, 255.0);
	PIDDTerm = constrain<float>(Kd * (float)derivativeFilter.GetDerivative(), -255.0, 255.0);	// constrain D so that we can graph it more sensibly
	const float sumOfTerms = PIDPTerm + PIDITerm + PIDDTerm;
	PIDControlSignal = (int16_t) constrain<float>(sumOfTerms, -255.0, 255.0);					// clamp between -255 and 255

	// Calculate the offset required to produce the torque in the correct direction
	// i.e. if we are moving in the positive direction, we must apply currents with a positive phase shift
	// The max abs value of phase shift we want is 25%.
	// Given that PIDControlSignal is -255 .. 255 and phase is 0 .. 4095
	// and that 25% of 4095 ~= 1024, our max phase shift ~= 4 * PIDControlSignal
	phaseShift = (4 * PIDControlSignal);

	// Calculate the required motor currents to induce that torque
	// (If stepPhase < phaseShift, we need to add on an extra 4095 to bring us back within the correct range)
	desiredStepPhase = stepPhase + phaseShift + ((stepPhase < -phaseShift) * 4095);
	desiredStepPhase = desiredStepPhase % 4096;

	// Assert the required motor currents
	SetMotorPhase(desiredStepPhase, abs(PIDControlSignal)/255.0);

	// Update vars for the next cycle
	lastError = currentError;
}

void ClosedLoop::Diagnostics(const StringRef& reply) noexcept
{
	reply.printf("Closed loop enabled: %s", closedLoopEnabled ? "yes" : "no");
	reply.catf(", live status: %#x", ReadLiveStatus());
	reply.catf(", encoder type %s", GetEncoderType().ToString());
	reply.catf(", pre-error threshold: %f, error threshold: %f", (double) errorThresholds[0], (double) errorThresholds[1]);
	reply.catf(", reverse polarity: %s", (reversePolarityMultiplier < 0.0) ? "yes" : "no");
	reply.catf(", tuning mode: %#x, tuning error: %#x", tuning, tuningError);

	if (encoder != nullptr)
	{
		reply.catf(", position %" PRIi32, encoder->GetReading());
		encoder->AppendDiagnostics(reply);
	}
	reply.catf(", collecting data: %s", CollectingData() ? "yes" : "no");
	if (CollectingData())
	{
		reply.catf(" (filter: %#x, mode: %u, rate: %u, movement: %u)", filterRequested, samplingMode, (unsigned int)(StepTimer::StepClockRate/dataCollectionIntervalTicks), movementRequested);
	}

	reply.catf(", ultimateGain=%f, oscillationPeriod=%f", (double) ultimateGain, (double) oscillationPeriod);

	reply.cat(", Control loop runtime (ms): ");
	reply.catf("min=%f, ", (double) TickPeriodToTimePeriod(minControlLoopRuntime));
	reply.catf("max=%f, ", (double) TickPeriodToTimePeriod(maxControlLoopRuntime));
	reply.catf("avg=%f"  , (double) TickPeriodToTimePeriod(ewmaControlLoopRuntime));

	reply.cat(", Control loop frequency (Hz): ");
	reply.catf("min=%f, ", (double) TickPeriodToFreq(maxControlLoopCallInterval));
	reply.catf("max=%f, ", (double) TickPeriodToFreq(minControlLoopCallInterval));
	reply.catf("avg=%f"  , (double) TickPeriodToFreq(ewmaControlLoopCallInterval));

	ResetMonitoringVariables();

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
	targetMotorSteps = targetMotorSteps + (stepDirection ? microstepAngle : -microstepAngle) * (Platform::GetDirectionValue(0) ? 1 : -1) * reversePolarityMultiplier;
	if (samplingMode == RecordingMode::OnNextMove)
	{
		dataCollectionStartTicks = whenNextSampleDue = StepTimer::GetTimerTicks();
		samplingMode = RecordingMode::Immediate;
	}
# else
#  error Cannot support closed loop with the specified hardware
# endif
}

uint8_t ClosedLoop::ReadLiveStatus() noexcept
{
	uint8_t result = 0;
	result |= stall << 0;																						// prestall
	result |= preStall << 1;																					// stall
	result |= (encoder != nullptr && (tuningError & minimalTunes[encoder->GetType().ToBaseType()]) > 0) << 2;	// status - 0=not tuned
	result |= ((tuningError & TUNE_ERR_TUNING_FAILURE) > 0) << 3;												// status - 1=tuning error
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

void ClosedLoop::SetHoldingCurrent(float percent)
{
	holdCurrent = constrain<long>(percent, 0, 100) / 100.0;
}

void ClosedLoop::ResetError(size_t driver) noexcept
{
# if SINGLE_DRIVER
	if (driver == 0) {
		// Set the target position to the current position
		ReadState(StepTimer::GetTimerTicks());
		derivativeFilter.Reset();
		targetMotorSteps = currentMotorSteps;
	}
# else
#  error Cannot support closed loop with the specified hardware
# endif
}

bool ClosedLoop::SetClosedLoopEnabled(bool enabled, const StringRef &reply) noexcept
{
	if (enabled) {
# if SUPPORT_SLOW_DRIVERS
		// TODO: Check what we need to do here
		if (Platform::IsSlowDriver()) {
			reply.copy("Closed loop drive mode not yet supported");
			return false;
		}
# endif
# if USE_TC_FOR_STEP || !SINGLE_DRIVER || !SUPPORT_TMC2160
		// TODO: Check what we need to do here
		reply.copy("Closed loop drive mode not yet supported");
		return false;
# endif
		if (encoder == nullptr)
		{
			reply.copy("No encoder specified for closed loop drive mode");
			return false;
		}

		// Reset the tuning (We have already checked encoder != nullptr)
		tuningError = minimalTunes[encoder->GetType().ToBaseType()];
	}

	// Set the target position to the current position
	ResetError(0);

	// Set the closed loop enabled state
	closedLoopEnabled = enabled;
	return true;
}

#endif

// End
