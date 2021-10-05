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
bool 		ClosedLoop::reversePolarity = false;		// Flag if the polarity on this motor is reversed
uint8_t		ClosedLoop::tuning = 0;						// Bitmask of any tuning manoeuvres that have been requested
uint8_t		ClosedLoop::tuningError;					// Flags for any tuning errors
uint16_t	ClosedLoop::stepPhase;						// The current position of the motor
uint16_t	ClosedLoop::desiredStepPhase = 0;			// The desired position of the motor
atomic<int32_t> ClosedLoop::rawEncoderReading;			// The raw reading taken from the encoder
atomic<float> 	ClosedLoop::targetMotorSteps = 0;		// The number of steps the motor should have taken relative to it's zero position
float 		ClosedLoop::encoderPulsePerStep;			// How many encoder readings do we get per step?
float 		ClosedLoop::currentMotorSteps;				// The number of steps the motor has taken relative to it's zero position

namespace ClosedLoop
{
	// Variables and constant private to this module
	constexpr size_t TaskStackWords = 200;				// Size of the stack for all closed loop tasks
	constexpr unsigned int derivativeFilterSize = 8;	// The range of the derivative filter

	// Control variables
	// Variables that can be set by the user to determine how the closed loop controller works
	bool 	closedLoopEnabled = false;					// Has closed loop been enabled by the user?
	uint8_t prevTuningError;							// Used to see what errors have been introduced by tuning

	float 	holdCurrent = 0;							// The minimum holding current when stationary

	float 	Kp = 100;									// The proportional constant for the PID controller
	float 	Ki = 0;										// The proportional constant for the PID controller
	float 	Kd = 0;										// The proportional constant for the PID controller

	float 	errorThresholds[2];							// The error thresholds. [0] is pre-stall, [1] is stall

	float 	ultimateGain = 0;							// The ultimate gain of the controller (used for tuning)
	float 	oscillationPeriod = 0;						// The oscillation period when Kp = ultimate gain

	bool 		collectingData = false;					// Are we currently collecting data? If so:
	StepTimer::Ticks dataCollectionStartTicks;			// - At what tick did data collection start?
	uint16_t 	rateRequested;							//	- What sample rate did they request?
	uint16_t 	filterRequested;						//	- What filter did they request?
	uint16_t 	samplesRequested;						//	- How many samples did they request?
	RecordingMode modeRequested;						//	- What mode did they request?
	uint8_t 	movementRequested;						//	- Which calibration movement did they request? 0=none, 1=polarity, 2=continuous
	float 		sampleBuffer[DataBufferSize * 14];		//	- Store the samples here (max. CLOSED_LOOP_DATA_BUFFER_SIZE samples of 12 variables)
	ReadWriteLock sampleBufferLock;						//  - Lock for the sample buffer
	uint16_t 	sampleBufferReadPointer = 0;			//  - Send this sample next to the mainboard
	uint16_t 	sampleBufferWritePointer = 0;			//  - Store the next sample at this point in the buffer

	// Working variables
	// These variables are all used to calculate the required motor currents. They are declared here so they can be reported on by the data collection task
	bool 			stepDirection = true;				// The direction the motor is attempting to take steps in
	float 			currentError;						// The current error
	float 			lastError = 0;						// The error from the previous iteration

	DerivativeAveragingFilter<derivativeFilterSize> derivativeFilter;	// An averaging filter to smooth the derivative of the error

	float 	PIDPTerm;									// Proportional term
	float 	PIDITerm = 0;								// Integral term
	float 	PIDDTerm;									// Derivative term
	int16_t PIDControlSignal;							// The overall -255 to 255 signal from the PID controller

	int16_t  phaseShift;								// The desired shift in the position of the motor

	int16_t coilA;										// The current to run through coil A
	int16_t coilB;										// The current to run through coil A

	bool 	stall = false;								// Has the closed loop error threshold been exceeded?
	bool 	preStall = false;							// Has the closed loop warning threshold been exceeded?

	// The bitmask of a minimal tune for each encoder type
	// This is an array so that ZEROING_MANOEUVRE can be removed from the magnetic encoders if the LUT is in NVM
	uint8_t minimalTunes[5] = {
		// None
		0,
		// linearQuadrature
		POLARITY_DETECTION_MANOEUVRE | ZEROING_MANOEUVRE | POLARITY_CHECK | CONTROL_CHECK | ENCODER_STEPS_CHECK,
		// rotaryQuadrature
		POLARITY_DETECTION_MANOEUVRE | ZEROING_MANOEUVRE | POLARITY_CHECK | CONTROL_CHECK | ENCODER_STEPS_CHECK,
		// AS5047
		POLARITY_DETECTION_MANOEUVRE | ZEROING_MANOEUVRE | POLARITY_CHECK | CONTROL_CHECK | ENCODER_STEPS_CHECK,
		// TLI5012
		POLARITY_DETECTION_MANOEUVRE | ZEROING_MANOEUVRE | POLARITY_CHECK | CONTROL_CHECK | ENCODER_STEPS_CHECK,
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

	// Functions private to this module
	EncoderType GetEncoderType() noexcept
	{
		return (encoder == nullptr) ? EncoderType::none : encoder->GetType();
	}

	void ReadState() noexcept;
	void CollectSample() noexcept;
	void ControlMotorCurrents() noexcept;

	extern "C" [[noreturn]] void DataCollectionLoop(void *param) noexcept;
	extern "C" [[noreturn]] void DataTransmissionLoop(void *param) noexcept;

}	// end namespace

// Tasks and task loops
static Task<ClosedLoop::TaskStackWords> *dataCollectionTask;		// Data collection task - handles sampling some of the static vars in this file
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
//TODO make this a virtual function member fo the encoder?
float ClosedLoop::PulsePerStepToExternalUnits(float pps, uint8_t encoderType) noexcept {
	switch (encoderType)
	{
	case EncoderType::rotaryQuadrature:
		return pps / 4;													// Output count per step
	case EncoderType::AS5047:
		return (360.0 / ((AS5047D*) ClosedLoop::encoder)->GetMaxValue()) * pps;		// Output degree per step
	default:
		return pps;														// Output pulse per step
	}
}

//TODO make this a virtual function member fo the encoder?
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
	if (tuningErrorBitmask & ClosedLoop::TUNE_ERR_NOT_ZEROED) 					{reply.catf(" The drive has not been zeroed.");}
	if (tuningErrorBitmask & ClosedLoop::TUNE_ERR_NOT_CHECKED_POLARITY) 		{reply.catf(" The drive has not had its polarity checked.");}
	if (tuningErrorBitmask & ClosedLoop::TUNE_ERR_NOT_CHECKED_CONTROL) 			{reply.catf(" The drive has not had its control checked.");}
	if (tuningErrorBitmask & ClosedLoop::TUNE_ERR_NOT_CHECKED_ENCODER_STEPS) 	{reply.catf(" The encoder has not had its count per revolution checked.");}
	if (tuningErrorBitmask & ClosedLoop::TUNE_ERR_INCORRECT_POLARITY) 			{reply.catf(" The drive has been found to have an incorrect polarity.");}
	if (tuningErrorBitmask & ClosedLoop::TUNE_ERR_CONTROL_FAILED) 				{reply.catf(" The drive has been found to be uncontrollable.");}
}

// Helper function to set the motor to a given phase and magnitude
void ClosedLoop::SetMotorPhase(uint16_t phase, float magnitude) noexcept
{
	magnitude = constrain<float>(magnitude, holdCurrent, 1.0);
	coilA = 255 * Trigonometry::FastCos(phase) * magnitude;
	coilB = 255 * Trigonometry::FastSin(phase) * (reversePolarity ? magnitude : -magnitude);

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
	// Init the ATTiny programmer
	pinMode(EncoderCsPin, OUTPUT_HIGH);													// make sure that any attached SPI encoder is not selected

	// The EXP1HCL board uses the standard shared SPI device
	GenerateTmcClock();

	// Initialise to no error thresholds
	errorThresholds[0] = 0;
	errorThresholds[1] = 0;

	// Initialise the monitoring variables
	ResetMonitoringVariables();

	derivativeFilter.Reset();

	// Set up the data collection task
	dataCollectionTask = new Task<ClosedLoop::TaskStackWords>;
	dataCollectionTask->Create(DataCollectionLoop, "CLData", nullptr, TaskPriority::ClosedLoop);

	// Set up the data transmission task
	dataTransmissionTask = new Task<ClosedLoop::TaskStackWords>;
	dataTransmissionTask->Create(DataTransmissionLoop, "CLSend", nullptr, TaskPriority::ClosedLoop);
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
							minimalTunes[EncoderType::AS5047] &= ~ZEROING_MANOEUVRE;
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
		reply.copy("No encoder has been specified");
		return GCodeResult::error;
	}

	if (collectingData)
	{
		reply.copy("Drive is already collecting data");
		return GCodeResult::error;
	}

	if (msg.rate == 0) {
		int variableCount = CountVariablesCollected(msg.filter);

		const unsigned int maxSamples = (DataBufferSize * 14) / variableCount;

		if (msg.numSamples > maxSamples)
		{
			reply.printf("Maximum samples is %d when sample rate is continuous (R0) and %d variables are being collected (D%d)", maxSamples, variableCount - 1, msg.filter);
			return GCodeResult::error;
		}
	}

	if (msg.movement > FULL_TUNE) {
		reply.printf("Maximum value for V is %d. V%d is invalid.", FULL_TUNE, msg.movement);
		return GCodeResult::error;
	}

	// Set up the recording vars
	dataCollectionStartTicks = StepTimer::GetTimerTicks();
	collectingData = true;
	rateRequested = msg.rate == 0 ? 0 : (1000.0 / msg.rate) / portTICK_PERIOD_MS;
	filterRequested = msg.filter;
	tuning |= msg.movement;
	samplesRequested = msg.numSamples;
	modeRequested = (RecordingMode) msg.mode;

	// If we are using RecordingMode::OnNextMove, mark the current state
	if (modeRequested == RecordingMode::OnNextMove)
	{
		startRecordingTrigger = targetMotorSteps;
	}

	// Start the data collection task
	dataCollectionTask->Give();
	return GCodeResult::ok;
}

GCodeResult ClosedLoop::ProcessM569Point6(const CanMessageGeneric &msg, const StringRef &reply) noexcept
{
	CanMessageGenericParser parser(msg, M569Point6Params);

	uint8_t desiredTuning;
	if (!parser.GetUintParam('V', desiredTuning)) {
		if (tuning != 0) {
			return GCodeResult::notFinished;
		} else {
			// Tuning has finished - there are now 3 scenarios
			// 1. No tuning errors exist (!tuningError)							= OK
			// 2. No new tuning errors exist !(~prevTuningError & tuningError)	= WARNING
			// 3. A new tuning error has been introduced (else)					= WARNING

			if (!tuningError) {
				reply.copy("Tuning completed successfully.");
				return GCodeResult::ok;
			} else if (!(~prevTuningError & tuningError)) {
				reply.copy("No new tuning errors have been found, but some existing tuning errors exist.");
				ReportTuningErrors(tuningError, reply);
				return GCodeResult::warning;
			} else {
				reply.copy("One or more tuning errors occurred.");
				ReportTuningErrors(~prevTuningError & tuningError, reply);
				if (prevTuningError & tuningError) {
					reply.catf(" In addition, the following tuning errors were already present:");
					ReportTuningErrors(prevTuningError & tuningError, reply);
				}
				return GCodeResult::warning;
			}
		}
	}

	// Check we are in direct drive mode
	if (SmartDrivers::GetDriverMode(0) != DriverMode::direct) {
		reply.copy("Drive is not in closed loop mode.");
		return GCodeResult::error;
	}

	if (desiredTuning > FULL_TUNE)
	{
		reply.printf("Invalid 'V' parameter value. V may be 0-%d.", FULL_TUNE);
		return GCodeResult::error;
	}

	// Enable all motors & disable them becoming idle
	Platform::DriveEnableOverride(0, true);
	//TODO with a delay here, tuning DC's large motor hardly ever succeeds. With the delay removed, it succeeds more often. So removed the delay for now.
	//TODO the following delay will affect heater timing, so instead we should get the task that actually does the tuning to wait 100ms before starting
	//delay(100);							// allow the motor current to build up and the brake (if any) to release

	prevTuningError = tuningError;
	tuning = desiredTuning;

	return GCodeResult::notFinished;
}

void ClosedLoop::ControlLoop() noexcept
{
	// Record the control loop call interval
	StepTimer::Ticks loopCallTime = StepTimer::GetTimerTicks();
	if (prevControlLoopCallTime != 0) {
		StepTimer::Ticks timeElapsed = loopCallTime - prevControlLoopCallTime;
		ewmaControlLoopCallInterval = ewmaControlLoopCallInterval == 0
				? timeElapsed
				: timeElapsed * 0.5 + ewmaControlLoopCallInterval * 0.5;
		minControlLoopCallInterval = min<StepTimer::Ticks>(minControlLoopCallInterval, timeElapsed);
		maxControlLoopCallInterval = max<StepTimer::Ticks>(maxControlLoopCallInterval, timeElapsed);
	}

	// Read the current state of the drive
	ReadState();

	if (!closedLoopEnabled) {
		// If closed loop disabled, do nothing
	} else if (tuning != 0) {											// If we need to tune, tune
		PerformTune();
		if (tuning == 0) {Platform::DriveEnableOverride(0, false);}		// If that was the last tuning move, release the override
	} else if (tuningError) {
		// Don't do anything if there is a tuning error
		SetMotorPhase(0, 0);
	} else {
		ControlMotorCurrents();							// Otherwise control those motor currents!
	}

	// Collect a sample, if we need to
	if (collectingData && rateRequested == 0) {
		if (modeRequested != RecordingMode::OnNextMove || startRecordingTrigger != targetMotorSteps) {
			modeRequested = RecordingMode::Immediate;
			if (samplesRequested-- > 0) {
				CollectSample();
			} else {
				collectingData = false;
				movementRequested = 0;	// Just to be safe
				dataTransmissionTask->Give();
			}
		}
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

[[noreturn]] void ClosedLoop::DataCollectionLoop(void *param) noexcept
{
	while (true)
	{

		// If we are not collecting data, block the task
		// If rateRequested == 0, the data collection is handled in ClosedLoop::ControlLoop()
		while (!collectingData || rateRequested == 0)
		{
			TaskBase::Take();
		}

		// If we are using RecordingMode::OnNextMove, wait for a move to start
		startRecordingTrigger = targetMotorSteps;
		while (modeRequested == RecordingMode::OnNextMove && startRecordingTrigger == targetMotorSteps)
		{
			TaskBase::Take(10);
		}

		uint32_t lastWakeTime = xTaskGetTickCount();

		// Loop for each sample
		for (int i = 0; i < samplesRequested; i++)
		{
			{
				// Set up a CAN message
				CanMessageBuffer buf(nullptr);
				CanMessageClosedLoopData& msg = *(buf.SetupStatusMessage<CanMessageClosedLoopData>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress()));

				// Populate the control fields
				msg.numSamples = 1;
				msg.lastPacket = (i == samplesRequested - 1);
				msg.firstSampleNumber = i;
				msg.filter = filterRequested;
				msg.zero = msg.zero2 = 0;

				// Populate the data fields
				// TODO: Pack more than one set of data into a message
				unsigned int dataPointer = 0;
																		{msg.data[dataPointer++] = TickPeriodToTimePeriod(StepTimer::GetTimerTicks() - dataCollectionStartTicks);}	// (Always collect this)
				if (filterRequested & CL_RECORD_RAW_ENCODER_READING) 	{msg.data[dataPointer++] = rawEncoderReading;}
				if (filterRequested & CL_RECORD_CURRENT_MOTOR_STEPS) 	{msg.data[dataPointer++] = currentMotorSteps;}
				if (filterRequested & CL_RECORD_TARGET_MOTOR_STEPS)  	{msg.data[dataPointer++] = targetMotorSteps;}
				if (filterRequested & CL_RECORD_CURRENT_ERROR) 		{msg.data[dataPointer++] = currentError;}
				if (filterRequested & CL_RECORD_PID_CONTROL_SIGNAL)  	{msg.data[dataPointer++] = PIDControlSignal;}
				if (filterRequested & CL_RECORD_PID_P_TERM)  			{msg.data[dataPointer++] = PIDPTerm;}
				if (filterRequested & CL_RECORD_PID_I_TERM)  			{msg.data[dataPointer++] = PIDITerm;}
				if (filterRequested & CL_RECORD_PID_D_TERM)  			{msg.data[dataPointer++] = PIDDTerm;}
				if (filterRequested & CL_RECORD_STEP_PHASE)  			{msg.data[dataPointer++] = stepPhase;}
				if (filterRequested & CL_RECORD_DESIRED_STEP_PHASE)  	{msg.data[dataPointer++] = desiredStepPhase;}
				if (filterRequested & CL_RECORD_PHASE_SHIFT)  			{msg.data[dataPointer++] = phaseShift;}
				if (filterRequested & CL_RECORD_COIL_A_CURRENT) 		{msg.data[dataPointer++] = coilA;}
				if (filterRequested & CL_RECORD_COIL_B_CURRENT) 		{msg.data[dataPointer++] = coilB;}

				// Send the CAN message
				buf.dataLength = msg.GetActualDataLength();
				CanInterface::Send(&buf);
			}

			// Pause to maintain the sample rate (TODO: Implement variable sample rate)
			vTaskDelayUntil(&lastWakeTime, rateRequested);
		}

		// Mark that we have finished collecting data
		collectingData = false;
	}
}

[[noreturn]] void ClosedLoop::DataTransmissionLoop(void *param) noexcept
{
	while (true)
	{
		if (!collectingData) {	// Only do if we are not collecting data - we don't want to lock the buffer whilst moving!
			ReadLocker locker(sampleBufferLock);

			// Only attempt to transmit data if there is any
			if (sampleBufferWritePointer > 0)
			{
				// Count how many bits are set in 'filterRequested'
				unsigned int variableCount = CountVariablesCollected(filterRequested);

				// Work out the maximum number of samples that can be sent in 1 packet
				const unsigned int maxSamplesInPacket = CanMessageClosedLoopData::MaxDataItems / variableCount;

				// Loop for until everything has been read
				while (sampleBufferReadPointer < sampleBufferWritePointer) {
					// Set up a CAN message
					CanMessageBuffer buf(nullptr);
					CanMessageClosedLoopData& msg = *(buf.SetupStatusMessage<CanMessageClosedLoopData>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress()));

					// Populate the control fields
					msg.firstSampleNumber = sampleBufferReadPointer / variableCount;
					msg.filter = filterRequested;
					msg.zero = msg.zero2 = 0;

					const unsigned int samplesRemaining = (sampleBufferWritePointer - sampleBufferReadPointer) / variableCount;
					msg.lastPacket = samplesRemaining <= maxSamplesInPacket;
					msg.numSamples = msg.lastPacket ? samplesRemaining : maxSamplesInPacket;

					const size_t numToCopy = msg.numSamples * variableCount;
					memcpy(msg.data, sampleBuffer + sampleBufferReadPointer, numToCopy * sizeof(float));
					sampleBufferReadPointer += numToCopy;

					// Send the CAN message
					buf.dataLength = msg.GetActualDataLength();
					CanInterface::Send(&buf);
				}

				// If we are finished collecting data, reset the buffer
				if (!collectingData)
				{
					sampleBufferReadPointer = 0;
					sampleBufferWritePointer = 0;
				}
			}
		}

		TaskBase::Take(100);
	}
}

void ClosedLoop::CollectSample() noexcept
{
	WriteLocker locker(sampleBufferLock);
															{sampleBuffer[sampleBufferWritePointer++] = TickPeriodToTimePeriod(StepTimer::GetTimerTicks() - dataCollectionStartTicks);}	// (Always collect this)
	if (filterRequested & CL_RECORD_RAW_ENCODER_READING) 	{sampleBuffer[sampleBufferWritePointer++] = rawEncoderReading;}
	if (filterRequested & CL_RECORD_CURRENT_MOTOR_STEPS) 	{sampleBuffer[sampleBufferWritePointer++] = currentMotorSteps;}
	if (filterRequested & CL_RECORD_TARGET_MOTOR_STEPS)  	{sampleBuffer[sampleBufferWritePointer++] = targetMotorSteps;}
	if (filterRequested & CL_RECORD_CURRENT_ERROR) 			{sampleBuffer[sampleBufferWritePointer++] = currentError;}
	if (filterRequested & CL_RECORD_PID_CONTROL_SIGNAL)  	{sampleBuffer[sampleBufferWritePointer++] = PIDControlSignal;}
	if (filterRequested & CL_RECORD_PID_P_TERM)  			{sampleBuffer[sampleBufferWritePointer++] = PIDPTerm;}
	if (filterRequested & CL_RECORD_PID_I_TERM)  			{sampleBuffer[sampleBufferWritePointer++] = PIDITerm;}
	if (filterRequested & CL_RECORD_PID_D_TERM)  			{sampleBuffer[sampleBufferWritePointer++] = PIDDTerm;}
	if (filterRequested & CL_RECORD_STEP_PHASE)  			{sampleBuffer[sampleBufferWritePointer++] = stepPhase;}
	if (filterRequested & CL_RECORD_DESIRED_STEP_PHASE)  	{sampleBuffer[sampleBufferWritePointer++] = desiredStepPhase;}
	if (filterRequested & CL_RECORD_PHASE_SHIFT)  			{sampleBuffer[sampleBufferWritePointer++] = phaseShift;}
	if (filterRequested & CL_RECORD_COIL_A_CURRENT) 		{sampleBuffer[sampleBufferWritePointer++] = coilA;}
	if (filterRequested & CL_RECORD_COIL_B_CURRENT) 		{sampleBuffer[sampleBufferWritePointer++] = coilB;}
}

void ClosedLoop::ReadState() noexcept
{
	if (encoder == nullptr) {return;}	// We can't read anything if encoder is a nullptr

	// Calculate the current position & phase from the encoder reading
	rawEncoderReading = encoder->GetReading();
	currentMotorSteps = rawEncoderReading / encoderPulsePerStep;

	// Calculate and store the current error
	currentError = targetMotorSteps - currentMotorSteps;
	float currentTimestamp = TickPeriodToTimePeriod(StepTimer::GetTimerTicks()) / 1000;
	if (!derivativeFilter.IsInit()) {derivativeFilter.Init(currentError, currentTimestamp);}
	derivativeFilter.ProcessReading(currentError, currentTimestamp);

	// Calculate stepPhase - a 0-4095 value representing the phase *within* the current step
	float tmp = currentMotorSteps / 4;
	if (tmp >= 0) {
		stepPhase = (tmp - (int) tmp) * 4095;
	} else {
		stepPhase = (1 + tmp - (int) tmp) * 4095;
	}
}

void ClosedLoop::ControlMotorCurrents() noexcept
{
	// Get the time delta
	float currentTimestamp = TickPeriodToTimePeriod(StepTimer::GetTimerTicks()) / 1000;
	float timeDelta = currentTimestamp - (TickPeriodToTimePeriod(prevControlLoopCallTime) / 1000);

	// Look for a stall or pre-stall
	if (!stall && abs(currentError) > errorThresholds[1]) {
		// We have just stalled! Alert RRF immediately!
		Platform::NewDriverFault();
	}
	preStall = errorThresholds[0] > 0 && abs(currentError) > errorThresholds[0];
	stall 	 = errorThresholds[1] > 0 && abs(currentError) > errorThresholds[1];

	// If the current error is zero, we don't need to do anything!
	if (!collectingData && currentError == 0) {return;}

	// Use a PID controller to calculate the required 'torque' - the control signal
	PIDPTerm = Kp * currentError;
	float newITerm = PIDITerm + Ki * currentError * timeDelta;
	if (abs(newITerm) < 255) {		// Limit to the value the PID control signal is clamped to
		PIDITerm = newITerm;
	}
	PIDDTerm = derivativeFilter.IsValid() ? Kd * (float)derivativeFilter.GetDerivative() : 0.0;
	float sumOfTerms = PIDPTerm + PIDITerm + PIDDTerm;
	PIDControlSignal = (int16_t) constrain<float>(sumOfTerms, -255, 255);	// Clamp between -255 and 255

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
	reply.catf(", reverse polarity: %s", reversePolarity ? "yes" : "no");
	reply.catf(", tuning mode: %#x, tuning error: %#x", tuning, tuningError);

	if (encoder != nullptr)
	{
		reply.catf(", position %" PRIi32, encoder->GetReading());
		encoder->AppendDiagnostics(reply);
	}
	reply.catf(", collecting data: %s", collectingData ? "yes" : "no");
	if (collectingData)
	{
		reply.catf(" (filter: %#x, mode: %d, rate: %d, movement: %d)", filterRequested, modeRequested, rateRequested, movementRequested);
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
	bool interpolation;	// TODO: Work out what this is for!
# if SUPPORT_TMC2160 && SINGLE_DRIVER
	unsigned int microsteps = SmartDrivers::GetMicrostepping(0, interpolation);
	float microstepAngle = microsteps == 0 ? 1 : 1.0/microsteps;
	targetMotorSteps = targetMotorSteps + (stepDirection ? microstepAngle : -microstepAngle) * (Platform::GetDirectionValue(0) ? 1 : -1) * (reversePolarity ? -1 : 1);
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
		ReadState();
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
