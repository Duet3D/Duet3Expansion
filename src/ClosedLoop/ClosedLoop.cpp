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
 * 6. When using an absolute encoder, although it could be treated like a relative encoder for the purposes of motor control,
 *    the angle information may be useful to compensate for leadscrew nut irregularities; so we should preserve the angle information.
 */

#include "ClosedLoop.h"

#if SUPPORT_CLOSED_LOOP

using std::atomic;
using std::numeric_limits;

# include "Encoders/AS5047D.h"
# include "Encoders/TLI5012B.h"
# include "Encoders/QuadratureEncoderPdec.h"
# include "Encoders/LinearCompositeEncoder.h"

# include <ClosedLoop/DerivativeAveragingFilter.h>

# include <math.h>
# include <Platform/Platform.h>
# include <Movement/Move.h>
# include <General/Bitmap.h>
# include <Platform/TaskPriorities.h>
# include <CAN/CanInterface.h>
# include <CanMessageBuffer.h>
# include <CanMessageFormats.h>
# include <CanMessageGenericParser.h>
# include <CanMessageGenericTables.h>

# if SUPPORT_TMC2160
#  include "Movement/StepperDrivers/TMC51xx.h"
# else
#  error Cannot support closed loop with the specified hardware
# endif

#define BASIC_TUNING_DEBUG	0

constexpr size_t DataCollectionTaskStackWords = 200;		// Size of the stack for the data collection task
constexpr size_t EncoderCalibrationTaskStackWords = 500;	// Size of the stack for the encoder calibration task

// Tasks and task loops
static Task<DataCollectionTaskStackWords> *dataTransmissionTask = nullptr;			// Data transmission task - handles sending back the buffered sample data
static Task<EncoderCalibrationTaskStackWords> *encoderCalibrationTask = nullptr;	// Encoder calibration task - handles calibrating the encoder in the background

extern "C" [[noreturn]] void DataTransmissionTaskEntry(void *param) noexcept
{
	((ClosedLoop*)param)->DataTransmissionTaskLoop();
}

extern "C" [[noreturn]] void EncoderCalibrationTaskEntry(void *param) noexcept
{
	((ClosedLoop*)param)->EncoderCalibrationTaskLoop();
}

// Helper function to convert a time period (expressed in StepTimer::Ticks) to ms
static inline float TickPeriodToMillis(StepTimer::Ticks tickPeriod) noexcept
{
	return tickPeriod * StepTimer::StepClocksToMillis;
}

// Helper function to convert a time period (expressed in StepTimer::Ticks) to us
static inline uint32_t TickPeriodToMicroseconds(StepTimer::Ticks tickPeriod) noexcept
{
	return (tickPeriod * 1000)/(StepTimer::GetTickRate()/1000);
}

// Helper function to convert a time period (expressed in StepTimer::Ticks) to a frequency in Hz
static inline uint32_t TickPeriodToFreq(StepTimer::Ticks tickPeriod) noexcept
{
	return StepTimer::StepClockRate/tickPeriod;
}

// Helper function to reset the 'monitoring variables' as defined above
void ClosedLoop::ResetMonitoringVariables() noexcept
{
	ClosedLoop::minControlLoopRuntime = numeric_limits<StepTimer::Ticks>::max();
	ClosedLoop::maxControlLoopRuntime = 1;
	ClosedLoop::minControlLoopCallInterval = numeric_limits<StepTimer::Ticks>::max();
	ClosedLoop::maxControlLoopCallInterval = 1;
}

// Helper function to cat all the current tuning errors onto a reply in human-readable form
void ClosedLoop::ReportTuningErrors(TuningErrors tuningErrorBitmask, const StringRef &reply) noexcept
{
	if (tuningErrorBitmask & TuningError::NeedsBasicTuning) 			{ reply.cat(", the drive has not had basic tuning done"); }
	if (tuningErrorBitmask & TuningError::NotCalibrated) 				{ reply.cat(", the drive has not been calibrated"); }
	if (tuningErrorBitmask & TuningError::SystemError) 					{ reply.cat(", a system error occurred while tuning"); }
	if (tuningErrorBitmask & TuningError::TuningOrCalibrationInProgress){ reply.cat(", encoder calibration is in progress"); }
	if (tuningErrorBitmask & TuningError::InconsistentMotion)			{ reply.cat(", the measured motion was inconsistent"); }
	if (tuningErrorBitmask & TuningError::TooLittleMotion)				{ reply.cat(", the measured motion was less than expected"); }
	if (tuningErrorBitmask & TuningError::TooMuchMotion)				{ reply.cat(", the measured motion was more than expected"); }
}

void ClosedLoop::SetTargetToCurrentPosition() noexcept
{
	mParams.position = (float)encoder->GetCurrentCount() / encoder->GetCountsPerStep();
	moveInstance->SetCurrentMotorSteps(0, mParams.position);
}

// Helper function to set the motor to a given phase and magnitude
// The phase is normally in the range 0 to 4095 but when tuning it can be 0 to somewhat over 8192.
// We must take it modulo 4096 when computing the currents. Function Trigonometry::FastSinCos does that.
// 'magnitude' must be in range 0.0..1.0
void ClosedLoop::SetMotorPhase(uint16_t phase, float magnitude) noexcept
{
	currentMotorPhase = phase;
	float sine, cosine;
	Trigonometry::FastSinCos(phase, sine, cosine);
	coilA = (int16_t)lrintf(cosine * magnitude);
	coilB = (int16_t)lrintf(sine * magnitude);

# if SUPPORT_TMC2160 && SINGLE_DRIVER
	SmartDrivers::SetMotorCurrents(0, (((uint32_t)(uint16_t)coilB << 16) | (uint32_t)(uint16_t)coilA) & 0x01FF01FF);
# else
#  error Cannot support closed loop with the specified hardware
# endif
}

static_assert(ClockGenGclkNumber == GclkClosedLoop);							// check that this GCLK number has been reserved

static void GenerateTmcClock()
{
	// Currently we program DPLL0 to generate 120MHz output, so to get 15MHz with 1:1 ratio we divide by 8.
	// We could divide by 7 instead giving 17.143MHz with 25ns and 33.3ns times. TMC2160A max is 18MHz, minimum 16ns and 16ns low.
	// Max SPI clock frequency is half this clock frequency.
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
	errorDerivativeFilter.Reset();
	speedFilter.Reset();

	// Set up the data transmission task
	dataTransmissionTask = new Task<DataCollectionTaskStackWords>;
	dataTransmissionTask->Create(DataTransmissionTaskEntry, "CLSend", this, TaskPriority::ClosedLoopDataTransmission);
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
	float tempKv = Kv;
	float tempKa = Ka;
	uint16_t tempStepsPerRev = 200;
	size_t numThresholds = 2;
	float tempErrorThresholds[numThresholds];
	float holdingCurrentPercent;

	// Pull changed parameters
	const bool seenT = parser.GetUintParam('T', tempEncoderType);
	const bool seenC = parser.GetFloatParam('C', tempCPR);
	const bool seenPid = parser.GetFloatParam('R', tempKp) | parser.GetFloatParam('I', tempKi)  | parser.GetFloatParam('D', tempKd)
						| parser.GetFloatParam('V', tempKv) | parser.GetFloatParam('A', tempKa);
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
			reply.lcatf("PID parameters P=%.1f I=%.3f D=%.3f V=%.1f A=%.1f, min. current %" PRIi32 "%%",
						(double)Kp, (double)Ki, (double)Kd, (double)Kv, (double)Ka, lrintf(holdCurrentFraction * 100.0));
			reply.lcatf("Warning/error threshold %.2f/%.2f", (double)errorThresholds[0], (double)errorThresholds[1]);
		}
		return GCodeResult::ok;
	}

	if (seenT && !seenC && (tempEncoderType == EncoderType::rotaryQuadrature || tempEncoderType == EncoderType::linearComposite))
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
	if (seenC && tempCPR < 4 * tempStepsPerRev && tempEncoderType != EncoderType::linearComposite)
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
		Kv = tempKv;
		Ka = tempKa;
		PIDITerm = 0.0;
		errorDerivativeFilter.Reset();
		speedFilter.Reset();
	}

	if (seenH)
	{
		holdCurrentFraction = constrain<float>(holdingCurrentPercent, 10.0, 100.0) / 100.0;
	}

	if (seenE)
	{
		errorThresholds[0] = tempErrorThresholds[0];
		errorThresholds[1] = tempErrorThresholds[1];
	}

	if (seenT)
	{
		//TODO do we need to get a lock here in case there is any movement?
		SetClosedLoopEnabled(0, ClosedLoopMode::open, reply);
		DeleteObject(encoder);

		switch (tempEncoderType)
		{
		case EncoderType::none:
		default:
			// encoder is already nullptr
			break;

		case EncoderType::rotaryAS5047:
			encoder = new AS5047D(tempStepsPerRev, *Platform::sharedSpi, EncoderCsPin);
			CreateCalibrationTask();
			break;

		case EncoderType::rotaryTLI5012:
			encoder = new TLI5012B(tempStepsPerRev, *Platform::sharedSpi, EncoderCsPin);
			CreateCalibrationTask();
			break;

		case EncoderType::linearComposite:
			CreateCalibrationTask();
			encoder = new LinearCompositeEncoder(tempCPR, tempStepsPerRev, *Platform::sharedSpi, EncoderCsPin);
			break;

		case EncoderType::rotaryQuadrature:
			encoder = new QuadratureEncoderPdec(tempCPR, tempStepsPerRev);
			break;
		}

		if (encoder != nullptr)
		{
			tuningError = encoder->MinimalTuningNeeded();
			const GCodeResult rslt = encoder->Init(reply);
			if (rslt == GCodeResult::ok)
			{
				encoder->LoadLUT(tuningError);
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

	{
		TaskCriticalSectionLocker lock;

		// Set up the recording vars
		filterRequested = msg.filter;
		sampleBuffer.Init(ClosedLoopSampleLength(filterRequested));
		sampleBufferOverflowed = false;
		samplesRequested = msg.numSamples;
		samplesCollected = samplesSent = 0;
		dataCollectionIntervalTicks = (msg.rate == 0) ? 1 : StepTimer::StepClockRate/msg.rate;
		dataCollectionStartTicks = whenNextSampleDue = StepTimer::GetTimerTicks();
		samplingMode = (RecordingMode)requestedMode;				// do this one last, it triggers data collection

		StartTuning(msg.movement);
	}
	return GCodeResult::ok;
}

// This is called when tuning has finished and the basicTuningDataReady flag is set
GCodeResult ClosedLoop::ProcessBasicTuningResult(const StringRef& reply) noexcept
{
	// Tuning has finished - there are now 3 scenarios
	// 1. No tuning errors exist (!tuningError)							= OK
	// 2. No new tuning errors exist !(~prevTuningError & tuningError)	= WARNING
	// 3. A new tuning error has been introduced (else)					= WARNING
	basicTuningDataReady = false;
	reply.printf("Driver %u.0 basic tuning ", CanInterface::GetCanAddress());

	const TuningErrors newTuningErrors = encoder->ProcessTuningData();
	if (newTuningErrors != 0)
	{
		tuningError &= ~TuningError::TuningOrCalibrationInProgress;
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
	errorDerivativeFilter.Reset();
	speedFilter.Reset();
	SetTargetToCurrentPosition();
	tuningError &= ~TuningError::TuningOrCalibrationInProgress;

	const float hyst = encoder->GetMeasuredHysteresis();
	const unsigned int increaseFactor = (encoder->GetType() == EncoderType::linearComposite) ? LinearEncoderIncreaseFactor : 1;
	if (hyst >= MaxSafeBacklash * increaseFactor)
	{
		tuningError = TuningError::HysteresisTooHigh;
		reply.catf("failed, measured backlash (%.3f step) is too high", (double)hyst);
		return GCodeResult::error;
	}
	else if (hyst >= MaxGoodBacklash * increaseFactor)
	{
		reply.catf("succeeded but measured backlash (%.3f step) is high", (double)hyst);
		tuningError = 0;
		return GCodeResult::warning;
	}
	else
	{
		reply.catf("succeeded, measured backlash %.3f step", (double)hyst);
		tuningError = 0;
		return GCodeResult::ok;
	}
}

// This function is run by the encoder calibration task.
// Its purpose is to wait for encoder calibration data to become available and process it.
// Processing to takes several seconds, so we need to do it in a separate task to avoid the main board timing out awaiting CAN responses.
void ClosedLoop::EncoderCalibrationTaskLoop() noexcept
{
	for (;;)
	{
		TaskBase::Take();
		if (encoder != nullptr && encoder->UsesCalibration() && calibrationState == CalibrationState::dataReady)
		{
			calibrationErrors = encoder->Calibrate(calibrateNotCheck);
			calibrationState = CalibrationState::complete;
		}
	}
}

void ClosedLoop::CreateCalibrationTask() noexcept
{
	if (encoderCalibrationTask == nullptr)
	{
		encoderCalibrationTask = new Task<EncoderCalibrationTaskStackWords>;
		encoderCalibrationTask->Create(EncoderCalibrationTaskEntry, "EncCal", this, TaskPriority::SpinPriority);		// must be same priority as main task
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
			tuningError = (tuningError & ~(TuningError::TooMuchMotion | TuningError::TooLittleMotion | TuningError::InconsistentMotion | TuningError::TuningOrCalibrationInProgress)) | calibrationErrors;
		}
		else
		{
			tuningError &= ~TuningError::TuningOrCalibrationInProgress;
		}
		ReportTuningErrors(calibrationErrors, reply);
		if (calibrationErrors & (TuningError::TooMuchMotion | TuningError::TooLittleMotion))
		{
			reply.catf(", measured counts/step is about %.1f", (double)encoder->GetMeasuredCountsPerStep());
		}
		return GCodeResult::error;
	}

	PIDITerm = 0.0;
	errorDerivativeFilter.Reset();
	speedFilter.Reset();
	SetTargetToCurrentPosition();

	const float hyst = encoder->GetMeasuredHysteresis();
	if (hyst >= MaxSafeBacklash)
	{
		if (calibrateNotCheck) { tuningError = TuningError::HysteresisTooHigh; }
		else { tuningError &= ~TuningError::TuningOrCalibrationInProgress; }
		reply.catf("failed, measured backlash (%.3f step) is too high", (double)hyst);
		return GCodeResult::error;
	}
	else if (hyst >= MaxGoodBacklash)
	{
		if (calibrateNotCheck) { tuningError = 0; }
		else { tuningError &= ~TuningError::TuningOrCalibrationInProgress; }
		reply.catf("succeeded but measured backlash (%.3f step) is high", (double)hyst);
	}
	else
	{
		if (calibrateNotCheck) { tuningError = 0; }
		else { tuningError &= ~TuningError::TuningOrCalibrationInProgress; }
		reply.catf("succeeded, measured backlash is %.3f step", (double)hyst);
	}

	// Report the calibration errors and corrections
	reply.lcatf("%s encoder reading errors: ", (calibrateNotCheck) ? "Original" : "Residual");
	encoder->AppendCalibrationErrors(reply);
	if (calibrateNotCheck)
	{
		reply.lcatf("Corrections made: ");
		encoder->AppendLUTCorrections(reply);
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
		return (basicTuningDataReady) ? ProcessBasicTuningResult(reply) : ProcessCalibrationResult(reply);
	}

	switch (desiredTuning)
	{
	default:
		reply.copy("invalid tuning mode");
		return GCodeResult::error;

	case 1:		// basic calibration
		if (!encoder->UsesBasicTuning())
		{
			reply.copy("basic tuning is not applicable to absolute encoders");
			return GCodeResult::error;
		}
		break;

	case 2:
	case 3:
	case 4:
		if (!encoder->UsesCalibration())
		{
			reply.copy("calibration is not applicable to the configured encoder type");
			return GCodeResult::error;
		}
		if (desiredTuning == 4)
		{
			// Tuning move 4 just clears the lookup table
			encoder->ScrubLUT();
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
	if (SmartDrivers::GetDriverMode(0) < DriverMode::foc)
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
	tuningError |= TuningError::TuningOrCalibrationInProgress;				// to prevent movement until we are done tuning
}

// Call this when encoder calibration has finished collecting data
void ClosedLoop::ReadyToCalibrate(bool store) noexcept
{
	calibrateNotCheck = store;
	if (encoderCalibrationTask != nullptr)
	{
		calibrationState = CalibrationState::dataReady;
		tuningError |= TuningError::TuningOrCalibrationInProgress;			// to prevent movement in case we are re-calibrating
		encoderCalibrationTask->Give();
	}
}

// This is called by tuning to execute a step
void ClosedLoop::AdjustTargetMotorSteps(float amount) noexcept
{
	mParams.position += amount;
	moveInstance->SetCurrentMotorSteps(0, lrintf(mParams.position));
}

void ClosedLoop::ControlLoop() noexcept
{
	// Record the control loop call interval
	const StepTimer::Ticks loopCallTime = StepTimer::GetTimerTicks();
	const StepTimer::Ticks timeElapsed = loopCallTime - prevControlLoopCallTime;
	prevControlLoopCallTime = loopCallTime;
	minControlLoopCallInterval = min<StepTimer::Ticks>(minControlLoopCallInterval, timeElapsed);
	maxControlLoopCallInterval = max<StepTimer::Ticks>(maxControlLoopCallInterval, timeElapsed);

	// Read the current state of the drive. Do this even if we are not in closed loop mode.
	if (encoder != nullptr && !encoder->TakeReading())
	{
		// Calculate and store the current error in full steps
		const bool moving = moveInstance->GetCurrentMotion(0, loopCallTime, currentMode != ClosedLoopMode::open, mParams);
		if (moving && samplingMode == RecordingMode::OnNextMove)
		{
			dataCollectionStartTicks = whenNextSampleDue = StepTimer::GetTimerTicks();
			samplingMode = RecordingMode::Immediate;
		}

		const float targetEncoderReading = rintf(mParams.position * encoder->GetCountsPerStep());
		currentError = (float)(targetEncoderReading - encoder->GetCurrentCount()) * encoder->GetStepsPerCount();
		errorDerivativeFilter.ProcessReading(currentError, loopCallTime);
		speedFilter.ProcessReading(encoder->GetCurrentCount() * encoder->GetStepsPerCount(), loopCallTime);

		if (currentMode != ClosedLoopMode::open)
		{
			if (tuning != 0)											// if we need to tune, do it
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
			else if (tuningError == 0)
			{
				ControlMotorCurrents(timeElapsed);						// otherwise control those motor currents!

				// Look for a stall or pre-stall
				const float positionErr = fabsf(currentError);
				if (stall)
				{
					// Reset the stall flag when the position error falls to below half the tolerance, to avoid generating too many stall events
					//TODO do we need a minimum delay before resetting too?
					if (errorThresholds[1] <= 0 || positionErr < errorThresholds[1]/2)
					{
						stall = false;
					}
				}
				else
				{
					stall = errorThresholds[1] > 0 && positionErr > errorThresholds[1];
					if (stall)
					{
						Platform::NewDriverFault();
					}
					else
					{
						preStall = errorThresholds[0] > 0 && positionErr > errorThresholds[0];
					}
				}
			}
		}

		// Collect a sample, if we need to
		if (samplingMode == RecordingMode::Immediate && (int32_t)(loopCallTime - whenNextSampleDue) >= 0)
		{
			// It's time to take a sample
			CollectSample();
			whenNextSampleDue += dataCollectionIntervalTicks;
		}
	}

	// Record how long this has taken to run
	const StepTimer::Ticks loopRuntime = StepTimer::GetTimerTicks() - loopCallTime;
	minControlLoopRuntime = min<StepTimer::Ticks>(minControlLoopRuntime, loopRuntime);
	maxControlLoopRuntime = max<StepTimer::Ticks>(maxControlLoopRuntime, loopRuntime);
}

// Send data from the buffer to the main board over CAN
[[noreturn]] void ClosedLoop::DataTransmissionTaskLoop() noexcept
{
	while (true)
	{
		const RecordingMode locMode = samplingMode;										// to capture the volatile variable
		if (locMode == RecordingMode::Immediate || locMode == RecordingMode::SendingData)
		{
			// Started a new data collection
			samplesSent = 0;

			// Loop until everything has been read. Stop when either we have sent the requested number of samples, or the state is SendingData and we have sent all the data in the buffer.
			// Note, this may mean that the last packet contains no data and has the "last" flag set.
			bool finished;
			do
			{
				// Set up a CAN message
				CanMessageBuffer buf;
				CanMessageClosedLoopData& msg = *(buf.SetupStatusMessage<CanMessageClosedLoopData>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress()));

				// Populate the control fields
				msg.firstSampleNumber = samplesSent;
				msg.filter = filterRequested;
				msg.zero = msg.zero2 = 0;

				unsigned int numSamplesInMessage = 0;
				size_t dataIndex = 0;
				do
				{
					while (samplesSent == samplesCollected && samplingMode == RecordingMode::Immediate)
					{
						TaskBase::Take();							// wait for data to be available
					}

					if (samplesSent < samplesCollected)
					{
						dataIndex += sampleBuffer.GetSample(msg.data + dataIndex);
						++samplesSent;								// update this one first to avoid a race condition
						++numSamplesInMessage;
					}
					finished = (samplingMode != RecordingMode::Immediate && samplesSent == samplesCollected);
				} while (!finished && dataIndex + sampleBuffer.GetBytesPerSample() <= ARRAY_SIZE(msg.data));

				msg.numSamples = numSamplesInMessage;
				msg.lastPacket = finished;
				msg.overflowed = sampleBufferOverflowed;
				msg.badSample = sampleBuffer.HadBadSample();

				// Send the CAN message
				buf.dataLength = msg.GetActualDataLength(dataIndex);
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
	if (sampleBuffer.IsFull())
	{
		sampleBufferOverflowed = true;							// the buffer is full so tell the sending task about it
		samplingMode = RecordingMode::SendingData;				// stop collecting data
	}
	else
	{
		sampleBuffer.PutF32(TickPeriodToMillis(StepTimer::GetTimerTicks() - dataCollectionStartTicks));		// always collect this

		if (filterRequested & CL_RECORD_RAW_ENCODER_READING) 	{ sampleBuffer.PutI32(encoder->GetCurrentCount()); }
		if (filterRequested & CL_RECORD_CURRENT_MOTOR_STEPS) 	{ sampleBuffer.PutF32((float)encoder->GetCurrentCount() * encoder->GetStepsPerCount()); }
		if (filterRequested & CL_RECORD_TARGET_MOTOR_STEPS)  	{ sampleBuffer.PutF32(mParams.position); }
		if (filterRequested & CL_RECORD_CURRENT_ERROR) 			{ sampleBuffer.PutF32(currentError); }
		if (filterRequested & CL_RECORD_PID_CONTROL_SIGNAL)  	{ sampleBuffer.PutF16(PIDControlSignal); }
		if (filterRequested & CL_RECORD_PID_P_TERM)  			{ sampleBuffer.PutF16(PIDPTerm); }
		if (filterRequested & CL_RECORD_PID_I_TERM)  			{ sampleBuffer.PutF16(PIDITerm); }
		if (filterRequested & CL_RECORD_PID_D_TERM)  			{ sampleBuffer.PutF16(PIDDTerm); }
		if (filterRequested & CL_RECORD_PID_V_TERM)  			{ sampleBuffer.PutF16(PIDVTerm); }
		if (filterRequested & CL_RECORD_PID_A_TERM)  			{ sampleBuffer.PutF16(PIDATerm); }
		if (filterRequested & CL_RECORD_CURRENT_STEP_PHASE)  	{ sampleBuffer.PutU16(encoder->GetCurrentPhasePosition()); }
		if (filterRequested & CL_RECORD_DESIRED_STEP_PHASE)  	{ sampleBuffer.PutU16(desiredStepPhase); }
		if (filterRequested & CL_RECORD_PHASE_SHIFT)  			{ sampleBuffer.PutU16(phaseShift % 4096); }
		if (filterRequested & CL_RECORD_COIL_A_CURRENT) 		{ sampleBuffer.PutI16(coilA); }
		if (filterRequested & CL_RECORD_COIL_B_CURRENT) 		{ sampleBuffer.PutI16(coilB); }

		sampleBuffer.FinishSample();
		++samplesCollected;
		if (samplesCollected == samplesRequested)
		{
			samplingMode = RecordingMode::SendingData;			// stop collecting data
		}
	}

	dataTransmissionTask->Give();
}

inline void ClosedLoop::ControlMotorCurrents(StepTimer::Ticks ticksSinceLastCall) noexcept
{
	// Get the time delta in seconds
	const float timeDelta = (float)ticksSinceLastCall * (1.0/(float)StepTimer::StepClockRate);
	float currentFraction;

	// Use a PID controller to calculate the required 'torque' - the control signal
	// We choose to use a PID control signal in the range -256 to +256. This is arbitrary.
	PIDPTerm = constrain<float>(Kp * currentError, -256.0, 256.0);
	PIDITerm = constrain<float>(PIDITerm + Ki * currentError * timeDelta, -PIDIlimit, PIDIlimit);	// constrain I to prevent it running away
	PIDDTerm = constrain<float>(Kd * errorDerivativeFilter.GetDerivative() * StepTimer::StepClockRate, -256.0, 256.0);	// constrain D so that we can graph it more sensibly after a sudden step input
	PIDVTerm = mParams.speed * Kv * ticksSinceLastCall;
	PIDATerm = mParams.acceleration * Ka * fsquare(ticksSinceLastCall);
	PIDControlSignal = constrain<float>(PIDPTerm + PIDITerm + PIDDTerm + PIDVTerm + PIDATerm, -256.0, 256.0);	// clamp the sum between +/- 256

	if (currentMode == ClosedLoopMode::foc)
	{
		// Calculate the offset required to produce the torque in the correct direction
		// i.e. if we are moving in the positive direction, we must apply currents with a positive phase shift
		// The max abs value of phase shift we want is 1 full step i.e. 25%.
		// Given that PIDControlSignal is -256 .. 256 and phase is 0 .. 4095
		// and that 25% of 4096 = 1024, our max phase shift = 4 * PIDControlSignal

		// New algorithm: phase of motor current is always +/- 1 full step relative to current position, but motor current is adjusted according to the PID result
		const float PhaseFeedForwardFactor = 1000.0;
		const int16_t phaseFeedForward = lrintf(constrain<float>(speedFilter.GetDerivative() * ticksSinceLastCall * PhaseFeedForwardFactor, -256.0, 256.0));
		phaseShift = (PIDControlSignal < 0) ? phaseFeedForward + (3 * 1024) : phaseFeedForward + 1024;
		currentFraction = fabsf(PIDControlSignal) * (1.0/256.0);

		// Calculate the required motor currents to induce that torque and reduce it modulo 4096
		// The following assumes that signed arithmetic is 2's complement
		const uint32_t measuredStepPhase = encoder->GetCurrentPhasePosition();
		desiredStepPhase = (uint16_t)((int16_t)measuredStepPhase + phaseShift) % 4096;
	}
	else
	{
		// Must be in semi-open-loop mode
		currentFraction = holdCurrentFraction + (1.0 - holdCurrentFraction) * min<float>(fabsf(PIDControlSignal * (1.0/256.0)), 1.0);
		const uint16_t stepPhase = (uint16_t)llrintf(mParams.position * 1024.0);
		desiredStepPhase = (stepPhase + phaseOffset) & 4095;
	}

	// Assert the required motor currents
	SetMotorPhase(desiredStepPhase, currentFraction);
}

void ClosedLoop::Diagnostics(const StringRef& reply) noexcept
{
	reply.printf("Closed loop mode: %s", (currentMode == ClosedLoopMode::foc) ? "field-oriented control" : (currentMode == ClosedLoopMode::semiOpen) ? "semi-open-loop" : "open loop");
	reply.catf(", pre-error threshold: %.2f, error threshold: %.2f", (double) errorThresholds[0], (double) errorThresholds[1]);
	reply.catf(", encoder type %s", GetEncoderType().ToString());
	if (encoder != nullptr)
	{
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
	if (currentMode != ClosedLoopMode::open)
	{
		reply.lcatf("Tuning mode: %#x, tuning error: %#x, collecting data: %s", tuning, tuningError, CollectingData() ? "yes" : "no");
		if (CollectingData())
		{
			reply.catf(" (filter: %#x, mode: %u, rate: %u, movement: %u)", filterRequested, samplingMode, (unsigned int)(StepTimer::StepClockRate/dataCollectionIntervalTicks), movementRequested);
		}

		reply.lcatf("Control loop runtime (us): min=%" PRIu32 ", max=%" PRIu32 ", frequency (Hz): min=%" PRIu32 ", max=%" PRIu32,
					TickPeriodToMicroseconds(minControlLoopRuntime), TickPeriodToMicroseconds(maxControlLoopRuntime),
					TickPeriodToFreq(maxControlLoopCallInterval), TickPeriodToFreq(minControlLoopCallInterval));
		ResetMonitoringVariables();
	}

	//DEBUG
	//reply.catf(", event status 0x%08" PRIx32 ", TCC2 CTRLA 0x%08" PRIx32 ", TCC2 EVCTRL 0x%08" PRIx32, EVSYS->CHSTATUS.reg, QuadratureTcc->CTRLA.reg, QuadratureTcc->EVCTRL.reg);
}

StandardDriverStatus ClosedLoop::ReadLiveStatus() const noexcept
{
	StandardDriverStatus result;
	result.all = 0;
	result.closedLoopPositionNotMaintained = stall;
	result.closedLoopPositionWarning = preStall;
	result.closedLoopNotTuned = ((tuningError & encoder->MinimalTuningNeeded()) != 0);
	result.closedLoopTuningError = ((tuningError & TuningError::AnyTuningFailure) != 0);
	return result;
}

void ClosedLoop::ResetError(size_t driver) noexcept
{
# if SINGLE_DRIVER
	if (driver == 0 && encoder != nullptr)
	{
		// Set the target position to the current position
		const bool err = encoder->TakeReading();
		(void)err;		//TODO handle error
		errorDerivativeFilter.Reset();
		speedFilter.Reset();
		SetTargetToCurrentPosition();
	}
# else
#  error Cannot support closed loop with the specified hardware
# endif
}

// This is called before the driver mode is changed. Return true if success. Always succeeds if we are disabling closed loop.
bool ClosedLoop::SetClosedLoopEnabled(size_t driver, ClosedLoopMode mode, const StringRef &reply) noexcept
{
	// Trying to enable closed loop
	if (driver != 0)
	{
		reply.copy("Invalid driver number");
		return false;
	}

	if (mode != ClosedLoopMode::open)
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
		const bool err = encoder->TakeReading();
		if (err)
		{
			reply.copy("Error reading encoder");
			return false;
		}

		if (encoder->UsesBasicTuning() && (tuningError & TuningError::NeedsBasicTuning) != 0)
		{
			encoder->SetTuningBackwards(false);
			encoder->SetKnownPhaseAtCurrentCount(initialStepPhase);
		}

		desiredStepPhase = initialStepPhase;							// set this to be picked up later in DriverSwitchedToClosedLoop
		PIDITerm = 0.0;
		errorDerivativeFilter.Reset();
		speedFilter.Reset();
		SetTargetToCurrentPosition();

		// Set the target position to the current position
		ResetError(0);													// this calls ReadState again and sets up targetMotorSteps

		ResetMonitoringVariables();										// to avoid getting stupid values
		prevControlLoopCallTime = StepTimer::GetTimerTicks();			// to avoid huge integral term windup
	}

	// If we are disabling closed loop mode, we should ideally send steps to get the microstep counter to match the current phase here
	currentMode = mode;

	return true;
}

// This is called just after the driver has switched into closed loop mode (it may have been in closed loop mode already)
void ClosedLoop::DriverSwitchedToClosedLoop(size_t driver) noexcept
{
	if (driver == 0)
	{
		delay(3);														// allow time for the switch to complete and a few control loop iterations to be done
		const uint16_t currentPhasePosition = (uint16_t)encoder->GetCurrentPhasePosition();
		if (currentMode == ClosedLoopMode::semiOpen)
		{
			const uint16_t stepPhase = (uint16_t)llrintf(mParams.position * 1024.0);
			phaseOffset = (currentPhasePosition - stepPhase) & 4095;
		}
		SetMotorPhase(currentPhasePosition, SmartDrivers::GetStandstillCurrentPercent(0) * 0.01);	// set the motor currents to match the initial position using the open loop standstill current
		PIDITerm = 0.0;													// clear the integral term accumulator
		errorDerivativeFilter.Reset();
		speedFilter.Reset();
		ResetMonitoringVariables();										// the first loop iteration will have recorded a higher than normal loop call interval, so start again
	}
}

// If we are in closed loop modify the driver status appropriately
StandardDriverStatus ClosedLoop::ModifyDriverStatus(size_t driver, StandardDriverStatus originalStatus) noexcept
{
	if (currentMode != ClosedLoopMode::open && driver == 0)
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
