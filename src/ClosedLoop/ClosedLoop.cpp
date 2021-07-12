/*
 * ClosedLoop.cpp
 *
 *  Created on: 9 Jun 2020
 *      Author: David
 */

#include "ClosedLoop.h"

#if SUPPORT_CLOSED_LOOP

#include <math.h>
#include <Platform.h>
#include <CanMessageGenericParser.h>
#include "SpiEncoder.h"
#include "AS5047D.h"
#include "QuadratureEncoder.h"
#include "TLI5012B.h"
#include "AttinyProgrammer.h"
#include <TaskPriorities.h>
#include <CAN/CanInterface.h>
#include <CanMessageBuffer.h>
#include <CanMessageFormats.h>


#if SUPPORT_TMC2160
# include "Movement/StepperDrivers/TMC51xx.h"
#endif

// Note: Unusually, this file works in degrees instead of radians.
// 		 This is mainly due to not being required to work in fractions
//		 of a degree (the majority encoders have a resolution that does not
//		 require more decision than a degree). This means we may as well
//		 work in integer degrees, as working in float radians has no advantage.

static bool closedLoopEnabled;
static bool stepDirection;
static float targetMotorSteps;
static bool zeroing = false;	// Set to true when the motor is in the zero-ing process

// Data collection
static bool collectingData = false;
static uint16_t samplesRequested = 0;
static int32_t encoderReading;
static int32_t targetReading;
static int32_t controlReading;

static float absMotorSteps;		// For Debug - remove
static uint32_t prevReg, prevCoilA, prevCoilB;		// For Debug - remove
static uint32_t loopCounter=0;	// DEBUG
static float angle = 0;

static float encoderCountPerUnit = 1;
static Encoder *encoder = nullptr;
static SharedSpiDevice *encoderSpi = nullptr;
static AttinyProgrammer *programmer;

// PID tuning variables
static float Kp = 10;
static float Ki = 0;
static float Kd = 10;


// TODO: Rename this task to be a bit more descriptive
constexpr size_t ClosedLoopTaskStackWords = 200;
static Task<ClosedLoopTaskStackWords> *closedLoopTask;

constexpr uint32_t ClosedLoopPIDIntervalMillis = 1;		// Interval between iterations of the PID loop

constexpr size_t ClosedLoopDataCollectionTaskStackWords = 200;
static Task<ClosedLoopDataCollectionTaskStackWords> *closedLoopDataCollectionTask;

// Masks for each coil register
const uint32_t COIL_A_MASK = 0x000001FF;
const uint32_t COIL_B_MASK = 0x01FF0000;

static void GenerateAttinyClock()
{
	// Currently we program the DPLL to generate 48MHz output, so to get 16MHz we divide by 3 and set the Improve Duty Cycle bit
	// We could instead program the DPLL to generate 96MHz and divide it by an extra factor of 2 in the other GCLKs
	// Or we could divide by 4 and be content with 12MHz.
	ConfigureGclk(ClockGenGclkNumber, GclkSource::dpll, 3, true);
	SetPinFunction(ClockGenPin, ClockGenPinPeriphMode);
}

// Helper function to set a given coil current in
void SetMotorCurrents(int32_t coilA, int32_t coilB)
{
# if SUPPORT_TMC2160 && SINGLE_DRIVER
	SmartDrivers::SetRegister(0,
			SmartDriverRegister::xDirect,
			((coilB << 16) & COIL_B_MASK) | (coilA & COIL_A_MASK));
# else
#  error Cannot support closed loop with the specified hardware
# endif
}

void ZeroStepper()
{

	// Given we are setting motor currents directly, set the motor to enabled
	// (we can disable by setting the currents to 0 ourselves
# if SUPPORT_TMC2160 && SINGLE_DRIVER
	Platform::EnableDrive(0);
# else
#  error Undefined hardware
# endif

	switch (ClosedLoop::GetEncoderType().ToBaseType())
	{
	case EncoderType::rotaryQuadrature:
# if SUPPORT_TMC2160 && SINGLE_DRIVER
		// Force the motor to 0 phase by setting coilA = sin(0) and coilB = cos(0)
		SmartDrivers::SetRegister(0,
				SmartDriverRegister::xDirect,
				((255 << 16) & COIL_B_MASK) | (0 & COIL_A_MASK));
# else
#  error Undefined hardware
# endif
		break;
	}

	zeroing = true;
}

void  ClosedLoop::EnableEncodersSpi() noexcept
{
#ifdef EXP1HCE
	SetPinFunction(EncoderMosiPin, EncoderMosiPinPeriphMode);
	SetPinFunction(EncoderSclkPin, EncoderSclkPinPeriphMode);
	SetPinFunction(EncoderMisoPin, EncoderMisoPinPeriphMode);
#else
# error Undefined hardware
#endif
}

void  ClosedLoop::DisableEncodersSpi() noexcept
{
#ifdef EXP1HCE
	ClearPinFunction(EncoderMosiPin);
	ClearPinFunction(EncoderSclkPin);
	ClearPinFunction(EncoderMisoPin);
#else
# error Undefined hardware
#endif
}

extern "C" [[noreturn]] void ClosedLoopLoop(void * param) noexcept
{
	ClosedLoop::TaskLoop();
}

extern "C" [[noreturn]] void ClosedLoopDataCollectionLoop(void * param) noexcept
{
	ClosedLoop::DataCollectionLoop();
}

void ClosedLoop::Init() noexcept
{
	pinMode(EncoderCsPin, OUTPUT_HIGH);													// make sure that any attached SPI encoder is not selected
	encoderSpi = new SharedSpiDevice(EncoderSspiSercomNumber, EncoderSspiDataInPad);	// create the encoders SPI device
	GenerateAttinyClock();
	programmer = new AttinyProgrammer(*encoderSpi);
	programmer->InitAttiny();

	// Set default values
	closedLoopEnabled = false;
	stepDirection = true;
	targetMotorSteps = 0;
	absMotorSteps = 0;

	// Set up the PID task
	closedLoopTask = new Task<ClosedLoopTaskStackWords>;
	closedLoopTask->Create(ClosedLoopLoop, "ClosedLoopPID", nullptr, TaskPriority::ClosedLoop);

	// Set up the data collection task
	closedLoopDataCollectionTask = new Task<ClosedLoopDataCollectionTaskStackWords>;
	closedLoopDataCollectionTask->Create(ClosedLoopDataCollectionLoop, "ClosedLoopDataCollect", nullptr, TaskPriority::ClosedLoop);
}

bool ClosedLoop::GetClosedLoopEnabled() noexcept
{
	return closedLoopEnabled;
}

bool ClosedLoop::SetClosedLoopEnabled(bool enabled, const StringRef &reply) noexcept
{

	if (enabled) {
# if SUPPORT_SLOW_DRIVERS
		// TODO: Check what we need to do here
		if (Platform::IsSlowDriver()) {
			reply.copy("Closed loop drive mode not yet supported")
			return false;
		}
# endif
# if USE_TC_FOR_STEP || !SINGLE_DRIVER || !SUPPORT_TMC2160
		// TODO: Check what we need to do here
		reply.copy("Closed loop drive mode not yet supported")
		return false;
# endif
		if (encoder == nullptr)
		{
			reply.copy("No encoder specified for closed loop drive mode");
			return false;
		}
		ZeroStepper();
	}

	closedLoopEnabled = enabled;
	return true;
}

void ClosedLoop::SetStepDirection(bool dir) noexcept
{
	stepDirection = dir;
}

void  ClosedLoop::TurnAttinyOff() noexcept
{
	programmer->TurnAttinyOff();
}

EncoderType ClosedLoop::GetEncoderType() noexcept
{
	return (encoder == nullptr) ? EncoderType::none : encoder->GetType();
}

int32_t ClosedLoop::GetEncoderReading() noexcept
{
	if (encoder == nullptr)
	{
		return 0;
	}
	else
	{
		return encoder->GetReading() * encoderCountPerUnit;
	}
}

void ClosedLoop::TakeStep() noexcept
{
	bool interpolation;	// TODO: Work out what this is for!
# if HAS_SMART_DRIVERS && SINGLE_DRIVER
	float microstepAngle = 1.0/SmartDrivers::GetMicrostepping(0, interpolation);
# else
#  error Undefined hardware
# endif
	targetMotorSteps += (stepDirection ? microstepAngle : -microstepAngle) * 1.8;	// TODO: Need to * by actual step angle
	if (!zeroing)
	{
		closedLoopTask->Give();
	}
}

GCodeResult ClosedLoop::ProcessM569Point1(const CanMessageGeneric &msg, const StringRef &reply) noexcept
{
	// TODO: THIS NEEDS TO BE REFACTORED NOW WE'RE USING M569 instead
	// --------------------------------------------------------------

	CanMessageGenericParser parser(msg, M569Point1Params);
	bool seen = false;
	uint8_t tempUint;
	float tempFloat;

	bool finalClosedLoopEnabled = closedLoopEnabled;	// The 'final' state the user intends closedLoopEnabled to be

	// Check closed loop enable/disable
	if (parser.GetUintParam('S', tempUint))
	{
		seen = true;

		switch(tempUint) {
		case 0:
			finalClosedLoopEnabled = false;
			break;
		case 1:
# if SUPPORT_SLOW_DRIVERS
			// TODO: Check what we need to do here
			if (Platform::IsSlowDriver()) {
				reply.copy("Closed loop mode not yet supported")
				return GCodeResult::error;
			}
#endif
# if USE_TC_FOR_STEP || !SINGLE_DRIVER
			// TODO: Check what we need to do here
			reply.copy("Closed loop mode not yet supported")
			return GCodeResult::error;
#else
			finalClosedLoopEnabled = true;
			break;
#endif
		}
	}

	// Check encoder type
	if (parser.GetUintParam('T', tempUint))
	{
		seen = true;
		if (tempUint < EncoderType::NumValues)
		{
			if (tempUint != GetEncoderType().ToBaseType())
			{
				// Check that the user is requesting a valid selection
				if (finalClosedLoopEnabled && tempUint == EncoderType::none)
				{
					reply.copy("Invalid encoder type for closed loop mode S1");
					return GCodeResult::error;
				}

				//TODO need to get a lock here in case there is any movement
				delete encoder;
				switch (tempUint)
				{
				case EncoderType::none:
				default:
					encoder = nullptr;
					break;

				case EncoderType::as5047:
					encoder = new AS5047D(*encoderSpi, EncoderCsPin);
					break;

				case EncoderType::tli5012:
					encoder = new TLI5012B(*encoderSpi, EncoderCsPin);
					break;

				case EncoderType::linearQuadrature:
					encoder = new QuadratureEncoder(true);
					break;

				case EncoderType::rotaryQuadrature:
					encoder = new QuadratureEncoder(false);
					break;
				}

				if (encoder != nullptr)
				{
					encoder->Enable();
				}
			}
		}
		else
		{
			reply.copy("Encoder type out of range");
			return GCodeResult::error;
		}
	}
	else
	{
		// If no encoder has been specified, and none is already selected
		// Then we cannot be in closed loop mode
		if (encoder == nullptr && finalClosedLoopEnabled)
		{
			reply.copy("Invalid encoder type for closed loop mode S1");
			return GCodeResult::error;
		}
	}

	// 'commit' the finalClosedLoopEnabled value now no errors have been thrown
	if (closedLoopEnabled != finalClosedLoopEnabled)
	{
		closedLoopEnabled = finalClosedLoopEnabled;
	}

	// Check encoder count per unit value
	if (parser.GetFloatParam('E', tempFloat))
	{
		encoderCountPerUnit = tempFloat;
	}

	if (!seen)
	{
		reply.printf("Closed loop mode %s", (closedLoopEnabled) ? "enabled" : "disabled");
		reply.catf(", encoder type %s", GetEncoderType().ToString());
		reply.catf(", encoder steps per unit %f", (double) encoderCountPerUnit);
	}
	return GCodeResult::ok;
}

void ClosedLoop::Diagnostics(const StringRef& reply) noexcept
{
	reply.printf("Encoder programmed status %s, encoder type %s", programmer->GetProgramStatus().ToString(), GetEncoderType().ToString());
	if (encoder != nullptr)
	{
		reply.catf(", position %" PRIi32, encoder->GetReading());
		encoder->AppendDiagnostics(reply);
	}

	//DEBUG
	reply.cat("\n~~~~~\n");
	reply.catf("targetMotorSteps = %f\n", (double) targetMotorSteps);
	reply.catf("absMotorSteps = %f\n", (double) absMotorSteps);
	reply.catf("closedLoopEnabled = %i\n", closedLoopEnabled);
//	if (encoder != nullptr)
//	{
//		const int32_t encoderStepsPer4Steps = encoderCountPerUnit * 4.0;
//		reply.catf("absPhase = %f\n", (double) (2*PI * (encoder->GetReading() % (int32_t) encoderStepsPer4Steps) / encoderStepsPer4Steps));
//	}

	reply.catf("angle = %f\n", (double) angle);

	reply.catf("prevReg = %x\n", prevReg);
	reply.catf("prevCoilA = %i\n", prevCoilA);
	reply.catf("prevCoilB = %i\n", prevCoilB);
	reply.catf("zeroReg = %x\n", ((255 << 16) & COIL_B_MASK) | (0 & COIL_A_MASK));

	reply.catf("loopCounter = %d\n", loopCounter);

	reply.catf("encoderReading = %d\n", encoderReading);
	reply.catf("targetReading = %d\n", targetReading);
	reply.catf("controlReading = %d\n", controlReading);



//	static char b[19];
//	b[0] = '\0';
//	int z;
//	for (z = 131072; z > 0; z >>= 1)
//	{
//		reply.catf(((prevReg & z) == z) ? "1" : "0");
//	}
//	reply.catf("\n");
//
//	reply.cat("~~~~~\n");

	//DEBUG
	//reply.catf(", event status 0x%08" PRIx32 ", TCC2 CTRLA 0x%08" PRIx32 ", TCC2 EVCTRL 0x%08" PRIx32, EVSYS->CHSTATUS.reg, QuadratureTcc->CTRLA.reg, QuadratureTcc->EVCTRL.reg);
}

// TODO: Give this task loop a more descriptive name
[[noreturn]] void ClosedLoop::TaskLoop() noexcept
{
	// PID variables
	int32_t PIDControlSignal;
	int32_t err,lastError = 0;
	int32_t PIDPTerm,PIDITerm = 0,PIDDTerm;

	uint32_t lastWakeTime = xTaskGetTickCount();

	while (true)
	{
		// TODO: Can we enable/disable this task when closed loop is turned on/off
		if (closedLoopEnabled)
		{
			// If we are zeroing, deal with that
			if (zeroing)
			{
				// Wait for the register to have written
# if SUPPORT_TMC2160 && SINGLE_DRIVER
				while (SmartDrivers::UpdatePending(0)) {
					TaskBase::Take(10);
				}
# else
#  error Cannot support closed loop with the specified hardware
# endif
				// Wait for the motor to have finished moving
				TaskBase::Take(100);

				// Zero the encoder
				((QuadratureEncoder*) encoder)->SetReading(0);

				// Set the motor currents back to zero
				SetMotorCurrents(0, 0);

				// We are now done zero-ing
				zeroing = false;
			}

			// Calculate the current position & phase from the encoder reading
			int32_t tempEncoderReading = encoder->GetReading();
			float currentMotorSteps = tempEncoderReading / encoderCountPerUnit;

//			const int32_t encoderStepsPer4Steps = encoderCountPerUnit * 4.0;
//			float absPhase = 0;//2*PI * (tempEncoderReading % (int32_t) encoderStepsPer4Steps) / encoderStepsPer4Steps;

			uint16_t stepPhase;		// 0-4095 value representing the phase *within* the current step
			float temp = currentMotorSteps / 4;
			if (temp >= 0) {
				stepPhase = (temp - (int) temp) * 4095;
			} else {
				stepPhase = (temp - (int) temp + 1) * 4095;
			}

			// Use a PID controller to calculate the required 'torque' - the control signal
			err = targetMotorSteps - currentMotorSteps;
			PIDPTerm = Kp * err;
			PIDITerm += Ki * err;
			PIDDTerm = Kd * (lastError - err);
			PIDControlSignal = PIDPTerm + PIDITerm + PIDDTerm;
			PIDControlSignal = constrain<int32_t>(PIDControlSignal, -255, 255);	// Clamp between -255 and 255

			// Store the readings if we are collecting data
			// TODO: We should probably have some kind of lock here (?)
			if (true)
			{
				encoderReading = currentMotorSteps;
				targetReading = (int32_t) targetMotorSteps;
				controlReading = PIDControlSignal;
			}

			// Calculate the offset required to produce the torque in the correct direction
			// i.e. if we are moving in the positive direction, we must apply currents with a positive phase shift
			// The max abs value of phase shift we want is 25%.
			// Given that PIDControlSignal is -255 .. 255 and phase is 0 .. 4095
			// and that 25% of 4095 ~= 1024, our phase shift ~= 4 * PIDControlSignal
			// DEBUG: For now a max phase shift of 12.5% is all we are using, so do 2 * PIDControlSignal
			int16_t phaseShift = 2 * PIDControlSignal;
//			float phaseShift = 0;//(PIDControlSignal / 255.0) * 0.1 * 2*PI;

			// Calculate the required motor currents to induce that torque
			// TODO: Implement sin/cos as lookups
			// TODO: Have a minimum holding current
//			int32_t coilB = cos(absPhase + phaseShift) * abs(PIDControlSignal);
//			int32_t coilA = sin(absPhase + phaseShift) * abs(PIDControlSignal);
			// (If stepPhase < phaseShift, we need to add on an extra 4095 to bring us back within the correct range)
			uint16_t desiredStepPhase = stepPhase + phaseShift + ((stepPhase < phaseShift) * 4095);

			int32_t coilA = sin(desiredStepPhase) * abs(PIDControlSignal);
			int32_t coilB = cos(desiredStepPhase) * abs(PIDControlSignal);

			// Assert the required motor currents
			SetMotorCurrents(coilA, coilB);

			// Update vars for the next cycle
			lastError = err;
		}

		// We need a timeout here because the condition for doing something cannot always
		// trigger an interrupt - e.g. motor being moved by hand
		// TODO: Do this in a better way
		TaskBase::Take(1);
//		vTaskDelayUntil(&lastWakeTime, ClosedLoopPIDIntervalMillis);
	}
}

GCodeResult ClosedLoop::StartDataCollection(const CanMessageStartClosedLoopDataCollection& msg, const StringRef& reply) noexcept
{
	if (msg.deviceNumber != 0 || encoder == nullptr)
	{
		reply.printf("Drive is not in closed loop mode");
		return GCodeResult::error;
	}

	if (collectingData)
	{
		reply.printf("Drive is already collecting data");
		return GCodeResult::error;
	}

	collectingData = true;
	samplesRequested = msg.numSamples;
//	closedLoopDataCollectionTask->Give();
	return GCodeResult::ok;
}

[[noreturn]] void ClosedLoop::DataCollectionLoop() noexcept
{
	while (true)
	{
		// If we are not collecting data, block the task
		if (!collectingData)
		{
			TaskBase::Take();
		}

		// Loop for each sample
		for (uint16_t i = 0; i < samplesRequested; i++)
		{
			loopCounter = i;

			{
				// Set up a CAN message
				CanMessageBuffer buf(nullptr);
				CanMessageClosedLoopData& msg = *(buf.SetupStatusMessage<CanMessageClosedLoopData>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress()));

				// DEBUG: Send a dummy packet
				msg.encoderData[0] = encoderReading;
				msg.targetData[0] = targetReading;
				msg.controlData[0] = controlReading;

				msg.firstSampleNumber = i;
				msg.numSamples = 1;
				msg.actualSampleRate = 0;
				msg.lastPacket = (i == samplesRequested - 1);

				buf.dataLength = msg.GetActualDataLength();
				CanInterface::Send(&buf);
			}

			// Pause to maintain the sample rate (TODO: Implement variable sample rate)
			TaskBase::Take(10);
		}

		// Mark that we have finished collecting data
		collectingData = false;
	}
}

#endif

// End
