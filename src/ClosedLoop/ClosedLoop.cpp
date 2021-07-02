/*
 * ClosedLoop.cpp
 *
 *  Created on: 9 Jun 2020
 *      Author: David
 */

#include "ClosedLoop.h"

#if SUPPORT_CLOSED_LOOP

#include <CanMessageGenericParser.h>
#include "SpiEncoder.h"
#include "AS5047D.h"
#include "QuadratureEncoder.h"
#include "TLI5012B.h"
#include "AttinyProgrammer.h"
#include <TaskPriorities.h>

static bool closedLoopEnabled;
static bool stepDirection;
static float targetMotorSteps;

static float encoderCountPerUnit = 1;
static Encoder *encoder = nullptr;
static SharedSpiDevice *encoderSpi = nullptr;
static AttinyProgrammer *programmer;

constexpr size_t ClosedLoopTaskStackWords = 200;
static Task<ClosedLoopTaskStackWords> *closedLoopTask;

static void GenerateAttinyClock()
{
	// Currently we program the DPLL to generate 48MHz output, so to get 16MHz we divide by 3 and set the Improve Duty Cycle bit
	// We could instead program the DPLL to generate 96MHz and divide it by an extra factor of 2 in the other GCLKs
	// Or we could divide by 4 and be content with 12MHz.
	ConfigureGclk(ClockGenGclkNumber, GclkSource::dpll, 3, true);
	SetPinFunction(ClockGenPin, ClockGenPinPeriphMode);
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
//	static_cast<ClosedLoop*>(param)->TaskLoop();
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

	// Set up the task
	closedLoopTask = new Task<ClosedLoopTaskStackWords>;
	closedLoopTask->Create(ClosedLoopLoop, "Move", nullptr, TaskPriority::ClosedLoop);
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
# if USE_TC_FOR_STEP || !SINGLE_DRIVER
		// TODO: Check what we need to do here
		reply.copy("Closed loop drive mode not yet supported")
		return false;
# endif
		if (encoder == nullptr)
		{
			reply.copy("No encoder specified for closed loop drive mode");
			return false;
		}
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
	targetMotorSteps += (stepDirection ? 1 : -1) * 1;	// TODO: Need to * by step angle
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
	reply.catf("closedLoopEnabled = %i\n", closedLoopEnabled);
	reply.cat("~~~~~\n");

	//DEBUG
	//reply.catf(", event status 0x%08" PRIx32 ", TCC2 CTRLA 0x%08" PRIx32 ", TCC2 EVCTRL 0x%08" PRIx32, EVSYS->CHSTATUS.reg, QuadratureTcc->CTRLA.reg, QuadratureTcc->EVCTRL.reg);
}

[[noreturn]] void ClosedLoop::TaskLoop() noexcept
{
	while (true)
	{
		targetMotorSteps++;
		TaskBase::Take();
	}
}

#endif

// End
