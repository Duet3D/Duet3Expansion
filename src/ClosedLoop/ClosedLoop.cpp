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

#ifdef EXP1HCE
# include "QuadratureEncoderAttiny.h"
#endif

#ifdef EXP1HCL
# include "QuadratureEncoderPdec.h"
#endif

#include "TLI5012B.h"
#include "AttinyProgrammer.h"

static bool closedLoopEnabled = false;
static Encoder *encoder = nullptr;
static SharedSpiDevice *encoderSpi = nullptr;

#ifdef EXP1HCE

static AttinyProgrammer *programmer;

static void GenerateAttinyClock()
{
	// Currently we program the DPLL to generate 48MHz output, so to get 16MHz we divide by 3 and set the Improve Duty Cycle bit
	// We could instead program the DPLL to generate 96MHz and divide it by an extra factor of 2 in the other GCLKs
	// Or we could divide by 4 and be content with 12MHz.
	ConfigureGclk(ClockGenGclkNumber, GclkSource::dpll, 3, true);
	SetPinFunction(ClockGenPin, ClockGenPinPeriphMode);
}

void  ClosedLoop::TurnAttinyOff() noexcept
{
	programmer->TurnAttinyOff();
}

#endif

#ifdef EXP1HCL

static void GenerateTmcClock()
{
	// Currently we program DPLL0 to generate 120MHz output, so to get 15MHz we divide by 8
	ConfigureGclk(ClockGenGclkNumber, GclkSource::dpll0, 8, true);
	SetPinFunction(ClockGenPin, ClockGenPinPeriphMode);
}

#endif

void  ClosedLoop::EnableEncodersSpi() noexcept
{
	SetPinFunction(EncoderMosiPin, EncoderMosiPinPeriphMode);
	SetPinFunction(EncoderSclkPin, EncoderSclkPinPeriphMode);
	SetPinFunction(EncoderMisoPin, EncoderMisoPinPeriphMode);
}

void  ClosedLoop::DisableEncodersSpi() noexcept
{
	ClearPinFunction(EncoderMosiPin);
	ClearPinFunction(EncoderSclkPin);
	ClearPinFunction(EncoderMisoPin);
}

void ClosedLoop::Init() noexcept
{
	pinMode(EncoderCsPin, OUTPUT_HIGH);													// make sure that any attached SPI encoder is not selected
	encoderSpi = new SharedSpiDevice(EncoderSspiSercomNumber, EncoderSspiDataInPad);	// create the encoders SPI device

#if defined(EXP1HCE)
	GenerateAttinyClock();
	programmer = new AttinyProgrammer(*encoderSpi);
	programmer->InitAttiny();
#elif defined(EXP1HCL)
	GenerateTmcClock();
#endif
}

EncoderType ClosedLoop::GetEncoderType() noexcept
{
	return (encoder == nullptr) ? EncoderType::none : encoder->GetType();
}

GCodeResult ClosedLoop::ProcessM569Point1(const CanMessageGeneric &msg, const StringRef &reply) noexcept
{
	CanMessageGenericParser parser(msg, M569Point1Params);
	bool seen = false;
	uint8_t temp;

	// Check closed loop enable/disable
	if (parser.GetUintParam('S', temp))
	{
		seen = true;
		//TODO
		if (temp != 0)
		{
			reply.copy("Closed loop mode not supported yet");
			return GCodeResult::error;
		}
	}
	if (parser.GetUintParam('T', temp))
	{
		seen = true;
		if (temp < EncoderType::NumValues)
		{
			if (temp != GetEncoderType().ToBaseType())
			{
				//TODO need to get a lock here in case there is any movement
				delete encoder;
				switch (temp)
				{
				case EncoderType::none:
				default:
					break;

				case EncoderType::as5047:
					encoder = new AS5047D(*encoderSpi, EncoderCsPin);
					break;

				case EncoderType::tli5012:
					encoder = new TLI5012B(*encoderSpi, EncoderCsPin);
					break;

				case EncoderType::linearQuadrature:
#ifdef EXP1HCE
					encoder = new QuadratureEncoderAttiny(true);
#elif defined(EXP1HCL)
					encoder = new QuadratureEncoderPdec(true);
#else
# error Unknown board
#endif
					break;

				case EncoderType::rotaryQuadrature:
#ifdef EXP1HCE
					encoder = new QuadratureEncoderAttiny(false);
#elif defined(EXP1HCL)
					encoder = new QuadratureEncoderPdec(false);
#else
# error Unknown board
#endif
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

	if (!seen)
	{
		reply.printf("Closed loop mode %s, encoder type %s", (closedLoopEnabled) ? "enabled" : "disabled", GetEncoderType().ToString());
	}
	return GCodeResult::ok;
}

void ClosedLoop::Diagnostics(const StringRef& reply) noexcept
{
#if defined(EXP1HCE)
	reply.printf("Encoder programmed status %s, encoder type %s", programmer->GetProgramStatus().ToString(), GetEncoderType().ToString());
#elif defined(EXP1HCL)
	reply.printf("Encoder type %s", GetEncoderType().ToString());
#endif

	if (encoder != nullptr)
	{
		reply.catf(", position %" PRIi32, encoder->GetReading());
		encoder->AppendDiagnostics(reply);
	}

	//DEBUG
	//reply.catf(", event status 0x%08" PRIx32 ", TCC2 CTRLA 0x%08" PRIx32 ", TCC2 EVCTRL 0x%08" PRIx32, EVSYS->CHSTATUS.reg, QuadratureTcc->CTRLA.reg, QuadratureTcc->EVCTRL.reg);
}

#endif

// End
