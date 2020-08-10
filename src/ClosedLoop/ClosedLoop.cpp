/*
 * ClosedLoop.cpp
 *
 *  Created on: 9 Jun 2020
 *      Author: David
 */

#include "ClosedLoop.h"

#if SUPPORT_CLOSED_LOOP

#include <CanMessageGenericParser.h>
#include "ClockGen.h"
#include "SpiEncoder.h"
#include "AS5047D.h"
#include "QuadratureEncoder.h"
#include "TLI5012B.h"
#include "AttinyProgrammer.h"

static bool closedLoopEnabled = false;
static Encoder *encoder = nullptr;
static SharedSpiDevice *encoderSpi = nullptr;
static AttinyProgrammer *programmer;

void  ClosedLoop::EnableEncodersSpi() noexcept
{
#ifdef EXP1HCE
	gpio_set_pin_function(EncoderMosiPin, EncoderMosiPinPeriphMode);
	gpio_set_pin_function(EncoderSclkPin, EncoderSclkPinPeriphMode);
	gpio_set_pin_function(EncoderMisoPin, EncoderMisoPinPeriphMode);
#else
# error Undefined hardware
#endif
}

void  ClosedLoop::DisableEncodersSpi() noexcept
{
#ifdef EXP1HCE
	gpio_set_pin_function(EncoderMosiPin, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_function(EncoderSclkPin, GPIO_PIN_FUNCTION_OFF);
	gpio_set_pin_function(EncoderMisoPin, GPIO_PIN_FUNCTION_OFF);
#else
# error Undefined hardware
#endif
}

void ClosedLoop::Init() noexcept
{
	pinMode(EncoderCsPin, OUTPUT_HIGH);									// make sure that any attached SPI encoder is not selected
	encoderSpi = new SharedSpiDevice(EncoderSspiSercomNumber);			// create the encoders SPI device
	ClockGen::Init();
	programmer = new AttinyProgrammer(*encoderSpi);
	programmer->InitAttiny();
}

void  ClosedLoop::TurnAttinyOff() noexcept
{
	programmer->TurnAttinyOff();
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

	if (!seen)
	{
		reply.printf("Closed loop mode %s, encoder type %s", (closedLoopEnabled) ? "enabled" : "disabled", GetEncoderType().ToString());
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
}

#endif

// End
