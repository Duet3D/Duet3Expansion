/*
 * MFMhandler.cpp
 *
 *  Created on: 15 Jan 2024
 *      Author: David
 */

#include "MFMhandler.h"

#if SUPPORT_AS5601

#include <Hardware/AS5601.h>
#include <Hardware/TCA6408A.h>
#include <Platform/TaskPriorities.h>

namespace MFMHandler
{
	constexpr uint32_t PollIntervalMillis = 40;						// how many milliseconds between device polling
	constexpr size_t AS5601TaskStackWords = 100;
	constexpr unsigned int buttonDebounceCount = 10;

	AS5601 *encoder = nullptr;
	TCA6408A *expander = nullptr;
	unsigned int buttonCount = 0;									// incremented (up to max. buttonDebounceCount) when button is read down, decremented when button is read up
	bool buttonState = false;										// true if button is pressed, after doing debouncing
	bool mfmActive = false;

	Task<AS5601TaskStackWords> *as5601Task = nullptr;

	[[noreturn]] void MfmTaskCode(void*) noexcept;
}

void MFMHandler::Init(SharedI2CMaster& i2cDevice) noexcept
{
	if (encoder == nullptr)
	{
		encoder = new AS5601(i2cDevice);
		if (!encoder->Init())
		{
			DeleteObject(encoder);
		}
		expander = new TCA6408A(i2cDevice);
		if (!expander->Init())
		{
			DeleteObject(expander);
		}
	}

	if (encoder != nullptr || expander != nullptr)
	{
		as5601Task = new Task<AS5601TaskStackWords>();
		as5601Task->Create(MfmTaskCode, "MFM", nullptr, TaskPriority::Mfm);
	}
}

bool MFMHandler::IsButtonPressed() noexcept
{
	return expander != nullptr && expander->IsButtonPressed();
}

void MFMHandler::AppendDiagnostics(const StringRef& reply) noexcept
{
	reply.lcatf("Integrated AS5601%s found, TCA6408A%s found", (EncoderPresent()) ? "" : " not", ExpanderPresent() ? "" : " not");
}

bool MFMHandler::EncoderPresent() noexcept
{
	return encoder != nullptr;
}

bool MFMHandler::ExpanderPresent() noexcept
{
	return expander != nullptr;
}

void MFMHandler::Start() noexcept
{
	//TODO
}

void MFMHandler::Stop() noexcept
{
	//TODO
}

void MFMHandler::MfmTaskCode(void *) noexcept
{
	uint32_t nextWakeTime = millis();
	for (;;)
	{
		// Wait until we are woken or it's time to poll the I2C devices. If we are really unlucky, we could end up waiting for one tick too long.
		nextWakeTime += HeatSampleIntervalMillis;
		{
			const uint32_t now = millis();
			int32_t delayTime = (int32_t)(nextWakeTime - now);
			// If we're late (e.g. due to problems sending CAN messages), wait at least until the next tick to let other tasks run
			if (delayTime < 1)
			{
				nextWakeTime = now + 1;
				delayTime = 1;
			}
			delay((uint32_t)delayTime);
		}

		if (encoder != nullptr)
		{
			encoder->Poll();
		}
		if (expander != nullptr)
		{
			expander->Poll();
		}
	}
}

#endif
