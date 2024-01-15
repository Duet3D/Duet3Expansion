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
		encoder->Init();
		expander = new TCA6408A(i2cDevice);
		expander->Init();
	}
}

void MFMHandler::AppendDiagnostics(const StringRef& reply) noexcept
{
	reply.lcatf("Integrated AS5601%s found, TCA6408A%s found", (EncoderPresent()) ? "" : " not", ExpanderPresent() ? "" : " not");
}

bool MFMHandler::EncoderPresent() noexcept
{
	return encoder != nullptr && encoder->Present();
}

bool MFMHandler::ExpanderPresent() noexcept
{
	return expander != nullptr && expander->Present();
}

void MFMHandler::Start() noexcept
{
	if (as5601Task == nullptr)
	{
		as5601Task = new Task<AS5601TaskStackWords>();
		as5601Task->Create(MfmTaskCode, "MFM", nullptr, TaskPriority::Mfm);
	}
	//TODO
}

void MFMHandler::Stop() noexcept
{
	//TODO
}

void MfmTaskCode() noexcept
{
	for(;;)
	{
		delay(100);
	}
}

#endif
