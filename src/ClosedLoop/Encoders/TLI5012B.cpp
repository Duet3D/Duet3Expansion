/*
 * TLI5012B.cpp
 *
 *  Created on: 10 Jun 2020
 *      Author: David
 */

#include "TLI5012B.h"

#if SUPPORT_CLOSED_LOOP

constexpr unsigned int TLI5012BResolutionBits = 14;

TLI5012B::TLI5012B(uint32_t p_stepsPerRev, SharedSpiDevice& spiDev, Pin p_csPin) noexcept
	: SpiEncoder(spiDev, 60000, SpiMode::mode1, false, p_csPin),	//TODO use correct frequency and mode
	  AbsoluteRotaryEncoder(p_stepsPerRev, TLI5012BResolutionBits)
{
}

// Initialise the encoder and enable it if successful. If there are any warnings or errors, put the corresponding message text in 'reply'.
GCodeResult TLI5012B::Init(const StringRef& reply) noexcept
{
	//TODO
	reply.copy("This encoder type is not supported");
	return GCodeResult::error;
}

void TLI5012B::Enable() noexcept
{
	//TODO
}

void TLI5012B::Disable() noexcept
{
	//TODO
}

bool TLI5012B::GetRawReading() noexcept
{
	//TODO
	return false;
}

void TLI5012B::AppendDiagnostics(const StringRef &reply) noexcept
{
	//TODO
	reply.cat("TLI5012B diagnostics not implemented");
}

void TLI5012B::AppendStatus(const StringRef& reply) noexcept
{
	//TODO
}

#endif
