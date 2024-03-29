/*
 * TLI5012B.cpp
 *
 *  Created on: 10 Jun 2020
 *      Author: David
 */

#include <ClosedLoop/TLI5012B.h>

#if SUPPORT_CLOSED_LOOP

TLI5012B::TLI5012B(SharedSpiDevice& spiDev, Pin p_csPin) noexcept
: SpiEncoder(spiDev, 60000, SpiMode::mode1, false, p_csPin),	//TODO use correct frequency and mode
  AbsoluteEncoder()
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

uint32_t TLI5012B::GetAbsolutePosition(bool& error) noexcept
{
	//TODO
	return 0;
}

void TLI5012B::AppendDiagnostics(const StringRef &reply) noexcept
{
	//TODO
	reply.cat(", TLI5012B diagnostics not implemented");
}

void TLI5012B::AppendStatus(const StringRef& reply) noexcept
{
	//TODO
}

#endif
