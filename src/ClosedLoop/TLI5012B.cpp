/*
 * TLI5012B.cpp
 *
 *  Created on: 10 Jun 2020
 *      Author: David
 */

#include <ClosedLoop/TLI5012B.h>

#if SUPPORT_CLOSED_LOOP

TLI5012B::TLI5012B(Pin p_csPin) noexcept : SpiEncoder(60000, SpiMode::mode1, false, p_csPin)	//TODO use correct frequency and mode
{
}

void TLI5012B::Enable() noexcept
{
	//TODO
}

void TLI5012B::Disable() noexcept
{
	//TODO
}

int32_t TLI5012B::GetReading() noexcept
{
	//TODO
	return 0;
}

void TLI5012B::Diagnostics(const StringRef &reply) noexcept
{
	//TODO
	reply.copy("not implemented");
}

#endif
