/*
 * TLI5012B.cpp
 *
 *  Created on: 10 Jun 2020
 *      Author: David
 */

#include <ClosedLoop/TLI5012B.h>

#if SUPPORT_CLOSED_LOOP

TLI5012B::TLI5012B(SharedSpiDevice& p_spi) : spi(p_spi)
{
	// TODO Auto-generated constructor stub

}

void TLI5012B::Init()
{
}

int16_t TLI5012B::GetAngle()
{
	//TODO
	return 0;
}

void TLI5012B::Diagnostics(const StringRef &reply)
{
	//TODO
	reply.copy("not implemented");
}

#endif
