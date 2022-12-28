/*
 * SpiEncoder.cpp
 *
 *  Created on: 10 Jun 2020
 *      Author: David
 */

#include "SpiEncoder.h"

#if SUPPORT_CLOSED_LOOP

SpiEncoder::SpiEncoder(SharedSpiDevice& spiDev, uint32_t clockFreq, SpiMode m, bool polarity, Pin p_csPin)
	: spi(spiDev, clockFreq, m, p_csPin, polarity), csPin(p_csPin)
{
	spi.SetCsPin(p_csPin);
}

#endif
