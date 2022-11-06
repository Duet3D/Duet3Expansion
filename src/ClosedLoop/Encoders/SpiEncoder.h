/*
 * SpiEncoder.h
 *
 *  Created on: 10 Jun 2020
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_SPIENCODER_H_
#define SRC_CLOSEDLOOP_SPIENCODER_H_

#include "Encoder.h"

#if SUPPORT_CLOSED_LOOP

#include <Hardware/SharedSpiClient.h>

class SpiEncoder
{
public:
	SpiEncoder(SharedSpiDevice& spiDev, uint32_t clockFreq, SpiMode m, bool polarity, Pin p_csPin) noexcept;

protected:
	SharedSpiClient spi;
	Pin csPin;
};

#endif

#endif /* SRC_CLOSEDLOOP_SPIENCODER_H_ */
