/*
 * ExtendedAnalog.h
 *
 *  Created on: 14 Nov 2020
 *      Author: David
 */

#ifndef SRC_HARDWARE_ATEIO_EXTENDEDANALOG_H_
#define SRC_HARDWARE_ATEIO_EXTENDEDANALOG_H_

#ifdef ATEIO

#include <RepRapFirmware.h>

class SharedSpiDevice;

namespace ExtendedAnalog
{
	void Init(SharedSpiDevice& sharedSpi) noexcept;
	uint16_t AnalogIn(unsigned int chan) noexcept;
}

#endif

#endif /* SRC_HARDWARE_ATEIO_EXTENDEDANALOG_H_ */
