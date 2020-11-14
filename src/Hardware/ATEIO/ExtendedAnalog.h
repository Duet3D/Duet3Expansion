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

namespace ExtendedAnalog
{
	void Init();
	uint16_t AnalogIn(unsigned int chan);
}

#endif

#endif /* SRC_HARDWARE_ATEIO_EXTENDEDANALOG_H_ */
