/*
 * Serial.h
 *
 *  Created on: 9 Aug 2019
 *      Author: David
 */

#ifndef SRC_SERIAL_H_
#define SRC_SERIAL_H_

#include "RepRapFirmware.h"

namespace Serial
{
	void EnableSercomClock(uint8_t sercomNumber);
	void InitUart(Sercom * sercom, uint8_t SercomNumber, uint32_t baudRate);
	void Disable(Sercom * sercom);
}

#endif /* SRC_SERIAL_H_ */
