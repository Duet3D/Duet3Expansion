/*
 * AngleSensor.h
 *
 *  Created on: 10 Jun 2020
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_ANGLESENSOR_H_
#define SRC_CLOSEDLOOP_ANGLESENSOR_H_

#include <RepRapFirmware.h>

#if SUPPORT_CLOSED_LOOP

class AngleSensor
{
public:
	AngleSensor();

	virtual void Init() = 0;
	virtual int16_t GetAngle() = 0;
	virtual void Diagnostics(const StringRef& reply);
};

#endif

#endif /* SRC_CLOSEDLOOP_ANGLESENSOR_H_ */
