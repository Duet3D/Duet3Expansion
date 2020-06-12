/*
 * TLI5012B.h
 *
 *  Created on: 10 Jun 2020
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_TLI5012B_H_
#define SRC_CLOSEDLOOP_TLI5012B_H_

#include "AngleSensor.h"

#if SUPPORT_CLOSED_LOOP

#include <Hardware/SharedSpiDevice.h>

class TLI5012B : public AngleSensor
{
public:
	TLI5012B(SharedSpiDevice& p_spi);

	void Init() override;
	int16_t GetAngle() override;
	void Diagnostics(const StringRef& reply) override;

private:
	SharedSpiDevice& spi;
};

#endif

#endif /* SRC_CLOSEDLOOP_TLI5012B_H_ */
