/*
 * AS5047D.h
 *
 *  Created on: 10 Jun 2020
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_AS5047D_H_
#define SRC_CLOSEDLOOP_AS5047D_H_

#include "AngleSensor.h"

#if SUPPORT_CLOSED_LOOP

#include <Hardware/SharedSpiDevice.h>

class AS5047D : public AngleSensor
{
public:
	AS5047D(SharedSpiDevice& p_spi, Pin p_csPin);

	void Init() override;
	int16_t GetAngle() override;
	void Diagnostics(const StringRef& reply) override;

private:
	bool DoSpiTransaction(uint16_t command, uint16_t& response);

	SharedSpiDevice& spi;
	Pin csPin;
};

#endif

#endif /* SRC_CLOSEDLOOP_AS5047D_H_ */
