/*
 * AS5047D.h
 *
 *  Created on: 10 Jun 2020
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_AS5047D_H_
#define SRC_CLOSEDLOOP_AS5047D_H_

#include <ClosedLoop/SpiEncoder.h>

#if SUPPORT_CLOSED_LOOP

class AS5047D : public SpiEncoder
{
public:
	AS5047D(Pin p_csPin) noexcept;

	void Enable() noexcept override;
	void Disable() noexcept override;
	int32_t GetReading() noexcept override;
	void Diagnostics(const StringRef& reply) noexcept override;

private:
	bool DoSpiTransaction(uint16_t command, uint16_t& response) noexcept;
};

#endif

#endif /* SRC_CLOSEDLOOP_AS5047D_H_ */
