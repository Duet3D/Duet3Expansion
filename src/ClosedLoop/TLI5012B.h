/*
 * TLI5012B.h
 *
 *  Created on: 10 Jun 2020
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_TLI5012B_H_
#define SRC_CLOSEDLOOP_TLI5012B_H_

#include <ClosedLoop/SpiEncoder.h>

#if SUPPORT_CLOSED_LOOP

#include <ClosedLoop/SpiEncoder.h>
#include <Hardware/SharedSpiDevice.h>

class TLI5012B : public SpiEncoder
{
public:
	TLI5012B(Pin p_csPin) noexcept;

	void Enable() noexcept override;
	void Disable() noexcept override;
	int32_t GetReading() noexcept override;
	void Diagnostics(const StringRef& reply) noexcept override;

private:
};

#endif

#endif /* SRC_CLOSEDLOOP_TLI5012B_H_ */
