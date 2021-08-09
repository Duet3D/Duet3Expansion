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

#include <General/FreelistManager.h>

class TLI5012B : public SpiEncoder
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<TLI5012B>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<TLI5012B>(p); }

	TLI5012B(SharedSpiDevice& spiDev, Pin p_csPin) noexcept;
	~TLI5012B() { Disable(); }

	EncoderType GetType() const noexcept override { return EncoderType::TLI5012; }
	GCodeResult Init(const StringRef& reply) noexcept override;
	void Enable() noexcept override;
	void Disable() noexcept override;
	int32_t GetReading() noexcept override;
	void AppendDiagnostics(const StringRef& reply) noexcept override;

private:
};

#endif

#endif /* SRC_CLOSEDLOOP_TLI5012B_H_ */
