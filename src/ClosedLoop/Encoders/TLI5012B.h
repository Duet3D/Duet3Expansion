/*
 * TLI5012B.h
 *
 *  Created on: 10 Jun 2020
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_TLI5012B_H_
#define SRC_CLOSEDLOOP_TLI5012B_H_

#include "AbsoluteRotaryEncoder.h"
#include "SpiEncoder.h"

#if SUPPORT_CLOSED_LOOP

#include <General/FreelistManager.h>

// TODO: Fill out MAX parameter in MagneticEncoder below
class TLI5012B : public SpiEncoder, public AbsoluteRotaryEncoder
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<TLI5012B>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<TLI5012B>(p); }

	TLI5012B(uint32_t p_stepsPerRev, SharedSpiDevice& spiDev, Pin p_csPin) noexcept;
	~TLI5012B() { TLI5012B::Disable(); }

	EncoderType GetType() const noexcept override { return EncoderType::rotaryTLI5012; }
	GCodeResult Init(const StringRef& reply) noexcept override;
	void Enable() noexcept override;
	void Disable() noexcept override;
	void AppendDiagnostics(const StringRef& reply) noexcept override;
	void AppendStatus(const StringRef& reply) noexcept override;

protected:
	bool GetRawReading() noexcept override;

private:
};

#endif

#endif /* SRC_CLOSEDLOOP_TLI5012B_H_ */
