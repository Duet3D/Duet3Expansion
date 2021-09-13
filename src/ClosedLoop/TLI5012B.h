/*
 * TLI5012B.h
 *
 *  Created on: 10 Jun 2020
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_TLI5012B_H_
#define SRC_CLOSEDLOOP_TLI5012B_H_

#include <ClosedLoop/AbsoluteEncoder.h>
#include <ClosedLoop/SpiEncoder.h>

#if SUPPORT_CLOSED_LOOP

#include <General/FreelistManager.h>

constexpr int16_t TLI5012B_READING_RANGE = 0x1 << 14;

// TODO: Fill out MAX parameter in MagneticEncoder below
class TLI5012B : public SpiEncoder, public AbsoluteEncoder<TLI5012B_READING_RANGE, 16>
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
	uint32_t GetAbsolutePosition(bool& error) noexcept;
	void AppendDiagnostics(const StringRef& reply) noexcept override;

private:
};

#endif

#endif /* SRC_CLOSEDLOOP_TLI5012B_H_ */
