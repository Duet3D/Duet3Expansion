/*
 * AS5047D.h
 *
 *  Created on: 10 Jun 2020
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_AS5047D_H_
#define SRC_CLOSEDLOOP_AS5047D_H_

#include <ClosedLoop/AbsoluteEncoder.h>
#include <ClosedLoop/SpiEncoder.h>

#if SUPPORT_CLOSED_LOOP

#include <General/FreelistManager.h>

constexpr int16_t AS5047D_READING_RANGE = 0x1 << 14;
constexpr int16_t AS5047D_ABS_READING_OFFSET = 0x1 << 13;

class AS5047D : public SpiEncoder, public AbsoluteEncoder<AS5047D_READING_RANGE, 16>
{
public:
	void* operator new(size_t sz) noexcept { return FreelistManager::Allocate<AS5047D>(); }
	void operator delete(void* p) noexcept { FreelistManager::Release<AS5047D>(p); }

	AS5047D(SharedSpiDevice& spiDev, Pin p_csPin) noexcept;
	~AS5047D() { AS5047D::Disable(); }

	EncoderType GetType() const noexcept override { return EncoderType::AS5047; }
	GCodeResult Init(const StringRef& reply) noexcept override;
	void Enable() noexcept override;
	void Disable() noexcept override;
	uint32_t GetAbsolutePosition(bool& error) noexcept;
	void AppendDiagnostics(const StringRef& reply) noexcept override;
	void AppendStatus(const StringRef& reply) noexcept override;

private:
	struct DiagnosticRegisters
	{
		uint16_t diag;
		uint16_t mag;
		uint16_t errFlags;
	};

	bool DoSpiTransaction(uint16_t command, uint16_t& response) noexcept;
	bool GetDiagnosticRegisters(DiagnosticRegisters& regs) noexcept;
};

#endif

#endif /* SRC_CLOSEDLOOP_AS5047D_H_ */
