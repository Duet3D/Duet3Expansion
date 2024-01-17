/*
 * CanInterface.h
 *
 *  Created on: 17 Sep 2018
 *      Author: David
 */

#ifndef SRC_CAN_CANINTERFACE_H_
#define SRC_CAN_CANINTERFACE_H_

#include <RepRapFirmware.h>
#include <CanId.h>
#include <CanMessageFormats.h>

struct CanMessageMovement;
class CanMessageBuffer;

namespace CanInterface
{
	void Init(CanAddress defaultBoardAddress, bool useAlternatePins, bool full) noexcept;
	void Shutdown() noexcept;
	void Diagnostics(const StringRef& reply) noexcept;

	CanAddress GetCanAddress() noexcept;
	CanAddress GetCurrentMasterAddress() noexcept;
	GCodeResult ChangeAddressAndDataRate(const CanMessageSetAddressAndNormalTiming& msg, const StringRef& reply) noexcept;
	bool GetCanMessage(CanMessageBuffer *buf) noexcept;
	CanMessageBuffer *GetCanMove(uint32_t timeout) noexcept;
	bool Send(CanMessageBuffer *buf) noexcept;
	bool SendAsync(CanMessageBuffer *buf) noexcept;
	bool SendAndFree(CanMessageBuffer *buf) noexcept;
	CanMessageBuffer *GetCanCommand(uint32_t timeout) noexcept;

#if !USE_SERIAL_DEBUG
	bool DebugPutc(char c) noexcept;
#endif

#if !SAME70 && !RP2040
	uint16_t GetTimeStampCounter() noexcept;
	uint16_t GetTimeStampPeriod() noexcept;
#endif

	bool SendAnnounce(CanMessageBuffer *buf) noexcept;
	void RaiseEvent(EventType type, uint16_t param, uint8_t device, const char *format, va_list vargs) noexcept;

	void WakeAsyncSender() noexcept;
}

#endif /* SRC_CAN_CANINTERFACE_H_ */
