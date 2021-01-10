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
#include <GCodes/GCodeResult.h>

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
	CanMessageBuffer *GetCanMove() noexcept;
	bool Send(CanMessageBuffer *buf) noexcept;
	bool SendAsync(CanMessageBuffer *buf) noexcept;
	bool SendAndFree(CanMessageBuffer *buf) noexcept;
	CanMessageBuffer *GetCanCommand() noexcept;
	uint16_t GetTimeStampCounter() noexcept;

	void SendAnnounce(CanMessageBuffer *buf) noexcept;

	void MoveStoppedByZProbe() noexcept;
	void WakeAsyncSenderFromIsr() noexcept;
}

#endif /* SRC_CAN_CANINTERFACE_H_ */
