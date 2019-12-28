/*
 * CanInterface.h
 *
 *  Created on: 17 Sep 2018
 *      Author: David
 */

#ifndef SRC_CAN_CANINTERFACE_H_
#define SRC_CAN_CANINTERFACE_H_

#include "RepRapFirmware.h"
#include <CanId.h>

struct CanMessageMovement;
class CanMessageBuffer;

namespace CanInterface
{
	void Init(CanAddress pBoardAddress);
	void Shutdown();
	void Diagnostics(const StringRef& reply);

	CanAddress GetCanAddress();
	bool GetCanMove(CanMessageMovement& move);
	bool Send(CanMessageBuffer *buf);
	bool SendAsync(CanMessageBuffer *buf);
	bool SendAndFree(CanMessageBuffer *buf);
	CanMessageBuffer *GetCanCommand();

	void MoveStoppedByZProbe();
	void WakeAsyncSenderFromIsr();
}

#endif /* SRC_CAN_CANINTERFACE_H_ */
