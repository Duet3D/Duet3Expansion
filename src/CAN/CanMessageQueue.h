/*
 * CanMessageQueue.h
 *
 *  Created on: 19 Jan 2021
 *      Author: David
 */

#ifndef SRC_CAN_CANMESSAGEQUEUE_H_
#define SRC_CAN_CANMESSAGEQUEUE_H_

#include "CanMessageBuffer.h"
#include <RTOSIface/RTOSIface.h>

class CanMessageQueue
{
public:
	CanMessageQueue() noexcept;
	void AddMessage(CanMessageBuffer *buf) noexcept;
	CanMessageBuffer *GetMessage(uint32_t timeout) noexcept;

private:
	CanMessageBuffer * volatile pendingMessages;
	CanMessageBuffer * volatile lastPendingMessage;		// only valid when pendingMessages != nullptr
	TaskBase * volatile taskWaitingToGet;
};

#endif /* SRC_CAN_CANMESSAGEQUEUE_H_ */
