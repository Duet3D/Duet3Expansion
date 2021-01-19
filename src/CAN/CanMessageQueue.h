/*
 * CanMessageQueue.h
 *
 *  Created on: 19 Jan 2021
 *      Author: David
 */

#ifndef SRC_CAN_CANMESSAGEQUEUE_H_
#define SRC_CAN_CANMESSAGEQUEUE_H_

#include <CanMessageBuffer.h>

class CanMessageQueue
{
public:
	CanMessageQueue() noexcept;
	void AddMessage(CanMessageBuffer *buf) noexcept;
	CanMessageBuffer *GetMessage() noexcept;

private:
	CanMessageBuffer * volatile pendingMessages;
	CanMessageBuffer * volatile lastPendingMessage;		// only valid when pendingMessages != nullptr
};

#endif /* SRC_CAN_CANMESSAGEQUEUE_H_ */
