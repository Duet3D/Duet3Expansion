/*
 * CanMessageQueue.cpp
 *
 *  Created on: 19 Jan 2021
 *      Author: David
 */

#include "CanMessageQueue.h"
#include <RTOSIface/RTOSIface.h>

CanMessageQueue::CanMessageQueue() noexcept : pendingMessages(nullptr) { }

void CanMessageQueue::AddMessage(CanMessageBuffer *buf) noexcept
{
	buf->next = nullptr;
	{
		TaskCriticalSectionLocker lock;

		if (pendingMessages == nullptr)
		{
			pendingMessages = buf;
		}
		else
		{
			lastPendingMessage->next = buf;
		}
		lastPendingMessage = buf;
	}
}

// Fetch a message from the queue, or return nullptr if there are no messages
CanMessageBuffer *CanMessageQueue::GetMessage() noexcept
{
	CanMessageBuffer *buf;
	{
		TaskCriticalSectionLocker lock;

		buf = pendingMessages;
		if (buf != nullptr)
		{
			pendingMessages = buf->next;
		}
	}
	return buf;
}

// End
