/*
 * CanMessageQueue.cpp
 *
 *  Created on: 19 Jan 2021
 *      Author: David
 */

#include "CanMessageQueue.h"

CanMessageQueue::CanMessageQueue() noexcept : pendingMessages(nullptr), taskWaitingToGet(nullptr) { }

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

		TaskBase *waitingTask = taskWaitingToGet;
		if (waitingTask != nullptr)
		{
			taskWaitingToGet = nullptr;
			waitingTask->Give();
		}
	}
}

// Fetch a message from the queue, or return nullptr if there are no messages
CanMessageBuffer *CanMessageQueue::GetMessage() noexcept
{
	TaskCriticalSectionLocker lock;

	CanMessageBuffer * const buf = pendingMessages;
	if (buf != nullptr)
	{
		pendingMessages = buf->next;
	}
	return buf;
}

// Fetch a message from the queue, waiting if necessary
CanMessageBuffer *CanMessageQueue::BlockingGetMessage() noexcept
{
	while (true)
	{
		{
			TaskCriticalSectionLocker lock;

			CanMessageBuffer * const buf = pendingMessages;
			if (buf != nullptr)
			{
				pendingMessages = buf->next;
				return buf;
			}

			taskWaitingToGet = TaskBase::GetCallerTaskHandle();
		}

		TaskBase::Take();
	}
}

// End
