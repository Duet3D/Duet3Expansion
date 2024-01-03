/*
 * CanMessageQueue.cpp
 *
 *  Created on: 19 Jan 2021
 *      Author: David
 */

#include "CanMessageQueue.h"
#include <AppNotifyIndices.h>

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

		TaskBase * const waitingTask = taskWaitingToGet;
		if (waitingTask != nullptr)
		{
			taskWaitingToGet = nullptr;
			waitingTask->GiveFromISR(NotifyIndices::CanMessageQueue);
		}
	}
}

// Fetch a message from the queue, optionally waiting if necessary
CanMessageBuffer *CanMessageQueue::GetMessage(uint32_t timeout) noexcept
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

			if (timeout == 0)
			{
				return buf;
			}

			TaskBase::ClearCurrentTaskNotifyCount(NotifyIndices::CanMessageQueue);
			taskWaitingToGet = TaskBase::GetCallerTaskHandle();
		}

		if (!TaskBase::TakeIndexed(NotifyIndices::CanMessageQueue, timeout))
		{
			return nullptr;
		}
	}
}

// End
