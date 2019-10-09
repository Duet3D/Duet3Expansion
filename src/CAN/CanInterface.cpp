/*
 * CanInterface.cpp
 *
 *  Created on: 17 Sep 2018
 *      Author: David
 */

#include <CAN/CanInterface.h>
#include "CanMessageFormats.h"
#include "CanMessageBuffer.h"
#include "Platform.h"
#include "Movement/StepTimer.h"
#include "RTOSIface/RTOSIface.h"
#include "InputMonitors/InputMonitor.h"
#include "Movement/Move.h"

const unsigned int NumCanBuffers = 40;

static CanAddress boardAddress;
static bool enabled = false;

// CanReceiver management task
constexpr size_t CanReceiverTaskStackWords = 400;
static Task<CanReceiverTaskStackWords> canReceiverTask;

// Async sender task
constexpr size_t CanAsyncSenderTaskStackWords = 400;
static Task<CanAsyncSenderTaskStackWords> canAsyncSenderTask;

static TaskHandle sendingTaskHandle = nullptr;

static bool asyncSenderRunning = false;

class CanMessageQueue
{
public:
	CanMessageQueue();
	void AddMessage(CanMessageBuffer *buf);
	CanMessageBuffer *GetMessage();

private:
	CanMessageBuffer *pendingMessages;
	CanMessageBuffer *lastPendingMessage;			// only valid when pendingMessages != nullptr
};

CanMessageQueue::CanMessageQueue() : pendingMessages(nullptr) { }

void CanMessageQueue::AddMessage(CanMessageBuffer *buf)
{
	buf->next = nullptr;
	{
		TaskCriticalSectionLocker lock;

		if (pendingMessages == nullptr)
		{
			pendingMessages = lastPendingMessage = buf;
		}
		else
		{
			lastPendingMessage->next = buf;
		}
	}
}

// Fetch a message from the queue, or return nullptr if there are no messages
CanMessageBuffer *CanMessageQueue::GetMessage()
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

static CanMessageQueue PendingMoves;
static CanMessageQueue PendingCommands;

namespace CanInterface
{
	void ProcessReceivedMessage(CanMessageBuffer *buf);
}

extern "C" struct can_async_descriptor CAN_0;

extern "C" void CAN_0_tx_callback(struct can_async_descriptor *const descr)
{
	if (sendingTaskHandle != nullptr)
	{
		long higherPriorityTaskWoken;
		vTaskNotifyGiveFromISR(sendingTaskHandle, &higherPriorityTaskWoken);
		sendingTaskHandle = nullptr;
		portYIELD_FROM_ISR(higherPriorityTaskWoken);
	}
}

extern "C" void CAN_0_rx_callback(struct can_async_descriptor *const descr)
{
	canReceiverTask.GiveFromISR();
}

extern "C" [[noreturn]] void CanReceiverLoop(void *)
{
//	int32_t can_async_set_mode(struct can_async_descriptor *const descr, enum can_mode mode);

	// Set up CAN receiver filtering
	can_filter filter;

	// First a filter for our own ID
	filter.id = boardAddress << CanId::DstAddressShift;
	filter.mask = CanId::BoardAddressMask << CanId::DstAddressShift;
	can_async_set_filter(&CAN_0, 0, CAN_FMT_EXTID, &filter);

	// Now a filter for the broadcast ID
	filter.id = (uint32_t)CanId::BroadcastAddress << CanId::DstAddressShift;
	filter.mask = CanId::BoardAddressMask << CanId::DstAddressShift;
	can_async_set_filter(&CAN_0, 1, CAN_FMT_EXTID, &filter);

	can_async_enable(&CAN_0);
	CanMessageBuffer *buf = nullptr;
	for (;;)
	{
		// Get a buffer
		if (buf == nullptr)
		{
			for (;;)
			{
				buf = CanMessageBuffer::Allocate();
				if (buf != nullptr)
				{
					break;
				}
				delay(1);
			}
		}

		can_message msg;										// descriptor for the message
		msg.data = reinterpret_cast<uint8_t*>(&(buf->msg));		// set up where we want the message data to be stored
		const int32_t rslt = can_async_read(&CAN_0, &msg);	// fetch the message
		if (rslt == ERR_NOT_FOUND)
		{
			TaskBase::Take();											// wait until we are woken up because a message is available
		}
		else if (rslt == ERR_NONE)
		{
			if (enabled)
			{
				buf->dataLength = msg.len;
				buf->id.SetReceivedId(msg.id);
				CanInterface::ProcessReceivedMessage(buf);
				buf = nullptr;
			}
		}
		else
		{
			debugPrintf("CAN read err %d\n", (int)rslt);
		}
	}
}

extern "C" [[noreturn]] void CanAsyncSenderLoop(void *)
{
	asyncSenderRunning = true;
	uint32_t timeToWait = 0;
	CanMessageBuffer *buf;
	while ((buf = CanMessageBuffer::Allocate()) == nullptr)
	{
		delay(1);
	}

	for (;;)
	{
		// Set up a message ready
		auto msg = buf->SetupStatusMessage<CanMessageInputChanged>(CanInterface::GetCanAddress(), CanId::MasterAddress);
		msg->states = 0;
		msg->numHandles = 0;

		// Wait if necessary before looking for changed inputs
		if (timeToWait != 0)
		{
			TaskBase::Take(timeToWait);						// wait until we are woken up because a message is available
		}

		InputMonitor::AddStateChanges(msg, timeToWait);
		if (msg->numHandles != 0)
		{
			buf->dataLength = msg->GetActualDataLength();
			CanInterface::SendAsync(buf);					// this doesn't free the buffer, so we can re-use it
		}
	}
}

void CanInterface::Init(CanAddress pBoardAddress)
{
	boardAddress = pBoardAddress;
	CanMessageBuffer::Init(NumCanBuffers);

	can_async_register_callback(&CAN_0, CAN_ASYNC_RX_CB, (FUNC_PTR)CAN_0_rx_callback);
	can_async_register_callback(&CAN_0, CAN_ASYNC_TX_CB, (FUNC_PTR)CAN_0_tx_callback);

	enabled = true;

	// Create the task that receives CAN messages
	canReceiverTask.Create(CanReceiverLoop, "CanRecv", nullptr, TaskPriority::CanReceiverPriority);

	// Create the task that send endstop etc. updates
	canAsyncSenderTask.Create(CanAsyncSenderLoop, "CanAsync", nullptr, TaskPriority::CanAsyncSenderPriority);
}

// Shutdown is called when we are asked to update the firmware.
// We must allow the response to be sent, but we stop processing further messages.
void CanInterface::Shutdown()
{
	enabled = false;
}

CanAddress CanInterface::GetCanAddress()
{
	return boardAddress;
}

bool CanInterface::Send(CanMessageBuffer *buf)
{
	struct can_message msg;
	msg.id = buf->id.GetWholeId();
	msg.type = CAN_TYPE_DATA;
	msg.data = buf->msg.raw;
	msg.len = buf->dataLength;
	msg.fmt = CAN_FMT_EXTID;
	for (unsigned int tries = 0; tries < 5; ++tries)
	{
		{
			TaskCriticalSectionLocker lock;
			if (can_async_write(&CAN_0, &msg) == ERR_NONE)
			{
				return true;
			}
		}
		delay(2);
	}
	return false;
}

bool CanInterface::SendAsync(CanMessageBuffer *buf)
{
	//TODO use a dedicated buffer to send these high-priority messages
	return Send(buf);
}

bool CanInterface::SendAndFree(CanMessageBuffer *buf)
{
	const bool ok = Send(buf);
	CanMessageBuffer::Free(buf);
	return ok;
}

bool CanInterface::GetCanMove(CanMessageMovement& msg)
{
	// See if there is a movement message
	CanMessageBuffer * buf = PendingMoves.GetMessage();
	if (buf != nullptr)
	{
		msg = buf->msg.move;
		CanMessageBuffer::Free(buf);
		return true;
	}
	return false;
}

CanMessageBuffer *CanInterface::GetCanCommand()
{
	return PendingCommands.GetMessage();
}

void CanInterface::ProcessReceivedMessage(CanMessageBuffer *buf)
{
	switch (buf->id.MsgType())
	{
	case CanMessageType::timeSync:
		//TODO re-implement this as a PLL and use the CAN time stamps for greater accuracy
		StepTimer::SetLocalTimeOffset(StepTimer::GetInterruptClocks() - buf->msg.sync.timeSent);
		CanMessageBuffer::Free(buf);
		break;

	case CanMessageType::movement:
		//TODO if we haven't established time sync yet then we should defer this
// calling debugPrint here crashes the firmware even if we provide a large stack
//		buf->msg.move.DebugPrint();
		buf->msg.move.whenToExecute += StepTimer::GetLocalTimeOffset();
		PendingMoves.AddMessage(buf);
		break;

	case CanMessageType::stopMovement:
		moveInstance->StopDrivers(buf->msg.stopMovement.whichDrives);
		CanMessageBuffer::Free(buf);
		break;

	case CanMessageType::startup:
	case CanMessageType::controlledStop:
	case CanMessageType::emergencyStop:
		debugPrintf("Unsupported CAN message type %u\n", (unsigned int)(buf->id.MsgType()));
		CanMessageBuffer::Free(buf);
		break;

	default:
		if (buf->id.Dst() == GetCanAddress() && buf->id.IsRequest())
		{
			PendingCommands.AddMessage(buf);	// it's addressed to us, so queue it for processing
		}
		else
		{
			CanMessageBuffer::Free(buf);		// it's a broadcast message that we don't want, or a response, so throw it away
		}
		break;
	}
}

// This is called from the step ISR when the move is stopped by the Z probe
void CanInterface::MoveStoppedByZProbe()
{
	//TODO
}

void CanInterface::WakeAsyncSenderFromIsr()
{
	if (asyncSenderRunning)
	{
		canAsyncSenderTask.GiveFromISR();
	}
}

// End
