/*
 * Can.cpp
 *
 *  Created on: 17 Sep 2018
 *      Author: David
 */

#include "CanSlaveInterface.h"
#include "CanMessageFormats.h"
#include "CanMessageBuffer.h"
#include "Platform.h"
#include "Movement/StepTimer.h"
#include "RTOSIface/RTOSIface.h"

const unsigned int NumCanBuffers = 40;

// CanReceiver management task
constexpr size_t CanReceiverTaskStackWords = 400;		// need quite a large stack to allow for calls to debugPrint, 400 is not enough
static Task<CanReceiverTaskStackWords> canReceiverTask;

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

namespace CanSlaveInterface
{
	void ProcessReceivedMessage(CanMessageBuffer *buf);
}

extern "C" struct can_async_descriptor CAN_0;

extern "C" void CAN_0_tx_callback(struct can_async_descriptor *const descr)
{
	(void)descr;
}

extern "C" void CAN_0_rx_callback(struct can_async_descriptor *const descr)
{
	canReceiverTask.GiveFromISR();
}

CanAddress GetCanAddress()
{
	//TODO read this just once
	return Platform::ReadBoardId();
}

#if 0
/**
 * Example of using CAN_0 to Encrypt/Decrypt data.
 */
void CAN_0_example(void)
{
	struct can_message msg;
	uint8_t send_data[4];
	send_data[0] = 0x00;
	send_data[1] = 0x01;
	send_data[2] = 0x02;
	send_data[3] = 0x03;
	msg.id = 0x45A;
	msg.type = CAN_TYPE_DATA;
	msg.data = send_data;
	msg.len = 4;
	msg.fmt = CAN_FMT_STDID;
	can_async_register_callback(&CAN_0, CAN_ASYNC_TX_CB, (FUNC_PTR)CAN_0_tx_callback);
	can_async_enable(&CAN_0);
	/**
	* CAN_0_tx_callback callback should be invoked after call
	* can_async_write, and remote device should receive message with ID=0
	x45A
	*/
	can_async_write(&CAN_0, &msg);
	msg.id = 0x100000A5;
	msg.fmt = CAN_FMT_EXTID;
	/**
	* remote device should receive message with ID=0x100000A5
	*/
	can_async_write(&CAN_0, &msg);
	/**
	* CAN_0_rx_callback callback should be invoked after call
	* can_async_set_filter and remote device send CAN Message with the same
	* content as the filter.
	*/
	can_filter filter;
	can_async_register_callback(&CAN_0, CAN_ASYNC_RX_CB, (FUNC_PTR)CAN_0_rx_callback);
	filter.id = 0x469;
	filter.mask = 0;
	can_async_set_filter(&CAN_0, 0, CAN_FMT_STDID, &filter);
	filter.id = 0x10000096;
	filter.mask = 0;
	can_async_set_filter(&CAN_0, 1, CAN_FMT_EXTID, &filter);
}
#endif

extern "C" void CanReceiverLoop(void *)
{
//	int32_t can_async_set_mode(struct can_async_descriptor *const descr, enum can_mode mode);

	can_async_register_callback(&CAN_0, CAN_ASYNC_RX_CB, (FUNC_PTR)CAN_0_rx_callback);

	// Set up CAN receiver filtering
	can_filter filter;

	// First a filter for our own ID
	filter.id = (uint32_t)Platform::ReadBoardId() << CanId::DstAddressShift;
	filter.mask = CanId::BoardAddressMask << CanId::DstAddressShift;
	can_async_set_filter(&CAN_0, 0, CAN_FMT_EXTID, &filter);

	// Now a filter for the broadcast ID
	filter.id = (uint32_t)CanId::BroadcastAddress << CanId::DstAddressShift;
	filter.mask = CanId::BoardAddressMask << CanId::DstAddressShift;
	can_async_set_filter(&CAN_0, 1, CAN_FMT_EXTID, &filter);

	can_async_enable(&CAN_0);

	for (;;)
	{
		TaskBase::Take();											// wait until we are woken up because a message is available
		CanMessageBuffer *buf = CanMessageBuffer::Allocate();		// allocate a buffer to receive the message
		if (buf != nullptr)
		{
			can_message msg;										// descriptor for the message
			msg.data = reinterpret_cast<uint8_t*>(&(buf->msg));		// set up where we want the message data to be stored
			const int32_t rslt = can_async_read(&CAN_0, &msg);		// fetch the message
			if (rslt == ERR_NONE)
			{
				buf->dataLength = msg.len;
				buf->id.SetReceivedId(msg.id);
				CanSlaveInterface::ProcessReceivedMessage(buf);
			}
			else
			{
				debugPrintf("CAN read err %d\n", (int)rslt);
				CanMessageBuffer::Free(buf);
			}
		}
	}
}

void CanSlaveInterface::Init()
{
	CanMessageBuffer::Init(NumCanBuffers);

	// Create the task that sends CAN messages
	canReceiverTask.Create(CanReceiverLoop, "CanSender", nullptr, TaskPriority::CanReceiverPriority);
}

bool CanSlaveInterface::GetCanMove(CanMessageMovement& msg)
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

CanMessageBuffer *CanSlaveInterface::GetCanCommand()
{
	return PendingCommands.GetMessage();
}

void CanSlaveInterface::ProcessReceivedMessage(CanMessageBuffer *buf)
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

	case CanMessageType::m307:
	case CanMessageType::m308:
	case CanMessageType::m906:
	case CanMessageType::m950:
		PendingCommands.AddMessage(buf);
		break;

	case CanMessageType::startup:
	case CanMessageType::controlledStop:
	case CanMessageType::emergencyStop:
		debugPrintf("Unsupported CAN message type %u\n", (unsigned int)(buf->id.MsgType()));
		CanMessageBuffer::Free(buf);
		break;

	default:
		debugPrintf("Unknown CAN message type %u\n", (unsigned int)(buf->id.MsgType()));
		CanMessageBuffer::Free(buf);
		break;
	}
}

// This is called from the step ISR when the move is stopped by the Z probe
void CanSlaveInterface::MoveStoppedByZProbe()
{
	//TODO
}

// End
