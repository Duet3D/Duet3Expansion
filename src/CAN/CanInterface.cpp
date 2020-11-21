/*
 * CanInterface.cpp
 *
 *  Created on: 17 Sep 2018
 *      Author: David
 */

#include "CanInterface.h"

#include <CanSettings.h>
#include <CanMessageFormats.h>
#include <CanMessageBuffer.h>
#include <Platform.h>
#include <TaskPriorities.h>
#include <Movement/StepTimer.h>
#include <RTOSIface/RTOSIface.h>
#include <InputMonitors/InputMonitor.h>
#include <Movement/Move.h>
#include <General/SafeVsnprintf.h>

#define SUPPORT_CAN		1				// needed by CanDriver.h
#include <CanDevice.h>
#include <Hardware/IoPorts.h>
#include <Version.h>
#include <hpl_user_area.h>

#if SAME5x
constexpr uint32_t CanUserAreaDataOffset = 512 - sizeof(CanUserAreaData);
#elif SAMC21
constexpr uint32_t CanUserAreaDataOffset = 256 - sizeof(CanUserAreaData);
#endif

constexpr unsigned int NumCanBuffers = 40;

static CanDevice *can0dev = nullptr;
static CanUserAreaData canConfigData;
static CanAddress boardAddress;
static bool enabled = false;

constexpr CanDevice::Config Can0Config =
{
	.dataSize = 64,									// must be one of: 8, 12, 16, 20, 24, 32, 48, 64
	.numTxBuffers = 2,
	.txFifoSize = 10,								// enough to send a 512-byte response broken into 60-byte fragments
	.numRxBuffers = 0,
	.rxFifo0Size = 16,
	.rxFifo1Size = 16,
	.numShortFilterElements = 0,
	.numExtendedFilterElements = 3,
	.txEventFifoSize = 2
};

static_assert(Can0Config.IsValid());

// CAN buffer memory must be in the first 64Kb of RAM (SAME5x) or in non-cached RAM (SAME70), so put it in its own segment
static uint32_t can0Memory[Can0Config.GetMemorySize()] __attribute__ ((section (".CanMessage")));

// CanReceiver management task
constexpr size_t CanReceiverTaskStackWords = 120;
static Task<CanReceiverTaskStackWords> canReceiverTask;

// Async sender task
constexpr size_t CanAsyncSenderTaskStackWords = 120;
static Task<CanAsyncSenderTaskStackWords> canAsyncSenderTask;

static bool mainBoardAcknowledgedAnnounce = false;	// true after the main board has acknowledged our announcement
static bool isProgrammed = false;					// true after the main board has sent us any configuration commands

#if SUPPORT_DRIVERS
static uint32_t lastMotionMessageScheduledTime = 0;
static uint32_t lastMotionMessageReceivedAt = 0;
#endif

static unsigned int duplicateMotionMessages = 0;
static unsigned int oosMessages = 0;
uint8_t expectedSeq = 0xFF;

//DEBUG
//static int32_t accumulatedMotion = 0;

class CanMessageQueue
{
public:
	CanMessageQueue();
	void AddMessage(CanMessageBuffer *buf);
	CanMessageBuffer *GetMessage();

private:
	CanMessageBuffer * volatile pendingMessages;
	CanMessageBuffer * volatile lastPendingMessage;		// only valid when pendingMessages != nullptr
};

CanMessageQueue::CanMessageQueue() : pendingMessages(nullptr) { }

static CanMessageQueue PendingMoves;
static CanMessageQueue PendingCommands;

void CanMessageQueue::AddMessage(CanMessageBuffer *buf)
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

extern "C" [[noreturn]] void CanReceiverLoop(void *);
extern "C" [[noreturn]] void CanAsyncSenderLoop(void *);

namespace CanInterface
{
	void ProcessReceivedMessage(CanMessageBuffer *buf);
}

// Initialise this module and the CAN hardware
void CanInterface::Init(CanAddress defaultBoardAddress, bool useAlternatePins, bool full)
{
	// Read the CAN timing data from the top part of the NVM User Row
	canConfigData = *reinterpret_cast<CanUserAreaData*>(NVMCTRL_USER + CanUserAreaDataOffset);

	CanTiming timing;
	canConfigData.GetTiming(timing);

	// Set up the CAN pins
#if SAME5x
	// We don't support alternate pins for the SAME5x yet
	SetPinFunction(PortBPin(13), GpioPinFunction::H);
	SetPinFunction(PortBPin(12), GpioPinFunction::H);
	const unsigned int whichPort = 1;							// we use CAN1 on the SAME5x
#elif SAMC21
	if (useAlternatePins)
	{
		SetPinFunction(PortBPin(23), GpioPinFunction::G);
		SetPinFunction(PortBPin(22), GpioPinFunction::G);
	}
	else
	{
		SetPinFunction(PortAPin(25), GpioPinFunction::G);
		SetPinFunction(PortAPin(24), GpioPinFunction::G);
	}
	const unsigned int whichPort = 0;							// we use CAN0 on the SAMC21
#endif

	// Initialise the CAN hardware, using the timing data if it was valid
	can0dev = CanDevice::Init(0, whichPort, Can0Config, can0Memory, timing);

#ifdef SAMMYC21
	pinMode(CanStandbyPin, OUTPUT_LOW);							// take the CAN drivers out of standby
#endif

	boardAddress = canConfigData.GetCanAddress(defaultBoardAddress);

	// Set up CAN receiver filtering
	// Set up a CAN receive filter to receive all messages addressed to us in FIFO 0
	can0dev->SetExtendedFilterElement(0, CanDevice::RxBufferNumber::fifo0,
										(uint32_t)boardAddress << CanId::DstAddressShift,
										CanId::BoardAddressMask << CanId::DstAddressShift);

	if (full)
	{
		// Now a filter for the broadcast ID
		can0dev->SetExtendedFilterElement(1, CanDevice::RxBufferNumber::fifo0,
											(uint32_t)CanId::BroadcastAddress << CanId::DstAddressShift,
											CanId::BoardAddressMask << CanId::DstAddressShift);
	}
	can0dev->Enable();

	enabled = true;

	if (full)
	{
		CanMessageBuffer::Init(NumCanBuffers);

		// Create the task that receives CAN messages
		canReceiverTask.Create(CanReceiverLoop, "CanRecv", nullptr, TaskPriority::CanReceiverPriority);

		// Create the task that send endstop etc. updates
		canAsyncSenderTask.Create(CanAsyncSenderLoop, "CanAsync", nullptr, TaskPriority::CanAsyncSenderPriority);
	}
}

// Shutdown is called when we are asked to update the firmware.
// We must allow the response to be sent, but we stop processing further messages.
void CanInterface::Shutdown()
{
	enabled = false;
	if (can0dev != nullptr)
	{
		can0dev->DeInit();
	}

	// It's safe to terminate the tasks even if they haven't been created
	canReceiverTask.TerminateAndUnlink();
	canAsyncSenderTask.TerminateAndUnlink();
}

CanAddress CanInterface::GetCanAddress()
{
	return boardAddress;
}

// Send a message. On return the buffer is available to the caller to re-use or free.
// Any extra bytes needed as padding are set to zero by the CAN driver.
bool CanInterface::Send(CanMessageBuffer *buf)
{
	//TODO option to not force sending, and return true only if successful?
	can0dev->SendMessage(CanDevice::TxBufferNumber::fifo, 1000, buf);
	return true;

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

// Return a move message, if there is one. Caller must free the message buffer.
CanMessageBuffer * CanInterface::GetCanMove()
{
	return PendingMoves.GetMessage();
}

CanMessageBuffer *CanInterface::GetCanCommand()
{
	return PendingCommands.GetMessage();
}

// Process a received message and (eventually) release the buffer that it arrived in
void CanInterface::ProcessReceivedMessage(CanMessageBuffer *buf)
{
	switch (buf->id.MsgType())
	{
	case CanMessageType::timeSync:
		//TODO re-implement this as a PLL and use the CAN time stamps for greater accuracy
		StepTimer::SetLocalTimeOffset(StepTimer::GetTimerTicks() - buf->msg.sync.timeSent);
		Platform::SetPrinting(buf->msg.sync.isPrinting);

		if (buf->dataLength >= 16)				// if real time is included
		{
			Platform::SetDateTime(buf->msg.sync.realTime);
		}
		CanMessageBuffer::Free(buf);
		break;

#if SUPPORT_DRIVERS
	case CanMessageType::movement:
		// Check for duplicate and out-of-sequence message
		{
			const uint32_t now = millis();
			if (buf->msg.move.whenToExecute == lastMotionMessageScheduledTime && now - lastMotionMessageReceivedAt < 100)
			{
				++duplicateMotionMessages;
				CanMessageBuffer::Free(buf);
				break;
			}
			lastMotionMessageScheduledTime = buf->msg.move.whenToExecute;
			lastMotionMessageReceivedAt = now;

			const int8_t seq = buf->msg.move.seq;
			if (seq != expectedSeq && expectedSeq != 0xFF)
			{
				++oosMessages;
			}
			expectedSeq = (seq + 1) & 7;
		}

		//TODO if we haven't established time sync yet then we should defer this
		buf->msg.move.whenToExecute += StepTimer::GetLocalTimeOffset();
		//DEBUG
		//accumulatedMotion +=buf->msg.move.perDrive[0].steps;
		//END
		PendingMoves.AddMessage(buf);
		Platform::OnProcessingCanMessage();
		break;

	case CanMessageType::stopMovement:
		moveInstance->StopDrivers(buf->msg.stopMovement.whichDrives);
		CanMessageBuffer::Free(buf);
		Platform::OnProcessingCanMessage();
		break;
#endif

	case CanMessageType::emergencyStop:
		Platform::EmergencyStop();
		CanMessageBuffer::Free(buf);
		break;

	case CanMessageType::acknowledgeAnnounce:
		if (buf->id.Src() == CanId::MasterAddress)
		{
			mainBoardAcknowledgedAnnounce = true;
			Platform::OnProcessingCanMessage();
		}
		CanMessageBuffer::Free(buf);
		break;

	case CanMessageType::startup:
		if (millis() > 1000 || isProgrammed)		// if we've only just powered up and the main board hasn't programmed us yet, no need to start up again
		{
			Platform::EmergencyStop();
		}
		CanMessageBuffer::Free(buf);
		Platform::OnProcessingCanMessage();
		break;

	case CanMessageType::controlledStop:
		debugPrintf("Unsupported CAN message type %u\n", (unsigned int)(buf->id.MsgType()));
		CanMessageBuffer::Free(buf);
		Platform::OnProcessingCanMessage();
		break;

	default:
		if (buf->id.Dst() == GetCanAddress() && buf->id.IsRequest())
		{
			if (buf->id.Src() == CanId::MasterAddress)
			{
				isProgrammed = true;			// record that we've had a communication from the master since we started up
			}
			PendingCommands.AddMessage(buf);	// it's addressed to us, so queue it for processing
		}
		else
		{
			CanMessageBuffer::Free(buf);		// it's a broadcast message that we don't want, or a response, so throw it away
		}
		break;
	}
}

void CanInterface::Diagnostics(const StringRef& reply)
{
	unsigned int messagesQueuedForSending, messagesReceived, txTimeouts, messagesLost, busOffCount;
	can0dev->GetAndClearStats(messagesQueuedForSending, messagesReceived, txTimeouts, messagesLost, busOffCount);
	reply.lcatf("CAN messages queued %u, send timeouts %u, received %u, lost %u, free buffers %u\n",
					messagesQueuedForSending, txTimeouts, messagesReceived, messagesLost, CanMessageBuffer::FreeBuffers());
}

// Send an announcement message if we haven't had an announce acknowledgement form the main board. On return the buffer is available to use again.
void CanInterface::SendAnnounce(CanMessageBuffer *buf)
{
	if (!mainBoardAcknowledgedAnnounce)
	{
		auto msg = buf->SetupRequestMessage<CanMessageAnnounce>(0, boardAddress, CanId::MasterAddress);
		msg->timeSinceStarted = millis();
		msg->numDrivers = NumDrivers;
		msg->zero = 0;
		SafeSnprintf(msg->boardTypeAndFirmwareVersion, ARRAY_SIZE(msg->boardTypeAndFirmwareVersion), "%s|%s (%s)", BOARD_TYPE_NAME, VERSION, IsoDate);
		buf->dataLength = msg->GetActualDataLength();
		Send(buf);
	}
}

// This is called from the step ISR when the move is stopped by the Z probe
void CanInterface::MoveStoppedByZProbe()
{
	//TODO
}

void CanInterface::WakeAsyncSenderFromIsr()
{
	canAsyncSenderTask.GiveFromISR();
}

GCodeResult CanInterface::ChangeAddressAndDataRate(const CanMessageSetAddressAndNormalTiming &msg, const StringRef &reply)
{
	if (msg.oldAddress == boardAddress)
	{
		bool seen = false;

		// Check whether we are setting the address
		if (msg.newAddress != 0 && msg.newAddress <= CanId::MaxCanAddress && msg.newAddress == (uint8_t)~msg.newAddressInverted)
		{
			seen = true;
			canConfigData.SetCanAddress(msg.newAddress);
		}

		// Check whether we are changing the timing
		if (msg.doSetTiming == CanMessageSetAddressAndNormalTiming::DoSetTimingYes)
		{
			seen = true;
			canConfigData.SetTiming(msg.normalTiming);
		}

		if (seen)
		{
			const int32_t rc = _user_area_write(reinterpret_cast<void*>(NVMCTRL_USER), CanUserAreaDataOffset, reinterpret_cast<const uint8_t*>(&canConfigData), sizeof(canConfigData));
			if (rc != 0)
			{
				reply.printf("Failed to write NVM user area, code %" PRIi32, rc);
				return GCodeResult::error;
			}
		}
		else
		{
			CanTiming timing;
			can0dev->GetLocalCanTiming(timing);
			reply.printf("CAN bus speed %.1fkbps, tseg1 %.2f, jump width %.2f",
							(double)((float)CanTiming::ClockFrequency/(1000 * timing.period)),
							(double)((float)timing.tseg1/(float)timing.period),
							(double)((float)timing.jumpWidth/(float)timing.period));
		}
		return GCodeResult::ok;
	}

	reply.copy("Received ChangeAddress message for wrong board");
	return GCodeResult::error;
}

// Get a message, if there is one
bool CanInterface::GetCanMessage(CanMessageBuffer *buf)
{
	return can0dev->ReceiveMessage(CanDevice::RxBufferNumber::fifo0, 0, buf);
}

extern "C" [[noreturn]] void CanReceiverLoop(void *)
{
	CanMessageBuffer *buf = nullptr;
	for (;;)
	{
		if (!enabled)
		{
			delay(10);
		}
		else
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

			if (can0dev->ReceiveMessage(CanDevice::RxBufferNumber::fifo0, TaskBase::TimeoutUnlimited, buf))
			{
				CanInterface::ProcessReceivedMessage(buf);
				buf = nullptr;
			}
			else
			{
				debugPrintf("CAN read err\n");
			}
		}
	}
}

extern "C" [[noreturn]] void CanAsyncSenderLoop(void *)
{
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
		msg->zero = 0;
		msg->numHandles = 0;

		const uint32_t timeToWait = InputMonitor::AddStateChanges(msg);
		if (msg->numHandles != 0)
		{
			buf->dataLength = msg->GetActualDataLength();
			CanInterface::SendAsync(buf);					// this doesn't free the buffer, so we can re-use it
		}
		TaskBase::Take(timeToWait);						// wait until we are woken up because a message is available, or we time out
	}
}

// End
