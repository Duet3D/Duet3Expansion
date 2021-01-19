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
static CanAddress currentMasterAddress =
#if defined(ATECM) || defined(ATEIO)
										CanId::ATEMasterAddress;
#else
										CanId::MasterAddress;
#endif

static bool enabled = false;

constexpr CanDevice::Config Can0Config =
{
	.dataSize = 64,									// must be one of: 8, 12, 16, 20, 24, 32, 48, 64
	.numTxBuffers = 2,
	.txFifoSize = 10,								// enough to send a 512-byte response broken into 60-byte fragments
	.numRxBuffers = 1,								// we use a dedicated buffer for the clock sync messages
	.rxFifo0Size = 16,
	.rxFifo1Size = 16,
	.numShortFilterElements = 0,
	.numExtendedFilterElements = 3,
	.txEventFifoSize = 2
};

static_assert(Can0Config.IsValid());

// CAN buffer memory must be in the first 64Kb of RAM (SAME5x) or in non-cached RAM (SAME70), so put it in its own segment
static uint32_t can0Memory[Can0Config.GetMemorySize()] __attribute__ ((section (".CanMessage")));

// CanClock task
constexpr size_t CanClockTaskStackWords = 130;
static Task<CanClockTaskStackWords> canClockTask;

// CanReceiver management task
constexpr size_t CanReceiverTaskStackWords = 120;
static Task<CanReceiverTaskStackWords> canReceiverTask;

// Async sender task
constexpr size_t CanAsyncSenderTaskStackWords = 100;
static Task<CanAsyncSenderTaskStackWords> canAsyncSenderTask;

static bool mainBoardAcknowledgedAnnounce = false;	// true after the main board has acknowledged our announcement
static bool isProgrammed = false;					// true after the main board has sent us any configuration commands

#if SUPPORT_DRIVERS
static uint32_t lastMotionMessageScheduledTime = 0;
static uint32_t lastMotionMessageReceivedAt = 0;
static unsigned int duplicateMotionMessages = 0;
static unsigned int oosMessages = 0;
#endif

uint8_t expectedSeq = 0xFF;

//DEBUG
//static int32_t accumulatedMotion = 0;

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

CanMessageQueue::CanMessageQueue() noexcept : pendingMessages(nullptr) { }

static CanMessageQueue PendingMoves;
static CanMessageQueue PendingCommands;

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

extern "C" [[noreturn]] void CanClockLoop(void *) noexcept;
extern "C" [[noreturn]] void CanReceiverLoop(void *) noexcept;
extern "C" [[noreturn]] void CanAsyncSenderLoop(void *) noexcept;

namespace CanInterface
{
	CanMessageBuffer *ProcessReceivedMessage(CanMessageBuffer *buf) noexcept;
}

// Initialise this module and the CAN hardware
void CanInterface::Init(CanAddress defaultBoardAddress, bool useAlternatePins, bool full) noexcept
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
	can0dev = CanDevice::Init(0, whichPort, Can0Config, can0Memory, timing, nullptr);

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
		// Set up a CAN receive filter to receive clock sync messages in buffer 0
		can0dev->SetExtendedFilterElement(1, CanDevice::RxBufferNumber::buffer0,
											((uint32_t)CanMessageType::timeSync << CanId::MessageTypeShift) | ((uint32_t)CanId::BroadcastAddress << CanId::DstAddressShift),
											1);					// mask is unused when using a dedicated Rx buffer, but must be nonzero to enable the element

		// Set up a filter for all other broadcast messages in FIFO 0
		can0dev->SetExtendedFilterElement(2, CanDevice::RxBufferNumber::fifo0,
											(uint32_t)CanId::BroadcastAddress << CanId::DstAddressShift,
											CanId::BoardAddressMask << CanId::DstAddressShift);
	}

	// For receiving into a dedicated buffer, the mask is ignored and only the extended ID mask is applied. We need to ignore the source address.
	can0dev->SetExtendedIdMask(0x1FFFFFFF & ~(CanId::BoardAddressMask << CanId::SrcAddressShift));
	can0dev->Enable();

	enabled = true;

	if (full)
	{
		CanMessageBuffer::Init(NumCanBuffers);

		// Create the clock sync
		canClockTask.Create(CanClockLoop, "CanClock", nullptr, TaskPriority::CanClockPriority);

		// Create the task that receives CAN messages
		canReceiverTask.Create(CanReceiverLoop, "CanRecv", nullptr, TaskPriority::CanReceiverPriority);

		// Create the task that send endstop etc. updates
		canAsyncSenderTask.Create(CanAsyncSenderLoop, "CanAsync", nullptr, TaskPriority::CanAsyncSenderPriority);
	}
}

// Shutdown is called when we are asked to update the firmware.
// We must allow the response to be sent, but we stop processing further messages.
void CanInterface::Shutdown() noexcept
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

CanAddress CanInterface::GetCanAddress() noexcept
{
	return boardAddress;
}

CanAddress CanInterface::GetCurrentMasterAddress() noexcept
{
	return currentMasterAddress;
}

// Send a message. On return the buffer is available to the caller to re-use or free.
// Any extra bytes needed as padding are set to zero by the CAN driver.
bool CanInterface::Send(CanMessageBuffer *buf) noexcept
{
	//TODO option to not force sending, and return true only if successful?
	can0dev->SendMessage(CanDevice::TxBufferNumber::fifo, 1000, buf);
	return true;

}

bool CanInterface::SendAsync(CanMessageBuffer *buf) noexcept
{
	//TODO use a dedicated buffer to send these high-priority messages
	return Send(buf);
}

bool CanInterface::SendAndFree(CanMessageBuffer *buf) noexcept
{
	const bool ok = Send(buf);
	CanMessageBuffer::Free(buf);
	return ok;
}

// Return a move message, if there is one. Caller must free the message buffer.
CanMessageBuffer * CanInterface::GetCanMove() noexcept
{
	return PendingMoves.GetMessage();
}

CanMessageBuffer *CanInterface::GetCanCommand() noexcept
{
	return PendingCommands.GetMessage();
}

// Process a received message. Return the buffer it arrived in if it is free for re-use, else nullptr.
CanMessageBuffer *CanInterface::ProcessReceivedMessage(CanMessageBuffer *buf) noexcept
{
	// Only respond to messages from a master address
#if defined(ATEIO) || defined(ATECM)
	if (buf->id.Src() == CanId::ATEMasterAddress)			// ATE boards only respond to the ATE master, because a main board under test may also transmit when it starts up
#else
	if (buf->id.Src() == CanId::MasterAddress || buf->id.Src() == CanId::ATEMasterAddress)
#endif
	{
		switch (buf->id.MsgType())
		{
#if SUPPORT_DRIVERS
		case CanMessageType::movementLinear:
			// Check for duplicate and out-of-sequence message
			{
				const uint32_t now = millis();
				if (buf->msg.moveLinear.whenToExecute == lastMotionMessageScheduledTime && now - lastMotionMessageReceivedAt < 100)
				{
					++duplicateMotionMessages;
					break;
				}
				lastMotionMessageScheduledTime = buf->msg.moveLinear.whenToExecute;
				lastMotionMessageReceivedAt = now;

				const int8_t seq = buf->msg.moveLinear.seq;
				if (seq != expectedSeq && expectedSeq != 0xFF)
				{
					++oosMessages;
				}
				expectedSeq = (seq + 1) & 7;
			}

			//TODO if we haven't established time sync yet then we should defer this
			buf->msg.moveLinear.whenToExecute += StepTimer::GetLocalTimeOffset();
			//DEBUG
			//accumulatedMotion +=buf->msg.moveLinear.perDrive[0].steps;
			//END
			PendingMoves.AddMessage(buf);
			Platform::OnProcessingCanMessage();
			return nullptr;

		case CanMessageType::stopMovement:
			moveInstance->StopDrivers(buf->msg.stopMovement.whichDrives);
			Platform::OnProcessingCanMessage();
			break;
#endif

		case CanMessageType::emergencyStop:
			Platform::EmergencyStop();
			break;

		case CanMessageType::acknowledgeAnnounce:
			mainBoardAcknowledgedAnnounce = true;
			Platform::OnProcessingCanMessage();
			break;

		case CanMessageType::startup:
			if (millis() > 1000 || isProgrammed)		// if we've only just powered up and the main board hasn't programmed us yet, no need to start up again
			{
				Platform::EmergencyStop();
			}
			Platform::OnProcessingCanMessage();
			break;

		case CanMessageType::controlledStop:
			debugPrintf("Unsupported CAN message type %u\n", (unsigned int)(buf->id.MsgType()));
			Platform::OnProcessingCanMessage();
			break;

		default:
			if (buf->id.Dst() == GetCanAddress() && buf->id.IsRequest())
			{
				isProgrammed = true;					// record that we've had a communication from the master since we started up
				PendingCommands.AddMessage(buf);		// it's addressed to us, so queue it for processing
				return nullptr;
			}
			break;
		}
	}

	return buf;
}

void CanInterface::Diagnostics(const StringRef& reply) noexcept
{
	unsigned int messagesQueuedForSending, messagesReceived, txTimeouts, messagesLost, busOffCount;
	can0dev->GetAndClearStats(messagesQueuedForSending, messagesReceived, txTimeouts, messagesLost, busOffCount);
	reply.lcatf("CAN messages queued %u, send timeouts %u, received %u, lost %u, free buffers %u, error reg %" PRIx32,
					messagesQueuedForSending, txTimeouts, messagesReceived, messagesLost, CanMessageBuffer::FreeBuffers(), can0dev->GetErrorRegister());
}

// Send an announcement message if we haven't had an announce acknowledgement from a main board. On return the buffer is available to use again.
void CanInterface::SendAnnounce(CanMessageBuffer *buf) noexcept
{
	if (!mainBoardAcknowledgedAnnounce)
	{
		auto msg = buf->SetupBroadcastMessage<CanMessageAnnounce>(boardAddress);
		msg->timeSinceStarted = millis();
		msg->numDrivers = NumDrivers;
		msg->zero = 0;
		SafeSnprintf(msg->boardTypeAndFirmwareVersion, ARRAY_SIZE(msg->boardTypeAndFirmwareVersion), "%s|%s (%s%s)", BOARD_TYPE_NAME, VERSION, IsoDate, TIME_SUFFIX);
		buf->dataLength = msg->GetActualDataLength();
		Send(buf);
	}
}

// This is called from the step ISR when the move is stopped by the Z probe
void CanInterface::MoveStoppedByZProbe() noexcept
{
	//TODO
}

void CanInterface::WakeAsyncSenderFromIsr() noexcept
{
	canAsyncSenderTask.GiveFromISR();
}

GCodeResult CanInterface::ChangeAddressAndDataRate(const CanMessageSetAddressAndNormalTiming &msg, const StringRef &reply) noexcept
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
bool CanInterface::GetCanMessage(CanMessageBuffer *buf) noexcept
{
	return can0dev->ReceiveMessage(CanDevice::RxBufferNumber::fifo0, 0, buf);
}

uint16_t CanInterface::GetTimeStampCounter() noexcept
{
	return can0dev->ReadTimeStampCounter();
}

uint16_t CanInterface::GetTimeStampPeriod() noexcept
{
	return can0dev->GetTimeStampPeriod();
}

extern "C" [[noreturn]] void CanClockLoop(void *) noexcept
{
	for (;;)
	{
		CanMessageBuffer buf(nullptr);
		can0dev->ReceiveMessage(CanDevice::RxBufferNumber::buffer0, TaskBase::TimeoutUnlimited, &buf);
		if (buf.id.MsgType() == CanMessageType::timeSync
#if defined(ATEIO) || defined(ATECM)
			&& (buf.id.Src() == CanId::ATEMasterAddress))			// ATE boards only respond to the ATE master, because a main board under test may also transmit when it starts up
#else
			&& (buf.id.Src() == CanId::MasterAddress || buf.id.Src() == CanId::ATEMasterAddress))
#endif
		{
			currentMasterAddress = buf.id.Src();
			StepTimer::ProcessTimeSyncMessage(buf.msg.sync, buf.dataLength, buf.timeStamp);
		}
	}
}

extern "C" [[noreturn]] void CanReceiverLoop(void *) noexcept
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
				buf = CanMessageBuffer::BlockingAllocate();
			}

			if (can0dev->ReceiveMessage(CanDevice::RxBufferNumber::fifo0, TaskBase::TimeoutUnlimited, buf))
			{
				buf = CanInterface::ProcessReceivedMessage(buf);
			}
			else
			{
				debugPrintf("CAN read err\n");
			}
		}
	}
}

extern "C" [[noreturn]] void CanAsyncSenderLoop(void *) noexcept
{
	CanMessageBuffer *buf;
	while ((buf = CanMessageBuffer::Allocate()) == nullptr)
	{
		delay(1);
	}

	for (;;)
	{
		// Set up a message ready
		auto msg = buf->SetupStatusMessage<CanMessageInputChanged>(CanInterface::GetCanAddress(), currentMasterAddress);
		msg->states = 0;
		msg->zero = 0;
		msg->numHandles = 0;

		const uint32_t timeToWait = InputMonitor::AddStateChanges(msg);
		if (msg->numHandles != 0)
		{
			buf->dataLength = msg->GetActualDataLength();
			CanInterface::SendAsync(buf);					// this doesn't free the buffer, so we can re-use it
		}
		TaskBase::Take(timeToWait);							// wait until we are woken up because a message is available, or we time out
	}
}

// End
