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
#include <Movement/StepTimer.h>
#include <RTOSIface/RTOSIface.h>
#include <InputMonitors/InputMonitor.h>
#include <Movement/Move.h>
#include <Hardware/CanDriver.h>
#include <Hardware/IoPorts.h>
#include <Version.h>
#include <peripheral_clk_config.h>
#include <hpl_user_area.h>

const unsigned int NumCanBuffers = 40;

static CanUserAreaData canConfigData;
static CanAddress boardAddress;
static bool enabled = false;

#if defined(SAME51)
constexpr uint32_t CanUserAreaDataOffset = 512 - sizeof(CanUserAreaData);
#elif defined(SAMC21)
constexpr uint32_t CanUserAreaDataOffset = 256 - sizeof(CanUserAreaData);
#endif

// CanReceiver management task
constexpr size_t CanReceiverTaskStackWords = 400;
static Task<CanReceiverTaskStackWords> canReceiverTask;

// Async sender task
constexpr size_t CanAsyncSenderTaskStackWords = 400;
static Task<CanAsyncSenderTaskStackWords> canAsyncSenderTask;

static TaskHandle sendingTaskHandle = nullptr;

static bool mainBoardAcknowledgedAnnounce = false;	// true after the main board has acknowledged our announcement
static bool isProgrammed = false;					// true after the main board has sent us any configuration commands

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

static CanMessageQueue PendingMoves;
static CanMessageQueue PendingCommands;

static can_async_descriptor CAN_0;

#ifdef SAME51

/**
 * \brief CAN initialization function
 *
 * Enables CAN peripheral, clocks and initializes CAN driver
 */
static void CAN_0_init(const CanTiming& timing)
{
	hri_mclk_set_AHBMASK_CAN1_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, CAN1_GCLK_ID, CONF_GCLK_CAN1_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	can_async_init(&CAN_0, CAN1, timing);
	gpio_set_pin_function(PortBPin(13), PINMUX_PB13H_CAN1_RX);
	gpio_set_pin_function(PortBPin(12), PINMUX_PB12H_CAN1_TX);
}

#endif

#ifdef SAMC21

/**
 * \brief CAN initialization function
 *
 * Enables CAN peripheral, clocks and initializes CAN driver
 */
static void CAN_0_init(const CanTiming& timing)
{
	hri_mclk_set_AHBMASK_CAN0_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, CAN0_GCLK_ID, CONF_GCLK_CAN0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	can_async_init(&CAN_0, CAN0, timing);
#ifdef SAMMYC21
	gpio_set_pin_function(PortBPin(23), PINMUX_PB23G_CAN0_RX);
	gpio_set_pin_function(PortBPin(22), PINMUX_PB22G_CAN0_TX);
	IoPort::SetPinMode(CanStandbyPin, OUTPUT_LOW);					// take the CAN drivers out of standby
#else
	gpio_set_pin_function(PortAPin(25), PINMUX_PA25G_CAN0_RX);
	gpio_set_pin_function(PortAPin(24), PINMUX_PA24G_CAN0_TX);
#endif
}

#endif

namespace CanInterface
{
	void ProcessReceivedMessage(CanMessageBuffer *buf);
}

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
		const int32_t rslt = can_async_read(&CAN_0, &msg);		// fetch the message
		if (rslt == ERR_NOT_FOUND)
		{
			TaskBase::Take();									// wait until we are woken up because a message is available
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
		msg->spare = 0;
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

void CanInterface::Init(CanAddress defaultBoardAddress)
{
	// Read the CAN timing data from the top part of the NVM User Row
	canConfigData = *reinterpret_cast<CanUserAreaData*>(NVMCTRL_USER + CanUserAreaDataOffset);

	CanTiming timing;
	canConfigData.GetTiming(timing);

	// Initialise the CAN hardware, using the timing data if it was valid
	CAN_0_init(timing);

	boardAddress = canConfigData.GetCanAddress(defaultBoardAddress);
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

// Send a message. On return the buffer is available to the caller to re-use or free.
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

// Process a received message and (eventually) release the buffer that it arrived in
void CanInterface::ProcessReceivedMessage(CanMessageBuffer *buf)
{
	switch (buf->id.MsgType())
	{
	case CanMessageType::timeSync:
		//TODO re-implement this as a PLL and use the CAN time stamps for greater accuracy
		StepTimer::SetLocalTimeOffset(StepTimer::GetTimerTicks() - buf->msg.sync.timeSent);
		CanMessageBuffer::Free(buf);
		break;

	case CanMessageType::movement:
		//TODO if we haven't established time sync yet then we should defer this
		buf->msg.move.whenToExecute += StepTimer::GetLocalTimeOffset();
		PendingMoves.AddMessage(buf);
		Platform::OnProcessingCanMessage();
		break;

	case CanMessageType::stopMovement:
		moveInstance->StopDrivers(buf->msg.stopMovement.whichDrives);
		CanMessageBuffer::Free(buf);
		Platform::OnProcessingCanMessage();
		break;

	case CanMessageType::emergencyStop:
		Platform::EmergencyStop();					// doesn't return
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
	reply.lcatf("Free CAN buffers: %u", CanMessageBuffer::FreeBuffers());
}

// Send an announcement message if we haven't had an announce acknowledgement form the main board. On return the buffer is available to use again.
void CanInterface::SendAnnounce(CanMessageBuffer *buf)
{
	if (!mainBoardAcknowledgedAnnounce)
	{
		auto msg = buf->SetupRequestMessage<CanMessageAnnounce>(0, boardAddress, CanId::MasterAddress);
		msg->timeSinceStarted = millis();
		msg->numDrivers = NumDrivers;
		msg->spare = 0;
		constexpr size_t BoardTypeLength = strlen(BoardTypeName);
		memcpy(msg->boardTypeAndFirmwareVersion, BoardTypeName, BoardTypeLength);
		msg->boardTypeAndFirmwareVersion[BoardTypeLength] = '|';
		strncpy(msg->boardTypeAndFirmwareVersion + BoardTypeLength + 1, FirmwareVersion, ARRAY_SIZE(msg->boardTypeAndFirmwareVersion) - BoardTypeLength - 1);
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
			GetLocalCanTiming(&CAN_0, timing);
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

// End
