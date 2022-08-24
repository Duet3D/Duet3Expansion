/*
 * CanInterface.cpp
 *
 *  Created on: 17 Sep 2018
 *      Author: David
 */

#include "CanInterface.h"
#include "CanMessageQueue.h"

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

#if !RP2040
# include <hpl_user_area.h>
#endif

#define OOS_DEBUG		0				// debug for out-of-sequence errors

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

static unsigned int txTimeouts = 0;
static uint32_t lastCancelledId = 0;
static bool enabled = false;

constexpr CanDevice::Config Can0Config =
{
	.dataSize = 64,									// must be one of: 8, 12, 16, 20, 24, 32, 48, 64
	.numTxBuffers = 2,
	.txFifoSize = 10,								// enough to send a 512-byte response broken into 60-byte fragments
	.numRxBuffers = 1,								// we use a dedicated buffer for the clock sync messages
	.rxFifo0Size = 32,
	.rxFifo1Size = 0,								// we don't use FIFO 1
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
static unsigned int oosMessages1Ahead = 0, oosMessages2Ahead = 0, oosMessages2Behind = 0, oosMessagesOther = 0;
static unsigned int badMoveCommands = 0;
static uint32_t worstBadMove = 0;
static int32_t minAdvance, maxAdvance;
static uint32_t maxMotionProcessingDelay = 0;

static void ResetAdvance() noexcept
{
	minAdvance = std::numeric_limits<int32_t>::max();
	maxAdvance = std::numeric_limits<int32_t>::min();
}

#endif

uint8_t expectedSeq = 0xFF;

//DEBUG
//static int32_t accumulatedMotion = 0;

static CanMessageQueue PendingMoves;
static CanMessageQueue PendingCommands;

static Mutex txFifoMutex;

#if OOS_DEBUG

struct OosInfo
{
	uint8_t seq;
	uint32_t startTime;
};

OosInfo oosBuffer[16];

size_t oosCount = 0;

#endif

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
	// Create the mutex
	txFifoMutex.Create("CANtx");

	// Read the CAN timing data from the top part of the NVM User Row
	canConfigData = *reinterpret_cast<CanUserAreaData*>(NVMCTRL_USER + CanUserAreaDataOffset);

	CanTiming timing;
	canConfigData.GetTiming(timing);

	// Set up the CAN pins
#if SAME5x
	unsigned int whichPort;
	if (useAlternatePins)
	{
		SetPinFunction(PortAPin(23), GpioPinFunction::I);
		SetPinFunction(PortAPin(22), GpioPinFunction::I);
		whichPort = 0;											// use CAN0	on EXP1HCL
	}
	else
	{
		SetPinFunction(PortBPin(13), GpioPinFunction::H);
		SetPinFunction(PortBPin(12), GpioPinFunction::H);
		whichPort = 1;											// use CAN1 on EXP3HC
	}
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
	const unsigned int whichPort = 0;							// we always use CAN0 on the SAMC21
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

#if SUPPORT_DRIVERS
	ResetAdvance();
#endif
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
	MutexLocker lock(txFifoMutex);
	const uint32_t cancelledId = can0dev->SendMessage(CanDevice::TxBufferNumber::fifo, 1000, buf);
	if (cancelledId != 0)
	{
		lastCancelledId = cancelledId;
		++txTimeouts;
	}
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
CanMessageBuffer * CanInterface::GetCanMove(uint32_t timeout) noexcept
{
	return PendingMoves.GetMessage(timeout);
}

CanMessageBuffer *CanInterface::GetCanCommand(uint32_t timeout) noexcept
{
	return PendingCommands.GetMessage(timeout);
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
			// We can get out-of-sequence messages because of a bug in the CAN hardware; so use only the sequence number to detect duplicates
			{
				const int8_t seq = buf->msg.moveLinear.seq;
				if (((seq + 1) & 0x7F) == expectedSeq)
				{
					++duplicateMotionMessages;
#if OOS_DEBUG
					if (oosCount != 0)
					{
						oosBuffer[oosCount].seq = buf->msg.moveLinear.seq;
						oosBuffer[oosCount].startTime = buf->msg.moveLinear.whenToExecute;
					}
#endif
					break;
				}

				lastMotionMessageScheduledTime = buf->msg.moveLinear.whenToExecute;
				lastMotionMessageReceivedAt = millis();

				if (seq != expectedSeq && expectedSeq != 0xFF)
				{
					switch ((seq - expectedSeq) & 0x7F)
					{
					case 1:
						++oosMessages1Ahead;
						break;

					case 2:
						++oosMessages2Ahead;
						break;

					case 0x7E:
						++oosMessages2Behind;
						break;

					default:
						++oosMessagesOther;
						break;
					}
#if OOS_DEBUG
					if (oosCount == 0)
					{
						qq;
						oosCount = 1;
					}
#endif
				}

#if OOS_DEBUG
				if (oosCount != 0)
				{
					oosBuffer[oosCount].seq = seq;
					oosBuffer[oosCount].startTime = buf->msg.moveLinear.whenToExecute;
				}
#endif
				expectedSeq = (seq + 1) & 0x7F;
			}

			//TODO if we haven't established time sync yet then we should defer this
# if 0
			//DEBUG
			static uint32_t lastMoveEndedAt = 0;
			if (lastMoveEndedAt != 0)
			{
				const int32_t gap = (int32_t)(buf->msg.moveLinear.whenToExecute - lastMoveEndedAt);
				if (gap < 0)
				{
					++badMoveCommands;
					if ((uint32_t)(-gap) > worstBadMove)
					{
						worstBadMove = (uint32_t)(-gap);
					}
				}
			}
			lastMoveEndedAt = buf->msg.moveLinear.whenToExecute + buf->msg.moveLinear.accelerationClocks + buf->msg.moveLinear.steadyClocks + buf->msg.moveLinear.decelClocks;
# endif
			buf->msg.moveLinear.whenToExecute += StepTimer::GetLocalTimeOffset();

			// Track how much processing delay there was
			{
				const uint16_t timeStampNow = CanInterface::GetTimeStampCounter();

				// The time stamp counter runs at the CAN normal bit rate, but the step clock runs at 48MHz/64. Calculate the delay to in step clocks.
				// Datasheet suggests that on the SAMC21 only 15 bits of timestamp counter are readable, but Microchip confirmed this is a documentation error (case 00625843)
				const uint32_t timeStampDelay = ((uint32_t)((timeStampNow - buf->timeStamp) & 0xFFFF) * CanInterface::GetTimeStampPeriod()) >> 6;	// timestamp counter is 16 bits
				if (timeStampDelay > maxMotionProcessingDelay)
				{
					maxMotionProcessingDelay = timeStampDelay;
				}
			}

			// Track how much we are given moves in advance
			{
				const int32_t advance = (int32_t)(buf->msg.moveLinear.whenToExecute - StepTimer::GetTimerTicks());
				if (advance < minAdvance)
				{
					minAdvance = advance;
				}
				if (advance > maxAdvance)
				{
					maxAdvance = advance;
				}
			}

			//DEBUG
			//accumulatedMotion +=buf->msg.moveLinear.perDrive[0].steps;
			//END
			PendingMoves.AddMessage(buf);
			Platform::OnProcessingCanMessage();
			return nullptr;

		case CanMessageType::stopMovement:
			moveInstance->StopDrivers(buf->msg.stopMovement.whichDrives);
# if 0
			//DEBUG
			lastMoveEndedAt = 0;
# endif
			Platform::OnProcessingCanMessage();
			break;

		case CanMessageType::revertPosition:
			{
				// Generate a regular movement message from this revert message. First, extract the data so that we can use the same buffer, in case we are short of buffers.
				int32_t stepsToTake[NumDrivers];
				size_t index = 0;
				bool needSteps = false;
				const volatile int32_t * const lastMoveStepsTaken = moveInstance->GetLastMoveStepsTaken();
				for (size_t driver = 0; driver < NumDrivers; ++driver)
				{
					int32_t steps = 0;
					if (buf->msg.revertPosition.whichDrives & (1u << driver))
					{
						const int32_t stepsWanted = buf->msg.revertPosition.finalStepCounts[index++];
						const int32_t stepsTaken = lastMoveStepsTaken[driver];
						if (((stepsWanted >= 0 && stepsTaken > stepsWanted) || (stepsWanted <= 0 && stepsTaken < stepsWanted)))
						{
							steps = stepsWanted - stepsTaken;
							needSteps = true;
						}
					}
					stepsToTake[driver] = steps;
				}

				if (!needSteps)
				{
					break;
				}

				const uint32_t clocksAllowed = buf->msg.revertPosition.clocksAllowed;

				// Now we can re-use the buffer to build a regular movement message
				auto msg = buf->SetupRequestMessage<CanMessageMovementLinear>(0, GetCurrentMasterAddress(), GetCanAddress());
				for (size_t driver = 0; driver < NumDrivers; ++driver)
				{
					msg->perDrive[driver].steps = stepsToTake[driver];
				}

				// Set up some reasonable parameters for this move. The move must be shorter than clocksAllowed.
				// When writing this, clocksAllowed was equivalent to 40ms.
				// We allow 10ms delay time to allow the motor to stop and reverse direction, 10ms acceleration time, 5ms steady time and 10ms deceleration time.
				msg->accelerationClocks = msg->decelClocks = clocksAllowed/4;
				msg->steadyClocks = clocksAllowed/8;
				msg->whenToExecute = StepTimer::GetTimerTicks() + clocksAllowed/4;
				msg->numDrivers = NumDrivers;
				msg->pressureAdvanceDrives = 0;
				msg->seq = 0;
				msg->initialSpeedFraction = msg->finalSpeedFraction = 0.0;
			}
			PendingMoves.AddMessage(buf);
			Platform::OnProcessingCanMessage();
			return nullptr;
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
			else
			{
				mainBoardAcknowledgedAnnounce = false;	// we've recently started up, so no need to reset; but assume that we need to announce ourselves
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
	else if (buf->id.Dst() == CanId::BroadcastAddress && buf->id.MsgType() == CanMessageType::sensorTemperaturesReport && buf->id.IsRequest())
	{
		PendingCommands.AddMessage(buf);				// it's a broadcast message that we are interested in, so queue it for processing
		return nullptr;
	}

	return buf;											// ignore the message, reuse the buffer
}

void CanInterface::Diagnostics(const StringRef& reply) noexcept
{
	unsigned int messagesQueuedForSending, messagesReceived, messagesLost, busOffCount;
	can0dev->GetAndClearStats(messagesQueuedForSending, messagesReceived, messagesLost, busOffCount);
	reply.lcatf("CAN messages queued %u, send timeouts %u, received %u, lost %u, free buffers %u, min %u, error reg %" PRIx32,
					messagesQueuedForSending, txTimeouts, messagesReceived, messagesLost, CanMessageBuffer::GetFreeBuffers(), CanMessageBuffer::GetAndClearMinFreeBuffers(), can0dev->GetErrorRegister());
	txTimeouts = 0;
	if (lastCancelledId != 0)
	{
		CanId id;
		id.SetReceivedId(lastCancelledId);
		lastCancelledId = 0;
		reply.lcatf("Last cancelled message type %u dest %u", (unsigned int)id.MsgType(), id.Dst());
	}
#if SUPPORT_DRIVERS
	reply.lcatf("dup %u, oos %u/%u/%u/%u, bm %u, wbm %" PRIu32 ", rxMotionDelay %" PRIu32,
					duplicateMotionMessages, oosMessages1Ahead, oosMessages2Ahead, oosMessages2Behind, oosMessagesOther, badMoveCommands, worstBadMove, maxMotionProcessingDelay);
	duplicateMotionMessages = oosMessages1Ahead = oosMessages2Ahead = oosMessages2Behind = oosMessagesOther = badMoveCommands = 0;
	worstBadMove = maxMotionProcessingDelay = 0;
	if (minAdvance <= maxAdvance)
	{
		reply.catf( ", adv %" PRIi32 "/%" PRIi32, minAdvance, maxAdvance);
	}
	ResetAdvance();
#endif
}

// Send an announcement message if we need to, returning true if we sent one. On return the buffer is available to use again.
bool CanInterface::SendAnnounce(CanMessageBuffer *buf) noexcept
{
	if (mainBoardAcknowledgedAnnounce)
	{
		return false;
	}

	auto msg = buf->SetupStatusMessage<CanMessageAnnounceNew>(boardAddress, currentMasterAddress);
	msg->timeSinceStarted = millis();
	msg->numDrivers = NumDrivers;
	msg->zero = 0;
	memcpy(msg->uniqueId, Platform::GetUniqueId().GetRaw(), sizeof(msg->uniqueId));
	// Note, board type name, firmware version, firmware date and firmware time are limited to 43 characters in the new format
	// We use vertical-bar to separate the three fields: board type, firmware version, date/time
	SafeSnprintf(msg->boardTypeAndFirmwareVersion, ARRAY_SIZE(msg->boardTypeAndFirmwareVersion), "%s|%s|%s%.6s", BOARD_TYPE_NAME, VERSION, IsoDate, TIME_SUFFIX);
	buf->dataLength = msg->GetActualDataLength();
	Send(buf);
	return true;
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

// Send an event. The text will be truncated if it is longer than 55 characters.
void CanInterface::RaiseEvent(EventType type, uint16_t param, uint8_t device, const char *format, va_list vargs) noexcept
{
	CanMessageBuffer buf(nullptr);
	auto msg = buf.SetupStatusMessage<CanMessageEvent>(GetCanAddress(), GetCurrentMasterAddress());
	msg->eventType = type.ToBaseType();
	msg->deviceNumber = device;
	msg->eventParam = param;
	msg->zero = 0;
	SafeVsnprintf(msg->text, ARRAY_SIZE(msg->text), format, vargs);
	buf.dataLength = msg->GetActualDataLength();
	CanInterface::Send(&buf);
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
