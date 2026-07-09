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
#include <AppNotifyIndices.h>
#include <Platform/Platform.h>
#include <Platform/TaskPriorities.h>
#include <Movement/StepTimer.h>
#include <RTOSIface/RTOSIface.h>
#include <InputMonitors/InputMonitor.h>
#include <Movement/Move.h>
#include <General/SafeVsnprintf.h>

#define SUPPORT_CAN		1				// needed by CanDevice.h
#include <CanDevice.h>
#include <Hardware/IoPorts.h>
#include <General/RingBuffer.h>
#include <Version.h>

#if SUPPORT_DRIVERS && HAS_STALL_DETECT
# include <Movement/StepperDrivers/SmartDrivers.h>			// for extern declaration of driverStallsToNotify
#endif

#if RP2040
# include <Hardware/NonVolatileMemory.h>
#elif SAME5x || SAMC21
# include <hpl_user_area.h>
#endif

#if SAME5x
constexpr uint32_t CanUserAreaDataOffset = CanUserAreaDataOffset_SAME5x;
#elif SAMC21
constexpr uint32_t CanUserAreaDataOffset = CanUserAreaDataOffset_SAMC21;
#endif

#if SAMC21
constexpr unsigned int NumCanBuffers = 10;		// SAMC21-based boards have at most one driver, so allocate fewer message buffers to save RAM
#else
constexpr unsigned int NumCanBuffers = 40;
#endif

static CanDevice *can0dev = nullptr;
static CanUserAreaData canConfigData;
static CanAddress boardAddress;
static CanAddress currentMasterAddress =
#if defined(ATECM) || defined(ATEIO)
										CanId::ATEMasterAddress;
#else
										CanId::MasterAddress;
#endif

static uint8_t fastDataRate = 0;											// the fast data phase bit rate multiplier minus one. 0 means don't use BRS.
static uint8_t dTseg1MinusOne = 0;											// the fast data rate sample point minus one

static unsigned int txTimeouts = 0;
static unsigned int messagesIgnored = 0;
static uint32_t lastCancelledId = 0;
static bool enabled = false;

constexpr CanDevice::Config Can0Config =
{
	.dataSize = 64,									// must be one of: 8, 12, 16, 20, 24, 32, 48, 64
#if STM32H5
	.numTxBuffers = 0,								// STM32H5 has no dedicated transmit buffers
	.txFifoSize = 3,								// STM32H5 has fixed size FIFOs
#elif RP2040
	.numTxBuffers = 0,								// RP2040 implementation doesn't support transmit buffers
	.txFifo0Size = 16,
	.txFifo1Size = 2,								// RP2040 supports multiple transmit fifos so use another one instead of dedicated transmit buffers
#else
	.numTxBuffers = 2,								// we allocate 2 buffers to sending urgent messages in case we need to send more than one in quick succession
	.txFifoSize = 16,								// enough to send a 512-byte response broken into 60-byte fragments, plus status messages
#endif
#if RP2040 || STM32H5
	.numRxBuffers = 0,								// RP2040 implementation doesn't support receive buffers
#else
	.numRxBuffers = 1,								// we use a dedicated buffer for the clock sync messages
#endif
#if SAMC21
	.rxFifo0Size = 16,								// save RAM on SAMC21
#elif STM32H5
	.rxFifo0Size = 3,								// STM32H5 has fixed size FIFOs
#else
	.rxFifo0Size = 32,
#endif
#if STM32H5
	.rxFifo1Size = 3,								// STM32H5 has fixed size FIFOs
#elif RP2040
	.rxFifo1Size = 1,								// we use FIFO 1 instead of a dedicated receive buffer to receive CAN clock messages
#else
	.rxFifo1Size = 0,								// we don't use FIFO 1
#endif
#if STM32H5
	.numShortFilterElements = 28,					// we don't use 11-bit addresses
	.numExtendedFilterElements = 8,
	.txEventFifoSize = 3							// we don't need transmit events
#else
	.numShortFilterElements = 0,					// we don't use 11-bit addresses
	.numExtendedFilterElements = 3,
	.txEventFifoSize = 0							// we don't need transmit events
#endif
};

static_assert(Can0Config.IsValid());

#if STM32H5

// STM32H5 has fixed CAN buffer layout so nothing needed here

#elif STM32H7

static uint32_t *canMemory = reinterpret_cast<uint32_t *>(FDCAN_SRAM_BASE);

#else

// CAN buffer memory must be in the first 64Kb of RAM (SAME5x) or in non-cached RAM (SAME70), so put it in its own segment
static uint32_t can0Memory[Can0Config.GetMemorySize()] __attribute__ ((section (".CanMessage")));

#endif

// CanClock task
constexpr size_t CanClockTaskStackWords =
#if RP2040
										400;		// to allow calls to debugPrintf
#else
										140;		// 140 is enough (minimum is 133 on 3HC) when we enable CAN debugging
#endif

static Task<CanClockTaskStackWords> *canClockTask = nullptr;				// allocated dynamically to save RAM when updating the bootloader

// CanReceiver management task
constexpr size_t CanReceiverTaskStackWords =
#if RP2040
										400;								// to allow calls to debugPrintf
#else
										120;
#endif

static Task<CanReceiverTaskStackWords> *canReceiverTask = nullptr;			// allocated dynamically to save RAM when updating the bootloader

// Async sender task
constexpr size_t CanAsyncSenderTaskStackWords = 134;
static Task<CanAsyncSenderTaskStackWords> *canAsyncSenderTask = nullptr;	// allocated dynamically to save RAM when updating the bootloader

static bool mainBoardAcknowledgedAnnounce = false;	// true after the main board has acknowledged our announcement
static bool isProgrammed = false;					// true after the main board has sent us any configuration commands

#if SUPPORT_DRIVERS
static uint8_t expectedSeq = 0xFF;
static uint32_t lastMotionMessageScheduledTime = 0;
static uint32_t lastMotionMessageReceivedAt = 0;
static unsigned int duplicateMotionMessages = 0;
static unsigned int oosMessages1Ahead = 0, oosMessages2Ahead = 0, oosMessages2Behind = 0, oosMessagesOther = 0;
static int32_t minAdvance, maxAdvance;
static uint32_t maxMotionProcessingDelay = 0;

static void ResetAdvance() noexcept
{
	minAdvance = std::numeric_limits<int32_t>::max();
	maxAdvance = std::numeric_limits<int32_t>::min();
}

#endif

//DEBUG
//static int32_t accumulatedMotion = 0;

static CanMessageQueue PendingMoves;
static CanMessageQueue PendingCommands;

static Mutex txFifoMutex;

extern "C" [[noreturn]] void CanClockLoop(void *) noexcept;
extern "C" [[noreturn]] void CanReceiverLoop(void *) noexcept;
extern "C" [[noreturn]] void CanAsyncSenderLoop(void *) noexcept;

namespace CanInterface
{
	CanMessageBuffer *ProcessReceivedMessage(CanMessageBuffer *buf) noexcept;
}

// Initialise this module and the CAN hardware
void CanInterface::Init(CanAddress defaultBoardAddress, unsigned int whichPort, bool useLaterPins, bool full) noexcept
{
	// Create the mutex
	txFifoMutex.Create("CANtx");

	CanTiming timing;

#if RP2040
	{
		NonVolatileMemory mem(NvmPage::common);
		mem.GetCanSettings(canConfigData);
		canConfigData.GetTiming(timing);
	}
#else
	// Read the CAN timing data from the top part of the NVM User Row
	canConfigData = *reinterpret_cast<CanUserAreaData*>(NVMCTRL_USER + CanUserAreaDataOffset);
	canConfigData.GetTiming(timing);
#endif

	// Set up the CAN pins
#if SAME5x
	if (whichPort == 0)		// if using CAN0
	{
		if (useLaterPins)
		{
			SetPinFunction(PortAPin(25), GpioPinFunction::I);
			SetPinFunction(PortAPin(24), GpioPinFunction::I);
		}
		else
		{
			SetPinFunction(PortAPin(23), GpioPinFunction::I);
			SetPinFunction(PortAPin(22), GpioPinFunction::I);
		}
	}
	else					// using CAN1
	{
		if (useLaterPins)
		{
			SetPinFunction(PortBPin(15), GpioPinFunction::H);
			SetPinFunction(PortBPin(14), GpioPinFunction::H);
		}
		else
		{
			SetPinFunction(PortBPin(13), GpioPinFunction::H);
			SetPinFunction(PortBPin(12), GpioPinFunction::H);
		}
# ifdef EXP3HC
		// On later board variants, initialise the second CAN port to avoid generating spurious signals on the second CAN bus
		if (Platform::GetBoardVariant() > 2)
		{
			SetPinFunction(PortAPin(23), GpioPinFunction::I);
			SetPinFunction(PortAPin(22), GpioPinFunction::I);
		}
# endif
	}
#elif SAMC21
	if (whichPort == 0)		// if using CAN0
	{
		if (useLaterPins)
		{
			SetPinFunction(PortBPin(23), GpioPinFunction::G);
			SetPinFunction(PortBPin(22), GpioPinFunction::G);
		}
		else
		{
			SetPinFunction(PortAPin(25), GpioPinFunction::G);
			SetPinFunction(PortAPin(24), GpioPinFunction::G);
		}
	}
	else					// using CAN1 (only one set of pins available on SAMC21G)
	{
		SetPinFunction(PortBPin(11), GpioPinFunction::G);
		SetPinFunction(PortBPin(10), GpioPinFunction::G);
	}
#endif

	// Initialise the CAN hardware, using the timing data if it was valid
	can0dev = CanDevice::Init(
#if RP2040
								CanTxPin, CanRxPin,				// which pins we use for CAN transmit and receive
#else
								0, whichPort,
#endif
								Can0Config,
#if STM32H5
								reinterpret_cast<uint32_t *>(SRAMCAN_BASE_NS + 0x0350 * whichPort),			// STM32H5 has fixed message buffer allocation
#elif STM32H7
								canMemory,
#else
								can0Memory,
#endif
								timing, nullptr);

#if STM32H7
	canMemory += Can0Config.GetMemorySize();					// advance memory pointer in case we create another CAN device
#endif

#ifdef SAMMYC21
	SetPinMode(CanStandbyPin, OUTPUT_LOW);						// take the CAN drivers out of standby
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
		can0dev->SetExtendedFilterElement(1,
#if RP2040
											CanDevice::RxBufferNumber::fifo1,
#else
											CanDevice::RxBufferNumber::buffer0,
#endif
											((uint32_t)CanMessageType::timeSync << CanId::MessageTypeShift) | ((uint32_t)CanId::BroadcastAddress << CanId::DstAddressShift),
#if RP2040
											(CanId::MessageTypeMask << CanId::MessageTypeShift) | (CanId::BoardAddressMask << CanId::DstAddressShift)
#else
											1					// mask is unused when using a dedicated Rx buffer, but must be nonzero to enable the element
#endif
											);

		// Set up a filter for all other broadcast messages in FIFO 0
		can0dev->SetExtendedFilterElement(2, CanDevice::RxBufferNumber::fifo0,
											(uint32_t)CanId::BroadcastAddress << CanId::DstAddressShift,
											CanId::BoardAddressMask << CanId::DstAddressShift);
	}

#if !RP2040
	// For receiving into a dedicated buffer, the mask is ignored and only the extended ID mask is applied. We need to ignore the source address.
	can0dev->SetExtendedIdMask(0x1FFFFFFF & ~(CanId::BoardAddressMask << CanId::SrcAddressShift));
#endif

	can0dev->Enable();

	enabled = true;

	if (full)
	{
		CanMessageBuffer::Init(NumCanBuffers);

		// Create the clock sync
		canClockTask = new Task<CanClockTaskStackWords>;
		canClockTask->Create(CanClockLoop, "CanClock", nullptr, TaskPriority::CanClockPriority);

		// Create the task that receives CAN messages
		canReceiverTask = new Task<CanReceiverTaskStackWords>;
		canReceiverTask->Create(CanReceiverLoop, "CanRecv", nullptr, TaskPriority::CanReceiverPriority);

		// Create the task that send endstop etc. updates
		canAsyncSenderTask = new Task<CanAsyncSenderTaskStackWords>;
		canAsyncSenderTask->Create(CanAsyncSenderLoop, "CanAsync", nullptr, TaskPriority::CanAsyncSenderPriority);
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

	if (canReceiverTask != nullptr) { canReceiverTask->TerminateAndUnlink(); }
	if (canAsyncSenderTask != nullptr) { canAsyncSenderTask->TerminateAndUnlink(); }
	if (canClockTask != nullptr) { canClockTask->TerminateAndUnlink(); }
}

CanAddress CanInterface::GetCanAddress() noexcept
{
	return boardAddress;
}

CanAddress CanInterface::GetCurrentMasterAddress() noexcept
{
	return currentMasterAddress;
}

void CanInterface::ReportCanTiming(const StringRef& reply) noexcept
{
	CanTiming timing;
	can0dev->GetLocalCanTiming(timing);
	reply.printf("CAN arbitration speed %.1fkbps, sample point %.2f, jump width %.2f, ",
					(double)((float)CanTiming::ClockFrequency/(float)(1000 * timing.period)),
					(double)((float)(timing.nTseg1 + 1)/(float)timing.period),
					(double)((float)timing.nJumpWidth/(float)timing.period));
	if (fastDataRate == 0)
	{
		reply.cat("bit rate switching disabled");
	}
	else
	{
		const uint32_t dataPeriod = timing.period/(fastDataRate + 1);
		reply.catf("data speed %.1fkbps, sample point %.2f, jump width %.2f",
					(double)((float)CanTiming::ClockFrequency/(float)(1000 * dataPeriod)),
					(double)((float)(timing.dTseg1 + 1)/(float)dataPeriod),
					(double)((float)timing.dJumpWidth/(float)dataPeriod));
	}
}

// Check that the BRS setting we are using matches the one n the time sync message and change it if necessary
void CanInterface::CheckBrs(const CanMessageTimeSync& msg) noexcept
{
	if (msg.fastDataRate != fastDataRate || msg.tseg1Minus1 != dTseg1MinusOne)
	{
		CanTiming timing;
		can0dev->GetLocalCanTiming(timing);
		timing.EnableBrs(msg.fastDataRate + 1);
		timing.SetDataSamplePointDirect(msg.tseg1Minus1);
		can0dev->ChangeLocalCanTiming(timing);
		fastDataRate = msg.fastDataRate;
		dTseg1MinusOne = msg.tseg1Minus1;
	}
}

// Send a message. On return the buffer is available to the caller to re-use or free.
// Any extra bytes needed as padding are set to zero by the CAN driver.
bool CanInterface::Send(CanMessageBuffer *buf) noexcept
{
	//TODO option to not force sending, and return true only if successful?
	MutexLocker lock(txFifoMutex);								// this is called from multiple tasks so we need to lock the mutex
	const uint32_t cancelledId = can0dev->SendMessage(CanDevice::TxBufferNumber::fifo, 1000, buf);
	if (cancelledId != 0)
	{
		lastCancelledId = cancelledId;
		++txTimeouts;
	}
	return true;
}

// Send a high-priority message. Called only from the CanAsyncSender task, so we don't need to use a mutex.
// We reserve two buffers for sending these messages.
bool CanInterface::SendAsync(CanMessageBuffer *buf) noexcept
{
#if RP2040
	// RP2040 doesn't support dedicated buffers but it can support more than one fifo. We prioritise fifo1 over fifo0.
	const CanDevice::TxBufferNumber bufferNumber = CanDevice::TxBufferNumber::fifo1;
#else
	const CanDevice::TxBufferNumber bufferNumber = (can0dev->IsSpaceAvailable(CanDevice::TxBufferNumber::buffer0, 0))
													? CanDevice::TxBufferNumber::buffer0
														: CanDevice::TxBufferNumber::buffer1;
#endif
	const uint32_t cancelledId = can0dev->SendMessage(bufferNumber, 1000, buf);
	if (cancelledId != 0)
	{
		lastCancelledId = cancelledId;
		++txTimeouts;
	}
	return true;
}

bool CanInterface::SendAndFree(CanMessageBuffer *buf) noexcept
{
	const bool ok = Send(buf);
	CanMessageBuffer::Free(buf);
	return ok;
}

// Return a move message, if there is one. Caller must free the message buffer.
CanMessageBuffer *CanInterface::GetCanMove(uint32_t timeout) noexcept
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
	// Mostly, we only respond to messages from a master address
#if defined(ATEIO) || defined(ATECM)
	if (buf->id.Src() == CanId::ATEMasterAddress)			// ATE boards only respond to the ATE master, because a main board under test may also transmit when it starts up
#else
	if (buf->id.Src() == CanId::MasterAddress || buf->id.Src() == CanId::ATEMasterAddress)
#endif
	{
		switch (buf->id.MsgType())
		{
#if SUPPORT_DRIVERS
		case CanMessageType::movementLinearShaped:
			// Check for duplicate and out-of-sequence message
			// We can get out-of-sequence messages because of a bug in the CAN hardware; so use only the sequence number to detect duplicates
			{
				const int8_t seq = buf->msg.moveLinearShaped.seq;
				if (((seq + 1) & CanMessageMovementLinearShaped::SeqMask) == expectedSeq)
				{
					++duplicateMotionMessages;
					break;
				}

				lastMotionMessageScheduledTime = buf->msg.moveLinearShaped.whenToExecute;
				lastMotionMessageReceivedAt = millis();

				if (seq != expectedSeq && expectedSeq != 0xFF)
				{
					switch ((seq - expectedSeq) & CanMessageMovementLinearShaped::SeqMask)
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
				}

				expectedSeq = (seq + 1) & CanMessageMovementLinearShaped::SeqMask;
			}

			// If we are not synced then don't accept any movement messages, because they are likely just to get queued and not executed within a reasonable time
			if (StepTimer::IsSynced())
			{
				// Track how much processing delay there was
				{
#if RP2040 && !USE_SPICAN
					// RP2040 uses the low 16 bits of the step counter for the time stamp
					const uint16_t timeStampNow = StepTimer::GetTimerTicks();
					const uint32_t timeStampDelay = (uint32_t)((timeStampNow - buf->timeStamp) & 0xFFFF);	// the delay in step clocks
#else
					const uint16_t timeStampNow = CanInterface::GetTimeStampCounter();

					// The time stamp counter runs at the CAN normal bit rate, but the step clock runs at 48MHz/64. Calculate the delay to in step clocks.
					// Datasheet suggests that on the SAMC21 only 15 bits of timestamp counter are readable, but Microchip confirmed this is a documentation error (case 00625843)
					const uint32_t timeStampDelay = ((uint32_t)((timeStampNow - buf->timeStamp) & 0xFFFF) * CanInterface::GetTimeStampPeriod()) >> 6;	// timestamp counter is 16 bits
#endif
					if (timeStampDelay > maxMotionProcessingDelay)
					{
						maxMotionProcessingDelay = timeStampDelay;
					}
				}

				// Track how much we are given moves in advance
				{
					const int32_t advance = (int32_t)(buf->msg.moveLinearShaped.whenToExecute - StepTimer::GetMovementTimerTicks());
					if (advance < minAdvance)
					{
						minAdvance = advance;
					}
					if (advance > maxAdvance)
					{
						maxAdvance = advance;
					}
				}

				PendingMoves.AddMessage(buf);
			}
			else
			{
				++messagesIgnored;
			}

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
				static_assert(NumDrivers < MaxLinearDriversPerCanSlave);			// the code assumes that we can build a movement message describing all our drivers
				int32_t stepsToTake[NumDrivers];
				size_t index = 0;
				bool needSteps = false;
				for (size_t driver = 0; driver < NumDrivers; ++driver)
				{
					int32_t steps = 0;
					if (buf->msg.revertPosition.whichDrives & (1u << driver))
					{
						const int32_t stepsWanted = buf->msg.revertPosition.finalStepCounts[index++];
						const int32_t stepsTaken = moveInstance->GetLastMoveStepsTaken(driver);
						//debugPrintf("Driver %u revert, wanted %ld taken %ld\n", driver, stepsWanted, stepsTaken);
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
				auto msg = buf->SetupRequestMessageNoRid<CanMessageMovementLinearShaped>(GetCurrentMasterAddress(), GetCanAddress());
				for (size_t driver = 0; driver < NumDrivers; ++driver)
				{
					msg->perDrive[driver].steps = stepsToTake[driver];
				}

				// Set up some reasonable parameters for this move. The move must be shorter than clocksAllowed.
				// When writing this, clocksAllowed was equivalent to 40ms.
				// We allow 10ms delay time to allow the motor to stop and reverse direction, 10ms acceleration time, 5ms steady time and 10ms deceleration time.
				msg->accelerationClocks = msg->decelClocks = clocksAllowed/4;
				msg->steadyClocks = clocksAllowed/8;
				// If we start and stop at zero speed:
				//	acceleration and deceleration distances are each 0.5 * a * accelerationClocks^2
				//	steady distance is a * accelerationClocks * steadyClocks
				// so totalDistance is a * accelerationClocks * (accelerationClock + steadyClocks)
				// The acceleration and deceleration must be specified with the distance normalised to 1.0
				msg->acceleration = msg->deceleration = 1.0/(msg->accelerationClocks * (msg->accelerationClocks + msg->steadyClocks));
				msg->whenToExecute = StepTimer::GetMovementTimerTicks() + clocksAllowed/4;
				msg->numDrivers = NumDrivers;
				msg->extruderDrives = 0;
				msg->usePressureAdvance = 0;
				msg->useLateInputShaping = 0;
				msg->seq = 0;
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
#ifdef DEBUG
			debugPrintf("Unsupported CAN message type %u\n", (unsigned int)(buf->id.MsgType()));
#endif
			Platform::OnProcessingCanMessage();
			break;

		case CanMessageType::sensorTemperaturesReport:
			PendingCommands.AddMessage(buf);			// it's a broadcast message that we are interested in from the master, so queue it for processing
			return nullptr;

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
		PendingCommands.AddMessage(buf);				// it's a broadcast message that we are interested in from an expansion board, so queue it for processing
		return nullptr;
	}

	return buf;											// ignore the message, reuse the buffer
}

void CanInterface::Diagnostics(const StringRef& reply) noexcept
{
	CanDevice::CanStats stats;
	can0dev->GetAndClearStats(stats);
#if RP2040 && !USE_SPICAN
	reply.lcatf("CAN messages queued %u, send timeouts %u, received %u, free buffers %u, min %u",
					stats.messagesQueuedForSending, txTimeouts, stats.messagesReceived, CanMessageBuffer::GetFreeBuffers(), CanMessageBuffer::GetAndClearMinFreeBuffers());
	CanErrorCounts errs;
	can0dev->GetAndClearErrorCounts(errs);
	reply.lcatf("Lost0 %" PRIu32 ", lost1 %" PRIu32 ", wt %" PRIu32 ", bs %" PRIu32 ", scp %" PRIu32 ", wsc %" PRIu32
					", wcrc %" PRIu32 ", mcd %" PRIu32 ", noack %" PRIu32 ", meof1 %" PRIu32 ", meof2 %" PRIu32 ", lack %" PRIu32,
					errs.rxFifoOverlow[0], errs.rxFifoOverlow[1], errs.wrongMessageType, errs.badStuffing, errs.stuffCountParity, errs.wrongStuffCount,
						errs.wrongCrc, errs.missingCrcDelimiter, errs.noAck, errs.missingEofBit1, errs.missingEofBit2, errs.tooLateToAck);
#else
	reply.lcatf("CAN messages queued %u, send timeouts %u, received %u, lost %u, ignored %u, errs %u, boc %u, free buffers %u, min %u, error reg %" PRIx32,
					stats.messagesQueuedForSending, txTimeouts, stats.messagesReceived, stats.messagesLost, messagesIgnored, stats.protocolErrors, stats.busOffCount,
					CanMessageBuffer::GetFreeBuffers(), CanMessageBuffer::GetAndClearMinFreeBuffers(), can0dev->GetErrorRegister());
#endif
	txTimeouts = 0;
	messagesIgnored = 0;
	if (lastCancelledId != 0)
	{
		CanId id;
		id.SetReceivedId(lastCancelledId);
		lastCancelledId = 0;
		reply.lcatf("Last cancelled message type %u dest %u", (unsigned int)id.MsgType(), id.Dst());
	}

#if SUPPORT_DRIVERS
	reply.lcatf("dup %u, oos %u/%u/%u/%u, rxMotionDelay %" PRIu32,
					duplicateMotionMessages, oosMessages1Ahead, oosMessages2Ahead, oosMessages2Behind, oosMessagesOther, maxMotionProcessingDelay);
	duplicateMotionMessages = oosMessages1Ahead = oosMessages2Ahead = oosMessages2Behind = oosMessagesOther = 0;
	maxMotionProcessingDelay = 0;
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

	auto msg = buf->SetupRequestMessageNoRid<CanMessageAnnounceV1>(boardAddress, currentMasterAddress);
	msg->timeSinceStarted = millis();
	msg->numDrivers = NumDrivers;
	msg->usesUf2Binary = BOARD_USES_UF2_BINARY;
	msg->zero = 0;
	memcpy(msg->uniqueId, Platform::GetUniqueId().GetRaw(), sizeof(msg->uniqueId));
	// Note, board type name, firmware version, firmware date and firmware time are limited to 43 characters in the new format
	// We use vertical-bar to separate the three fields: board type, firmware version, date/time
	SafeSnprintf(msg->boardTypeAndFirmwareVersion, ARRAY_SIZE(msg->boardTypeAndFirmwareVersion), "%s|%s|%s%.6s", BOARD_TYPE_NAME, VERSION, DateText, TimeSuffix);
	buf->dataLength = msg->GetActualDataLength();
	Send(buf);
	Platform::OnProcessingCanMessage();								// flash the ACT LED
	return true;
}

// Wake the CAN sender task, either from an ISR or from a task
void CanInterface::WakeAsyncSender() noexcept
{
	if (inInterrupt())
	{
		canAsyncSenderTask->GiveFromISR(NotifyIndices::CanAsyncSender);
	}
	else
	{
		canAsyncSenderTask->Give(NotifyIndices::CanAsyncSender);
	}
}

// Wake the CAN sender task when the caller is definitely an ISR
void CanInterface::WakeAsyncSenderFromIsr() noexcept
{
	canAsyncSenderTask->GiveFromISR(NotifyIndices::CanAsyncSender);
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
#if RP2040
			NonVolatileMemory mem(NvmPage::common);
			mem.SetCanSettings(canConfigData);
			mem.EnsureWritten();
#else
			const int32_t rc = _user_area_write(reinterpret_cast<void*>(NVMCTRL_USER), CanUserAreaDataOffset, reinterpret_cast<const uint8_t*>(&canConfigData), sizeof(canConfigData));
			if (rc != 0)
			{
				reply.printf("Failed to write NVM user area, code %" PRIi32, rc);
				return GCodeResult::error;
			}
#endif
		}
		else
		{
			ReportCanTiming(reply);
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

#if !RP2040 || USE_SPICAN

uint16_t CanInterface::GetTimeStampCounter() noexcept
{
	return can0dev->ReadTimeStampCounter();
}

uint16_t CanInterface::GetTimeStampPeriod() noexcept
{
	return can0dev->GetTimeStampPeriod();
}

#endif

// Send an event. The text will be truncated if it is longer than 55 characters.
void CanInterface::RaiseEvent(EventType type, uint16_t param, uint8_t device, const char *format, va_list vargs) noexcept
{
	CanMessageBuffer buf;
	auto msg = buf.SetupRequestMessageNoRid<CanMessageEvent>(GetCanAddress(), GetCurrentMasterAddress());
	msg->eventType = type.ToBaseType();
	msg->deviceNumber = device;
	msg->eventParam = param;
	SafeVsnprintf(msg->text, ARRAY_SIZE(msg->text), format, vargs);
	buf.dataLength = msg->GetActualDataLength();
	CanInterface::Send(&buf);
	Platform::OnProcessingCanMessage();								// flash the ACT LED
}

extern "C" [[noreturn]] void CanClockLoop(void *) noexcept
{
	for (;;)
	{
		CanMessageBuffer buf;
		can0dev->ReceiveMessage(
#if RP2040
								CanDevice::RxBufferNumber::fifo1,
#else
								CanDevice::RxBufferNumber::buffer0,
#endif
									TaskBase::TimeoutUnlimited, &buf);
		if (buf.id.MsgType() == CanMessageType::timeSync
#if defined(ATEIO) || defined(ATECM)
			&& (buf.id.Src() == CanId::ATEMasterAddress))			// ATE boards only respond to the ATE master, because a main board under test may also transmit when it starts up
#else
			&& (buf.id.Src() == CanId::MasterAddress || buf.id.Src() == CanId::ATEMasterAddress))
#endif
		{
			currentMasterAddress = buf.id.Src();
			StepTimer::ProcessTimeSyncMessage(buf.msg.sync, buf.dataLength, buf.timeStamp);
			CanInterface::CheckBrs(buf.msg.sync);
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
#ifdef DEBUG
				debugPrintf("CAN read err\n");
#endif
			}
		}
	}
}

#if !USE_SERIAL_DEBUG

// Debugging support. We maintain a ring buffer for debugging text, written by debugPrintf via DebugPutc, read by the CAN Async Sender task.
static RingBuffer<char> debugBuffer;
constexpr size_t DebugBufferSize = 512;
static_assert((DebugBufferSize & (DebugBufferSize - 1)) == 0);		// DebugBufferSize must be a power of 2
volatile bool debugBufferBeingWritten = false;

bool CanInterface::DebugPutc(char c) noexcept
{
	if (c != 0)
	{
		const bool b = debugBuffer.PutItem(c);
		debugBufferBeingWritten = b;
		return b;
	}

	debugBufferBeingWritten = false;
	WakeAsyncSender();
	return true;
}

#endif

// Code executed by the sync sender task
extern "C" [[noreturn]] void CanAsyncSenderLoop(void *) noexcept
{
#if !USE_SERIAL_DEBUG
	debugBuffer.Init(DebugBufferSize);
#endif

	CanMessageBuffer buf;

	for (;;)
	{
		// Set up a message ready
		auto msg = buf.SetupRequestMessageNoRid<CanMessageInputChangedV2>(CanInterface::GetCanAddress(), currentMasterAddress);
		msg->states = 0;
		msg->numHandles = 0;

#if SUPPORT_DRIVERS && HAS_STALL_DETECT
		// Start by adding any stall detect endstops pending. Do this first so that it must succeed.
		const uint16_t stallNotifications = SmartDrivers::driverStallsToNotify.exchange(0);
		if (stallNotifications != 0)
		{
			constexpr RemoteInputHandle h(RemoteInputHandle::typeStallEndstop, 0, 0);
			(void)msg->AddEntry(h.asU16(), StepTimer::GetMasterTime(), (uint32_t)stallNotifications, true);
		}
#endif
		uint32_t timeToWait = InputMonitor::AddStateChanges(msg);
		if (msg->numHandles != 0)
		{
			buf.dataLength = msg->GetActualDataLength();
			CanInterface::SendAsync(&buf);									// this doesn't free the buffer, so we can re-use it
			Platform::OnProcessingCanMessage();								// flash the ACT LED
		}

#if !USE_SERIAL_DEBUG
		size_t numChars;
		if (!debugBufferBeingWritten && (numChars = debugBuffer.ItemsPresent()) != 0)
		{
			auto debugMsg = buf.SetupRequestMessageNoRid<CanMessageDebugText>(CanInterface::GetCanAddress(), currentMasterAddress);
			size_t numToSend = min<size_t>(numChars, ARRAY_SIZE(debugMsg->text));
			debugBuffer.GetBlock(debugMsg->text, numToSend);
			if (debugMsg->text[numToSend - 1] == '\n')
			{
				debugMsg->text[numToSend - 1] = 0;
			}
			else if (numToSend < ARRAY_SIZE(debugMsg->text))
			{
				debugMsg->text[numToSend++] = 0;
			}
			buf.dataLength = numToSend;
			CanInterface::SendAsync(&buf);
			if (debugBuffer.ItemsPresent() != 0 && timeToWait > 10)
			{
				timeToWait = 10;
			}
		}
#endif

		TaskBase::TakeIndexed(NotifyIndices::CanAsyncSender, timeToWait);	// wait until we are woken up because a message is available, or we time out
	}
}

// End
