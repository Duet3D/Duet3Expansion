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
constexpr size_t CanReceiverTaskStackWords = 400;
static Task<CanReceiverTaskStackWords> canReceiverTask;

static CanMessageBuffer *pendingBuffers;
static CanMessageBuffer *lastBuffer;			// only valid when pendingBuffers != nullptr

extern "C" struct can_async_descriptor CAN_0;

extern "C" void CAN_0_tx_callback(struct can_async_descriptor *const descr)
{
	(void)descr;
}

extern "C" void CAN_0_rx_callback(struct can_async_descriptor *const descr)
{
	canReceiverTask.GiveFromISR();
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

// CanMovementMessage is declared in project Duet3Expansion, so we need to implement its members here
void CanMovementMessage::DebugPrint()
{
	debugPrintf("Can: %08" PRIx32 " %08" PRIx32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " %.2f %.2f:",
		timeNow, moveStartTime , accelerationClocks, steadyClocks, decelClocks, (double)initialSpeedFraction, (double)finalSpeedFraction);
#if 0
	for (size_t i = 0; i < DriversPerCanBoard; ++i)
	{
		debugPrintf(" %" PRIi32, perDrive[i].steps);
	}
	debugPrintf("\n");
#endif
}

extern "C" void CanReceiverLoop(void *)
{
//	int32_t can_async_set_mode(struct can_async_descriptor *const descr, enum can_mode mode);

	can_async_register_callback(&CAN_0, CAN_ASYNC_RX_CB, (FUNC_PTR)CAN_0_rx_callback);
	can_filter filter;
	filter.id = Platform::ReadBoardId();
	filter.mask = 3;
	can_async_set_filter(&CAN_0, 0, CAN_FMT_STDID, &filter);
	can_async_set_filter(&CAN_0, 1, CAN_FMT_EXTID, &filter);

	NVIC_SetPriority(CAN1_IRQn, NvicPriorityCan);
	can_async_enable(&CAN_0);

	for (;;)
	{
		TaskBase::Take(Mutex::TimeoutUnlimited);
		CanMessageBuffer *buf = CanMessageBuffer::Allocate();
		if (buf != nullptr)
		{
			can_message msg;
			msg.data = reinterpret_cast<uint8_t*>(&(buf->msg));
			const int32_t rslt = can_async_read(&CAN_0, &msg);
			switch (rslt)
			{
			case ERR_NONE:
				buf->msg.moveStartTime += StepTimer::GetInterruptClocks() - buf->msg.timeNow;
				buf->next = nullptr;
				{
					TaskCriticalSectionLocker lock;

					if (pendingBuffers == nullptr)
					{
						pendingBuffers = lastBuffer = buf;
					}
					else
					{
						lastBuffer->next = buf;
					}
				}

				buf->msg.DebugPrint();
				break;

			default:
				debugPrintf("CAN read err %d", (int)rslt);
				CanMessageBuffer::Free(buf);
				break;
			}
		}
	}
}

void CanSlaveInterface::Init()
{
	CanMessageBuffer::Init(NumCanBuffers);
	pendingBuffers = nullptr;

	// Create the task that sends CAN messages
	canReceiverTask.Create(CanReceiverLoop, "CanSender", nullptr, TaskBase::CanReceiverPriority);
}

bool CanSlaveInterface::GetCanMove(CanMovementMessage& msg)
{
#if 1
	// See if there is a movement message
	CanMessageBuffer *buf;
	{
		TaskCriticalSectionLocker lock;

		buf = pendingBuffers;
		if (buf != nullptr)
		{
			pendingBuffers = buf->next;
		}
	}

	if (buf != nullptr)
	{
		msg = buf->msg;
		CanMessageBuffer::Free(buf);
		return true;
	}
	return false;
#else
	static bool running = false;
	static bool forwards;

	static const int32_t steps1 = 6 * 200 * 16;

	if (!running)
	{
		forwards = true;
		msg.accelerationClocks = StepTimer::StepClockRate/2;
		msg.steadyClocks = StepTimer::StepClockRate * 2;
		msg.decelClocks = StepTimer::StepClockRate/2;
		msg.initialSpeedFraction = 0.0;
		msg.finalSpeedFraction = 0.0;

		msg.flags.deltaDrives = 0;
		msg.flags.endStopsToCheck = 0;
		msg.flags.pressureAdvanceDrives = 0;
		msg.flags.stopAllDrivesOnEndstopHit = false;

		msg.initialX = 0.0;			// only relevant for delta moves
		msg.initialY = 0.0;			// only relevant for delta moves
		msg.finalX = 1.0;			// only relevant for delta moves
		msg.finalY = 1.0;			// only relevant for delta moves
		msg.zMovement = 0.0;		// only relevant for delta moves

		msg.moveStartTime = StepTimer::GetInterruptClocks() + StepTimer::StepClockRate/100;		// start the move 10ms from now
		running = true;
	}
	else
	{
		if (msg.steadyClocks >= StepTimer::StepClockRate * 4)
		{
			msg.steadyClocks = 0;
		}
		else
		{
			msg.steadyClocks += StepTimer::StepClockRate;
		}
		// We can leave all the other parameters the same
		msg.moveStartTime += msg.accelerationClocks + msg.steadyClocks + msg.decelClocks;
	}

	msg.perDrive[0].steps = (forwards) ? steps1 : -steps1;
	msg.perDrive[1].steps = msg.perDrive[0].steps/2;
	msg.perDrive[2].steps = msg.perDrive[0].steps/4;
	msg.timeNow = StepTimer::GetInterruptClocks();
	forwards = !forwards;

	return true;
#endif
}

// This is called from the step ISR when the move is stopped by the Z probe
void CanSlaveInterface::MoveStoppedByZProbe()
{
	//TODO
}

// End
