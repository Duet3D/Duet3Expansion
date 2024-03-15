/*
 * TaskPriorities.h
 *
 *  Created on: 21 Nov 2020
 *      Author: David
 */

#ifndef SRC_TASKPRIORITIES_H_
#define SRC_TASKPRIORITIES_H_

// Task priorities. These must all be less than configMAX_PRIORITIES defined in FreeRTOSConfig.g.
namespace TaskPriority
{
	static constexpr unsigned int SpinPriority = 1;							// priority for tasks that rarely block
	static constexpr unsigned int HeatPriority = 2;
	static constexpr unsigned int UsbPriority = 2;
	static constexpr unsigned int TmcOpenLoop = 2;							// priority of the TMC task when in open loop modes
	static constexpr unsigned int AinPriority = 2;
	static constexpr unsigned int MfmNormal = 2;							// priority of the MFM task if we have an embedded AS5601
	static constexpr unsigned int CanReceiverPriority = 3;
	static constexpr unsigned int MovePriority = 3;
	static constexpr unsigned int Accelerometer = 3;
	static constexpr unsigned int ClosedLoopDataTransmission = 3;
	static constexpr unsigned int TmcClosedLoop = 4;						// priority of the TMC task when in closed loop mode
	static constexpr unsigned int MfmHigh = 5;								// priority of the MFM task if we have an embedded AS5601 while it is simulating an interrupt
	static constexpr unsigned int CanAsyncSenderPriority = 5;
	static constexpr unsigned int CanClockPriority = 5;

	// Assert that the highest priority one isn't too high
	static_assert(CanClockPriority < configMAX_PRIORITIES);
}

#endif /* SRC_TASKPRIORITIES_H_ */
