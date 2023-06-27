/*
 * TaskPriorities.h
 *
 *  Created on: 21 Nov 2020
 *      Author: David
 */

#ifndef SRC_TASKPRIORITIES_H_
#define SRC_TASKPRIORITIES_H_

// Task priorities
namespace TaskPriority
{
	constexpr unsigned int SpinPriority = 1;						// priority for tasks that rarely block
	constexpr unsigned int HeatPriority = 2;
	constexpr unsigned int TmcOpenLoop = 2;							// priority of the TMC task when in open loop modes
	constexpr unsigned int AinPriority = 2;
	constexpr unsigned int CanReceiverPriority = 3;
	constexpr unsigned int MovePriority = 3;
	constexpr unsigned int Accelerometer = 3;
	constexpr unsigned int ClosedLoopDataTransmission = 3;
	constexpr unsigned int TmcClosedLoop = 4;						// priority of the TMC task when in closed loop mode
	constexpr unsigned int CanAsyncSenderPriority = 5;
	constexpr unsigned int CanClockPriority = 5;

	static_assert(CanClockPriority < configMAX_PRIORITIES);
}

#endif /* SRC_TASKPRIORITIES_H_ */
