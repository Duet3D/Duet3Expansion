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
	static constexpr unsigned int SpinPriority = 1;							// priority for tasks that rarely block
	static constexpr unsigned int HeatPriority = 2;
	static constexpr unsigned int TmcOpenLoop = 2;							// priority of the TMC task when in open loop modes
	static constexpr unsigned int AinPriority = 2;
	static constexpr unsigned int CanReceiverPriority = 3;
	static constexpr unsigned int MovePriority = 3;
	static constexpr unsigned int Accelerometer = 3;
	static constexpr unsigned int ClosedLoopDataTransmission = 3;
	static constexpr unsigned int TmcClosedLoop = 4;						// priority of the TMC task when in closed loop mode
	static constexpr unsigned int CanAsyncSenderPriority = 5;
	static constexpr unsigned int CanClockPriority = 5;
}

#endif /* SRC_TASKPRIORITIES_H_ */
