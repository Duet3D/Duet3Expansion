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
	static constexpr int SpinPriority = 1;							// priority for tasks that rarely block
	static constexpr int HeatPriority = 2;
	static constexpr int DhtPriority = 2;
	static constexpr int TmcPriority = 2;
	static constexpr int AinPriority = 2;
	static constexpr int HeightFollowingPriority = 2;
	static constexpr int DueXPriority = 3;
	static constexpr int LaserPriority = 3;
	static constexpr int CanSenderPriority = 3;
	static constexpr int CanReceiverPriority = 3;
	static constexpr int CanAsyncSenderPriority = 4;
	static constexpr int CanClockPriority = 4;
}

#endif /* SRC_TASKPRIORITIES_H_ */
