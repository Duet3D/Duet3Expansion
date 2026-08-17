/*
 * TaskPriorities.h
 *
 *  Created on: 21 Nov 2020
 *      Author: David
 */

#ifndef SRC_TASKPRIORITIES_H_
#define SRC_TASKPRIORITIES_H_

#include <FreeRTOSConfig.h>

// Task priorities. These must all be less than configMAX_PRIORITIES defined in FreeRTOSConfig.g which is usually 8.
namespace TaskPriority
{
	constexpr unsigned int SpinPriority = 1;						// priority for tasks that rarely block
	constexpr unsigned int Led = 1;
	constexpr unsigned int HeatPriority = 2;
	constexpr unsigned int UsbPriority = 2;
	constexpr unsigned int TmcOpenLoop = 2;							// priority of the TMC task when in open loop modes
	constexpr unsigned int AinPriority = 2;
	constexpr unsigned int MfmNormal = 2;							// priority of the MFM task if we have an embedded AS5601
	constexpr unsigned int CanReceiverPriority = 3;
	constexpr unsigned int MovePriority = 3;
	constexpr unsigned int Accelerometer = 3;
	constexpr unsigned int ClosedLoopDataTransmission = 3;
	constexpr unsigned int TmcClosedLoop = 4;						// priority of the TMC task when in closed loop mode
	constexpr unsigned int MfmHigh = 5;								// priority of the MFM task if we have an embedded AS5601 while it is simulating an interrupt
	constexpr unsigned int CanAsyncSenderPriority = 5;
	constexpr unsigned int CanClockPriority = 5;
	constexpr unsigned int LdcTask = 6;								// priority of the AnalogIn task when it is reading the LDC1612
	constexpr unsigned int Ads131M02 = 6;							// priority of the task that handles the load sensor ADC
	constexpr unsigned int InductiveHeaterCalibration = 3;

	// Assert that the highest priority one isn't too high
	static_assert(CanClockPriority < configMAX_PRIORITIES);
}

#endif /* SRC_TASKPRIORITIES_H_ */
