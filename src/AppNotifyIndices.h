/*
 * AppNotifyIndices.h
 *
 *  Created on: 2 Jan 2024
 *      Author: David
 *
 *  Definitions of task notification indices used by the application layer
 */

#ifndef SRC_APPNOTIFYINDICES_H_
#define SRC_APPNOTIFYINDICES_H_

#include <CANlibNotifyIndices.h>

// RTOS task notification indices
// Multiple tasks can use the same set of indices.
// Where a task may wait on two or more different events at different times, we should use different indices for those events.
namespace NotifyIndices
{
	constexpr uint32_t CanAsyncSender = NextAvailableAfterCANlib;
	constexpr uint32_t AccelerometerHardware = NextAvailableAfterCANlib + 1;
	constexpr uint32_t AccelerometerDataCollector = NextAvailableAfterCANlib + 2;
	constexpr uint32_t Heat = AccelerometerDataCollector;
	constexpr uint32_t Tmc = AccelerometerDataCollector;
	constexpr uint32_t Move = AccelerometerDataCollector;
	constexpr uint32_t Ads131M02 = AccelerometerDataCollector;
	constexpr uint32_t ClosedLoopDataTransmission = AccelerometerDataCollector;
	constexpr uint32_t InductiveHeaterCalibration = AccelerometerDataCollector;
	constexpr uint32_t CanMessageQueue = NextAvailableAfterCANlib + 3;
	constexpr uint32_t LDC1612 = AccelerometerDataCollector;
	constexpr uint32_t TotalUsed = NextAvailableAfterCANlib + 4;
}

#ifdef RTOS
# include <FreeRTOSConfig.h>
static_assert(NotifyIndices::TotalUsed <= configTASK_NOTIFICATION_ARRAY_ENTRIES);
#endif

#endif /* SRC_APPNOTIFYINDICES_H_ */
