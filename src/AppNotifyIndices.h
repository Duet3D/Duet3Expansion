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

namespace NotifyIndices
{
	constexpr uint32_t I2C = NextAvailableAfterCANlib;
	constexpr uint32_t CanAsyncSender = I2C;
	constexpr uint32_t AccelerometerHardware = I2C + 1;
	constexpr uint32_t AccelerometerDataCollector = I2C + 2;
	constexpr uint32_t Heat = AccelerometerDataCollector;
	constexpr uint32_t Tmc = AccelerometerDataCollector;
	constexpr uint32_t Move = AccelerometerDataCollector;
	constexpr uint32_t ClosedLoopDataTransmission = AccelerometerDataCollector;
	constexpr uint32_t CanMessageQueue = I2C + 3;
	constexpr uint32_t TotalUsed = I2C + 4;
}

#ifdef RTOS
# include <FreeRTOSConfig.h>
static_assert(NotifyIndices::TotalUsed <= configTASK_NOTIFICATION_ARRAY_ENTRIES);
#endif

#endif /* SRC_APPNOTIFYINDICES_H_ */
