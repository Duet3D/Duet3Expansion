/*
 * AccelerometerHandler.cpp
 *
 *  Created on: 16 Mar 2021
 *      Author: David
 */

#include "AccelerometerHandler.h"

#if SUPPORT_LIS3DH

#include <RTOSIface/RTOSIface.h>
#include <Hardware/LIS3DH.h>
#include <CanMessageFormats.h>
#include <Platform.h>

constexpr size_t AccelerometerTaskStackWords = 130;
static Task<AccelerometerTaskStackWords> accelerometerTask;

static LIS3DH *accelerometer = nullptr;

volatile uint16_t samplingRate;
volatile uint8_t axes;
volatile int8_t resolution;
volatile uint8_t commandSequenceNumber;
uint8_t oldCommandSequencenumber;

// Interface functions called by the main task
void AccelerometerHandler::Init() noexcept
{
	LIS3DH *temp = new LIS3DH(Platform::GetSharedI2C(), Lis3dhAddressLsb);
	if (temp->CheckPresent())
	{
		accelerometer = temp;
	}
	else
	{
		delete temp;
	}
}

bool AccelerometerHandler::Present() noexcept
{
	return accelerometer != nullptr;
}

GCodeResult AccelerometerHandler::ProcessCanCommand(const CanMessageAccelerometerSettings &msg, const StringRef &reply) noexcept
{
	samplingRate = msg.sampleRate;
	axes = msg.axes;
	resolution = msg.resolutionBits;
	++commandSequenceNumber;					// this signals the CAN task to reprogram the accelerometer
	return GCodeResult::ok;
}

void AccelerometerHandler::Diagnostics(const StringRef& reply) noexcept
{
	reply.lcatf("Accelerometer detected: %s", (accelerometer != nullptr) ? "yes" : "no");
}

#endif

// End
