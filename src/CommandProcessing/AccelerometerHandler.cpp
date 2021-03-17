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
#include <TaskPriorities.h>
#include <CanMessageBuffer.h>
#include <CAN/CanInterface.h>

constexpr size_t AccelerometerTaskStackWords = 130;
static Task<AccelerometerTaskStackWords> accelerometerTask;

static LIS3DH *accelerometer = nullptr;

static volatile uint16_t samplingRate;
static volatile uint16_t numSamplesRequested;
static volatile uint8_t resolution;
static volatile uint8_t axis;

[[noreturn]] void AccelerometerTaskCode(void*) noexcept
{
	for (;;)
	{
		TaskBase::Take();
		if (resolution != 0)
		{
			// Set up the accelerometer to get as close to the requested sample rate and resolution as we can
			accelerometer->Configure(axis, samplingRate, resolution);
			CanMessageBuffer buf(nullptr);

			// Collect and send the samples
			unsigned int samplesSent = 0;
			unsigned int samplesInBuffer = 0;
			unsigned int samplesWanted = numSamplesRequested;
			const unsigned int MaxSamplesInBuffer = (resolution <= 8) ? sizeof(CanMessageAccelerometerData::data) : (2 * sizeof(CanMessageAccelerometerData::data))/3;
			unsigned int canDataIndex = 0;
			bool overflowed = false;
			do
			{
				uint16_t dataRate;
				const uint16_t *data;
				unsigned int samplesRead = accelerometer->CollectData(&data, dataRate, overflowed);
				data += 2 * axis;
				if (samplesRead >= samplesWanted)
				{
					samplesRead = samplesWanted;
				}

				while (samplesRead != 0)
				{
					unsigned int samplesToCopy = min<unsigned int>(samplesRead, MaxSamplesInBuffer - samplesInBuffer);
					CanMessageAccelerometerData& msg = buf.msg.accelerometerData;
					if (resolution <= 8)
					{
						// Send individual bytes in the CAN message
						while (samplesToCopy != 0)
						{
							msg.data[samplesInBuffer++] = (uint8_t)*data;
							data += 6;
							--samplesToCopy;
							--samplesRead;
							--samplesWanted;
						}
					}
					else
					{
						// Pack the 12-bit data into the buffer
						while (samplesToCopy != 0)
						{
							const uint16_t val = *data;
							data += 6;
							if (samplesInBuffer & 1)
							{
								msg.data[canDataIndex++] |= (uint8_t)(val << 4);
								msg.data[canDataIndex++] = (uint8_t)(val >> 4);
							}
							else
							{
								msg.data[canDataIndex++] = (uint8_t)val;
								msg.data[canDataIndex] = (uint8_t)((val >> 8) & 0x0F);
							}
							++samplesInBuffer;
							--samplesToCopy;
							--samplesWanted;
							--samplesRead;
						}
					}

					if (samplesInBuffer == MaxSamplesInBuffer || samplesWanted == 0)
					{
						// Send the buffer
						buf.SetupStatusMessage<CanMessageAccelerometerData>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress());
						msg.firstSampleNumber = samplesSent;
						msg.numSamples = samplesInBuffer;
						msg.actualSampleRate = dataRate;
						msg.overflowed = overflowed;
						msg.twelveBit = (resolution >= 8);

						buf.dataLength = msg.GetActualDataLength();
						CanInterface::Send(&buf);
						samplesSent += samplesInBuffer;
						samplesInBuffer = 0;
						canDataIndex = 0;
						overflowed = false;
					}
				}
			} while (samplesWanted != 0);

			accelerometer->StopCollecting();

			// Wait for another command
			resolution = 0;
		}
	}
}

// Interface functions called by the main task
void AccelerometerHandler::Init() noexcept
{
	LIS3DH *temp = new LIS3DH(Platform::GetSharedI2C(), Lis3dhInt1Pin, Lis3dhAddressLsb);
	if (temp->CheckPresent())
	{
		accelerometer = temp;
		accelerometerTask.Create(AccelerometerTaskCode, "ACCEL", nullptr, TaskPriority::Accelerometer);
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

GCodeResult AccelerometerHandler::ProcessCanRequest(const CanMessageAccelerometerSettings &msg, const StringRef &reply) noexcept
{
	if (msg.resolution != 0 && resolution != 0)
	{
		reply.copy("Accelerometer is busy");
		return GCodeResult::error;
	}

	samplingRate = msg.sampleRate;
	numSamplesRequested = msg.numSamples;
	axis = msg.axis;
	resolution = msg.resolution;						// this must be the last one written because it controls whether data is collected or not
	if (resolution != 0)
	{
		accelerometerTask.Give();
	}
	return GCodeResult::ok;
}

void AccelerometerHandler::Diagnostics(const StringRef& reply) noexcept
{
	reply.lcatf("Accelerometer detected: %s", (accelerometer != nullptr) ? "yes" : "no");
	if (accelerometer != nullptr)
	{
		reply.catf(", status: %02x", accelerometer->ReadStatus());
	}
}

#endif

// End
