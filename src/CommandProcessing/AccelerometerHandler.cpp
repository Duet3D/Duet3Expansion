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
static volatile uint8_t axes = 0;

[[noreturn]] void AccelerometerTaskCode(void*) noexcept
{
	for (;;)
	{
		TaskBase::Take();
		if (axes != 0)
		{
			// Set up the accelerometer to get as close to the requested sample rate and resolution as we can
			accelerometer->Configure(samplingRate, resolution);
			CanMessageBuffer buf(nullptr);

			// Collect and send the samples
			unsigned int samplesSent = 0;
			unsigned int samplesInBuffer = 0;
			unsigned int samplesWanted = numSamplesRequested;
			bool overflowed = false;
			if (resolution <= 8)
			{
				do
				{
					uint8_t collectedData[32];					// LIS3DH has a 32-deep FIFO
					uint16_t dataRate;
					unsigned int samplesRead = accelerometer->Collect8bitData(collectedData, dataRate, overflowed);
					if (samplesRead >= samplesWanted)
					{
						samplesRead = samplesWanted;
					}
					constexpr unsigned int MaxSamplesInBuffer = sizeof(CanMessageAccelerometerData::data);
					const unsigned int samplesToCopy = min<unsigned int>(samplesRead, MaxSamplesInBuffer - samplesInBuffer);
					CanMessageAccelerometerData& msg = buf.msg.accelerometerData;
					memcpy(msg.data + samplesInBuffer, collectedData, samplesToCopy);
					samplesWanted -= samplesToCopy;
					samplesInBuffer += samplesToCopy;
					if (samplesInBuffer == MaxSamplesInBuffer || samplesWanted == 0)
					{
						// Send the buffer
						buf.SetupStatusMessage<CanMessageAccelerometerData>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress());
						msg.firstSampleNumber = samplesSent;
						msg.numSamples = samplesInBuffer;
						msg.actualSampleRate = dataRate;
						msg.overflowed = overflowed;
						msg.twelveBit = false;

						buf.dataLength = msg.GetActualDataLength();
						CanInterface::Send(&buf);
						samplesSent += samplesInBuffer;
						overflowed = false;

						if (samplesRead > samplesToCopy)
						{
							samplesInBuffer = samplesRead - samplesToCopy;
							memcpy(msg.data, collectedData + samplesToCopy, samplesInBuffer);
						}
						else
						{
							samplesInBuffer = 0;
						}

					}
				} while (samplesWanted != 0);
			}
			else
			{
				unsigned int canDataIndex = 0;
				do
				{
					uint16_t collectedData[32];					// LIS3DH has a 32-deep FIFO
					uint16_t dataRate;
					unsigned int samplesRead = accelerometer->Collect16bitData(collectedData, dataRate, overflowed);
					if (samplesRead >= samplesWanted)
					{
						samplesRead = samplesWanted;
					}
					constexpr unsigned int MaxSamplesInBuffer = (2 * sizeof(CanMessageAccelerometerData::data))/3;
					const unsigned int samplesToCopy = min<unsigned int>(samplesRead, MaxSamplesInBuffer - samplesInBuffer);
					CanMessageAccelerometerData& msg = buf.msg.accelerometerData;

					// Pack the 12-bit data into the buffer
					const uint16_t *p = collectedData;
					{
						const unsigned int samplesWantedInBuffer = samplesInBuffer + samplesToCopy;
						while (samplesInBuffer < samplesWantedInBuffer)
						{
							const uint16_t val = *p++;
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
						}
					}
					samplesWanted -= samplesToCopy;
					samplesInBuffer += samplesToCopy;
					if (samplesInBuffer == MaxSamplesInBuffer || samplesWanted == 0)
					{
						// Send the buffer
						buf.SetupStatusMessage<CanMessageAccelerometerData>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress());
						msg.firstSampleNumber = samplesSent;
						msg.numSamples = samplesInBuffer;
						msg.actualSampleRate = dataRate;
						msg.overflowed = overflowed;
						msg.twelveBit = false;

						buf.dataLength = msg.GetActualDataLength();
						CanInterface::Send(&buf);
						samplesSent += samplesInBuffer;
						overflowed = false;

						samplesInBuffer = 0;
						if (samplesRead > samplesToCopy)
						{
							const unsigned int samplesWantedInBuffer = samplesRead - samplesToCopy;
							while (samplesInBuffer < samplesWantedInBuffer)
							{
								const uint16_t val = *p++;
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
							}
						}
					}
				} while (samplesWanted != 0);
			}
			accelerometer->Stop();

			// Wait for another command
			axes = 0;
		}
	}
}

// Interface functions called by the main task
void AccelerometerHandler::Init() noexcept
{
	LIS3DH *temp = new LIS3DH(Platform::GetSharedI2C(), Lis3dhAddressLsb);
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
	if (msg.axes != 0 && axes != 0)
	{
		reply.copy("Accelerometer is busy");
		return GCodeResult::error;
	}

	samplingRate = msg.sampleRate;
	numSamplesRequested = msg.numSamples;
	resolution = msg.resolutionBits;
	axes = msg.axes;							// this must be the last one written
	if (axes != 0)
	{
		accelerometerTask.Give();
	}
	return GCodeResult::ok;
}

void AccelerometerHandler::Diagnostics(const StringRef& reply) noexcept
{
	reply.lcatf("Accelerometer detected: %s", (accelerometer != nullptr) ? "yes" : "no");
}

#endif

// End
