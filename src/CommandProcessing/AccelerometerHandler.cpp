/*
 * AccelerometerHandler.cpp
 *
 *  Created on: 16 Mar 2021
 *      Author: David
 */

#include "AccelerometerHandler.h"

#if SUPPORT_I2C_SENSORS && SUPPORT_LIS3DH

#include <RTOSIface/RTOSIface.h>
#include <Hardware/LIS3DH.h>
#include <CanMessageFormats.h>
#include <Platform.h>
#include <TaskPriorities.h>
#include <CanMessageBuffer.h>
#include <CAN/CanInterface.h>
#include <CanMessageGenericParser.h>

constexpr uint16_t DefaultSamplingRate = 1000;
constexpr uint8_t DefaultResolution = 10;

constexpr size_t AccelerometerTaskStackWords = 130;
static Task<AccelerometerTaskStackWords> accelerometerTask;

static LIS3DH *accelerometer = nullptr;

static uint16_t samplingRate = DefaultSamplingRate;
static volatile uint16_t numSamplesRequested;
static uint8_t resolution = DefaultResolution;
static uint8_t orientation = 0;
static volatile uint8_t axes;
static volatile bool running = false;

[[noreturn]] void AccelerometerTaskCode(void*) noexcept
{
	for (;;)
	{
		TaskBase::Take();
		if (running)
		{
			// Collect and send the samples
			CanMessageBuffer buf(nullptr);
			CanMessageAccelerometerData& msg = *(buf.SetupStatusMessage<CanMessageAccelerometerData>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress()));
			const unsigned int MaxSamplesInBuffer = msg.SetAxesAndResolution(axes, resolution);
			const uint16_t mask = (1u << resolution) - 1u;

			unsigned int samplesSent = 0;
			unsigned int samplesInBuffer = 0;
			unsigned int samplesWanted = numSamplesRequested;
			size_t canDataIndex = 0;
			unsigned int bitsUsed = 0;
			uint16_t bitsPending = 0;
			bool overflowed = false;

			accelerometer->StartCollecting(axes);
			do
			{
				uint16_t dataRate;
				const uint16_t *data;
				unsigned int samplesRead = accelerometer->CollectData(&data, dataRate, overflowed);
				if (samplesRead >= samplesWanted)
				{
					samplesRead = samplesWanted;
				}

				while (samplesRead != 0)
				{
					unsigned int samplesToCopy = min<unsigned int>(samplesRead, MaxSamplesInBuffer - samplesInBuffer);
					unsigned int axis = 0;
					while (samplesToCopy != 0)
					{
						// Extract the required bits from the data and pack them into the CAN buffer
						if (axes & (1 << axis))
						{
							// Copy data for this axis
							if (resolution == 16)
							{
								msg.data[canDataIndex++] = *data;
							}
							else
							{
								const uint16_t val = *data & mask;
								bitsPending |= val << bitsUsed;
								bitsUsed += resolution;
								if (bitsUsed >= 16u)
								{
									msg.data[canDataIndex++] = bitsPending;
									bitsUsed -= 16u;
									bitsPending = val >> (resolution - bitsUsed);
								}
							}
						}
						++axis;
						++data;
						if (axis == 3)
						{
							axis = 0;
							++samplesInBuffer;
							--samplesToCopy;
							--samplesWanted;
							--samplesRead;
						}
					}

					if (samplesInBuffer == MaxSamplesInBuffer || samplesWanted == 0)
					{
						// Send the buffer
						if (bitsUsed != 0)
						{
							msg.data[canDataIndex] = bitsPending;
						}
						msg.firstSampleNumber = samplesSent;
						msg.numSamples = samplesInBuffer;
						msg.actualSampleRate = dataRate;
						msg.overflowed = overflowed;
						msg.lastPacket = (samplesWanted == 0);
						msg.zero = 0;

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
			running = false;
		}
	}
}

// Interface functions called by the main task
void AccelerometerHandler::Init() noexcept
{
	LIS3DH *temp = new LIS3DH(Platform::GetSharedI2C(), Lis3dhInt1Pin, Lis3dhAddressLsb);
	if (temp->CheckPresent())
	{
		temp->Configure(samplingRate, resolution);
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

GCodeResult AccelerometerHandler::ProcessConfigRequest(const CanMessageGeneric& msg, const StringRef &reply) noexcept
{
	CanMessageGenericParser parser(msg, M955Params);
	uint8_t deviceNumber;
	if (!parser.GetUintParam('P', deviceNumber))
	{
		reply.copy("Bad M955 message");
		return GCodeResult::error;
	}
	if (deviceNumber != 0 || accelerometer == nullptr)
	{
		reply.printf("Accelerometer %u.%u not present", CanInterface::GetCanAddress(), deviceNumber);
		return GCodeResult::error;
	}

	if (running)
	{
		reply.printf("Accelerometer %u.%u is busy collecting data", CanInterface::GetCanAddress(), deviceNumber);
		return GCodeResult::error;
	}

	bool seen = parser.GetUintParam('I', orientation);
	if (parser.GetUintParam('S', samplingRate)) { seen = true; }
	if (parser.GetUintParam('R', resolution))  { seen = true; }

	if (seen)
	{
		accelerometer->Configure(samplingRate, resolution);
	}

	reply.printf("Accelerometer %u:%u samples at %uHz with %u-bit resolution", CanInterface::GetCanAddress(), deviceNumber, samplingRate, resolution);
	return GCodeResult::ok;
}

GCodeResult AccelerometerHandler::ProcessStartRequest(const CanMessageStartAccelerometer& msg, const StringRef& reply) noexcept
{
	if (msg.deviceNumber != 0 || accelerometer == nullptr)
	{
		reply.printf("Accelerometer %u.%u not present", CanInterface::GetCanAddress(), msg.deviceNumber);
		return GCodeResult::error;
	}

	if (running)
	{
		reply.printf("Accelerometer %u.%u is busy collecting data", CanInterface::GetCanAddress(), msg.deviceNumber);
		return GCodeResult::error;
	}

	axes = msg.axes;
	numSamplesRequested = msg.numSamples;
	running = true;
	accelerometerTask.Give();
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
