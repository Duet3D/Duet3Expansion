/*
 * AccelerometerHandler.cpp
 *
 *  Created on: 16 Mar 2021
 *      Author: David
 */

#include "AccelerometerHandler.h"

#if SUPPORT_LIS3DH

#include <RTOSIface/RTOSIface.h>
#include <Hardware/LISAccelerometer.h>
#include <CanMessageFormats.h>
#include <Platform/TaskPriorities.h>
#include <Platform/Platform.h>
#include <CanMessageBuffer.h>
#include <CAN/CanInterface.h>
#include <CanMessageGenericParser.h>
#include <CanMessageGenericTables.h>
#include <AppNotifyIndices.h>

#define TEST_PACKING	0

constexpr uint16_t DefaultSamplingRate = 1000;
constexpr uint32_t StartTimeoutMillis = 800;				// must be less than the main board's response timeout (1000ms) or a slow start turns into a CAN response timeout

constexpr size_t AccelerometerTaskStackWords = 200;			// 150 was enough to stop overflowing but left only 13 words free when collecting at 5.4kHz on the SZP
static Task<AccelerometerTaskStackWords> *accelerometerTask;

static LISAccelerometer *accelerometer = nullptr;
static bool present = false;								// note that present => (accelerometer != nullptr)

static uint16_t samplingRate = DefaultSamplingRate;
static volatile uint32_t numSamplesRequested;
static uint8_t resolution = DefaultAccelerometerResolution;
static uint8_t orientation = DefaultAccelerometerOrientation;
static volatile uint8_t axesRequested;
static volatile bool running = false;
static volatile bool successfulStart = false;
static volatile bool failedStart = false;
static uint8_t axisLookup[3];								// mapping from each Cartesian axis to the corresponding accelerometer axis
static bool axisInverted[3];
static IoPort csPort, irqPort;								// port(s) used by the accelerometer, needed when using SPI-connected accelerometers

static uint8_t TranslateAxes(uint8_t axes) noexcept
{
	uint8_t rslt = 0;
	for (unsigned int i = 0; i < 3; ++i)
	{
		if (axes & (1u << i))
		{
			rslt |= 1u << axisLookup[i];
		}
	}
	return rslt;
}

[[noreturn]] void AccelerometerTaskCode(void*) noexcept
{
	for (;;)
	{
		TaskBase::TakeIndexed(NotifyIndices::AccelerometerDataCollector);
		if (running)
		{
			// Collect and send the samples
			CanMessageBuffer buf;
			CanMessageAccelerometerData& msg = *(buf.SetupRequestMessageNoRid<CanMessageAccelerometerData>(CanInterface::GetCanAddress(), CanInterface::GetCurrentMasterAddress()));
			const unsigned int MaxSamplesInBuffer = msg.SetAxesAndResolution(axesRequested, resolution);

			unsigned int samplesSent = 0;
			unsigned int samplesInBuffer = 0;
			unsigned int samplesWanted = numSamplesRequested;
			size_t canDataIndex = 0;
			unsigned int bitsUsed = 0;
			uint16_t bitsPending = 0;
			bool overflowed = false;

#if TEST_PACKING
			uint16_t pattern = 0;
#endif
			if (accelerometer->StartCollecting(TranslateAxes(axesRequested)))
			{
				successfulStart = true;
				do
				{
					uint16_t dataRate;
					const uint16_t *data;
					unsigned int samplesRead = accelerometer->CollectData(&data, dataRate, overflowed);
					if (samplesSent + samplesInBuffer == 0 && samplesRead != 0)
					{
						// The first sample taken after waking up is inaccurate, so discard it
						--samplesRead;
						data += 3;
					}
					if (samplesRead >= samplesWanted)
					{
						samplesRead = samplesWanted;
					}

					while (samplesRead != 0)
					{
						unsigned int samplesToCopy = min<unsigned int>(samplesRead, MaxSamplesInBuffer - samplesInBuffer);
						while (samplesToCopy != 0)
						{
							// Extract the required bits from the data and pack them into the CAN buffer
							for (unsigned int axis = 0; axis < 3; ++axis)
							{
								if (axesRequested & (1u << axis))
								{
									uint16_t dataVal =
#if TEST_PACKING
														pattern++;
#else
														data[axisLookup[axis]];
									if (axisInverted[axis])
									{
										dataVal = (dataVal == 0x8000) ? ~dataVal : ~dataVal + 1;
									}
									dataVal >>= (16u - resolution);								// data from LIS3DH is left justified
#endif
									// Copy data for this axis
									if (resolution == 16u)
									{
										msg.data[canDataIndex++] = dataVal;
									}
									else
									{
										const uint16_t val = dataVal;
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
							}
							data += 3;

							++samplesInBuffer;
							--samplesToCopy;
							--samplesWanted;
							--samplesRead;
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

							buf.dataLength = msg.GetActualDataLength();
							CanInterface::Send(&buf);
							Platform::OnProcessingCanMessage();								// flash the ACT LED

							samplesSent += samplesInBuffer;
							samplesInBuffer = 0;
							canDataIndex = 0;
							overflowed = false;
							bitsUsed = 0;
							bitsPending = 0;
						}
					}
				} while (samplesWanted != 0);
			}
			else
			{
				failedStart = true;
			}

			accelerometer->StopCollecting();

			// Wait for another command
			running = false;
		}
	}
}

static bool TranslateOrientation(uint8_t input) noexcept
{
	if (input >= 70u) { return false; }
	const uint8_t xDigit = input % 10u;
	if (xDigit >= 7u) { return false; }
	const uint8_t zDigit = input / 10u;
	const uint8_t xOrientation = xDigit & 0x03;
	const uint8_t zOrientation = zDigit & 0x03;
	if (xOrientation == 3u || zOrientation == 3u || xOrientation == zOrientation) { return false; }
	const uint8_t xInverted = xDigit & 0x04;
	const uint8_t zInverted = zDigit & 0x04;
	uint8_t yInverted = xInverted ^ zInverted;

	// Calculate the Y orientation. We must have axes 0, 1 and 2 so they must add up to 3.
	const uint8_t yOrientation = 3u - xOrientation - zOrientation;

	// The total number of inversions must be even if the cyclic order of the axes is 012, odd if it is 210 (can we prove this?)
	if ((xOrientation + 1) % 3 != yOrientation)
	{
		yInverted ^= 0x04;									// we need an odd number of axis inversions
	}

	// Now fill in the axis table
	axisLookup[xOrientation] = 0;
	axisInverted[xOrientation] = xInverted;
	axisLookup[yOrientation] = 1;
	axisInverted[yOrientation] = yInverted;
	axisLookup[zOrientation] = 2;
	axisInverted[zOrientation] = zInverted;
	return true;
}

// Interface functions called by the main task

// Initialise the accelerometer returning true if succeeded
#if ACCELEROMETER_USES_SPI
bool AccelerometerHandler::Init(SharedSpiDevice& dev, uint32_t freq, Pin csPin, Pin int1Pin) noexcept
#else
bool AccelerometerHandler::Init(SharedI2CMaster& dev) noexcept
#endif
{
#if ACCELEROMETER_USES_SPI
	accelerometer = new LISAccelerometer(dev, freq, csPin, int1Pin);
#else
	accelerometer = new LISAccelerometer(dev, Lis3dhInt1Pin);
#endif
	if (accelerometer->CheckPresent())
	{
		accelerometer->Configure(samplingRate, resolution);
		present = true;
		(void)TranslateOrientation(orientation);
		if (accelerometerTask == nullptr)
		{
			accelerometerTask = new Task<AccelerometerTaskStackWords>;
			accelerometerTask->Create(AccelerometerTaskCode, "ACCEL", nullptr, TaskPriority::Accelerometer);
		}
		return true;
	}

#ifdef TOOL1LC
	// The accelerometer should definitely be present. We will try to communicate with it at intervals to assist with hardware debugging.
	// So don't delete the accelerometer object, but don't set the 'present' flag either.
#else
	DeleteObject(accelerometer);
#endif
	present = false;
	return false;
}

bool AccelerometerHandler::IsPresent() noexcept
{
	return present;
}

bool AccelerometerHandler::IsCollecting() noexcept
{
	return running;
}

// The rate and resolution the accelerometer was actually programmed for, which may be lower than the ones requested
uint16_t AccelerometerHandler::GetSamplingRate() noexcept
{
	return samplingRate;
}

uint8_t AccelerometerHandler::GetResolution() noexcept
{
	return resolution;
}

// Process a request to configure the accelerometer.
// Note, the device number is passed to us fore historical reasons, but we no longer use it.
// Translate the orientation from a 2-digit number to translation tables, returning true if successful, false if bad orientation
GCodeResult AccelerometerHandler::ProcessConfigRequest(const CanMessageGeneric& msg, const StringRef &reply) noexcept
{
	CanMessageGenericParser parser(msg, M955Params);

	String<StringLength50> pinNames;
	if (parser.GetStringParam('C', pinNames.GetRef()))
	{
		if (running)
		{
			reply.printf("Accelerometer on board %u is busy collecting data", CanInterface::GetCanAddress());
			return GCodeResult::error;
		}

		csPort.Release();
		irqPort.Release();
		IoPort *const ports[] = { &csPort, &irqPort };
		PinAccess access[] = { PinAccess::write1, PinAccess::read };
		const size_t numPortsFound = IoPort::AssignPorts(pinNames.c_str(), reply, PinUsedBy::sensor, 2, ports, access);
		if (numPortsFound == 0)
		{
			return GCodeResult::error;
		}
#if ACCELEROMETER_USES_SPI
		if (numPortsFound != 2)
		{
			csPort.Release();
			irqPort.Release();
			reply.copy("SPI-connected accelerometer requires CS and IRQ pins but only one pin provided");
			return GCodeResult::error;
		}
		uint32_t clockSpeed = DefaultAccelerometerSpiFrequency;
		(void)parser.GetUintParam('Q', clockSpeed);
		if (!Init(Platform::GetSharedSpi(), clockSpeed, csPort.GetPin(), irqPort.GetPin()))
		{
			csPort.Release();
			irqPort.Release();
			reply.printf("Failed to initialise accelerometer on board %u using pins %s", CanInterface::GetCanAddress(), pinNames.c_str());
			return GCodeResult::error;
		}
#else
		// For I2C-connected accelerometers the only valid port is "i2c.lis" or one of its aliases
		if (numPortsFound != 1 || csPort.GetPin() != LisPinNumber)
		{
			csPort.Release();
			irqPort.Release();
			reply.copy("I2C-connected accelerometer can only use i2c.lis port");
			return GCodeResult::error;
		}
#endif
		uint8_t localOrientation;
		if (parser.GetUintParam('I', localOrientation))
		{
			if (TranslateOrientation(localOrientation))
			{
				orientation = localOrientation;
			}
			else
			{
				reply.copy("Invalid orientation");
				return GCodeResult::error;
			}
		}
		else
		{
			orientation = DefaultAccelerometerOrientation;
			(void)TranslateOrientation(orientation);
		}

		(void)parser.GetUintParam('S', samplingRate);
		(void)parser.GetUintParam('R', resolution);

		if (!accelerometer->Configure(samplingRate, resolution))
		{
			reply.copy("Failed to configure accelerometer");
			return GCodeResult::error;
		}
	}
	else if (!present)
	{
		reply.printf("Accelerometer on board %u not configured", CanInterface::GetCanAddress());
		return GCodeResult::error;
	}
	else
	{
		reply.printf("Accelerometer on board %u type %s with orientation %u samples at %uHz with %u-bit resolution",
						CanInterface::GetCanAddress(), accelerometer->GetTypeName(), orientation, samplingRate, resolution);
	}
	return GCodeResult::ok;
}

GCodeResult AccelerometerHandler::ProcessStartRequest(const CanMessageStartAccelerometer& msg, const StringRef& reply) noexcept
{
	if (msg.deviceNumber != 0 || !present)
	{
		reply.printf("Accelerometer %u.%u not present", CanInterface::GetCanAddress(), msg.deviceNumber);
		return GCodeResult::error;
	}

	if (running)
	{
		reply.printf("Accelerometer %u.%u is busy collecting data", CanInterface::GetCanAddress(), msg.deviceNumber);
		return GCodeResult::error;
	}

	axesRequested = msg.axes;
	numSamplesRequested = msg.numSamples;
	successfulStart = false;
	failedStart = false;
	running = true;
	accelerometerTask->Give(NotifyIndices::AccelerometerDataCollector);
	const uint32_t startTime = millis();
	do
	{
		delay(5);
		if (successfulStart)
		{
			return GCodeResult::ok;
		}
	} while (!failedStart && millis() - startTime < StartTimeoutMillis);

	reply.copy((failedStart) ? "Failed to start accelerometer data collection" : "Timed out waiting for accelerometer data collection to start");
	if (accelerometer->HasInterruptError())
	{
		reply.cat(": INT1 error");
	}
	return GCodeResult::error;
}

void AccelerometerHandler::Diagnostics(const StringRef& reply) noexcept
{
	reply.lcatf("Accelerometer: %s", (present) ? accelerometer->GetTypeName() : "none");
	if (present)
	{
		const uint8_t status = accelerometer->ReadStatus();
		reply.catf(", status: %02x", status);
		if (accelerometer->HasInterruptError())
		{
			reply.cat(", INT1 error!");
		}
	}
}

#endif

// End
