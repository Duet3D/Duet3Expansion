/*
 * LIS3DH.cpp
 *
 *  Created on: 14 Mar 2021
 *      Author: David
 */

#include "LIS3DH.h"

#if SUPPORT_I2C_SENSORS && SUPPORT_LIS3DH

#include <Hardware/IoPorts.h>
#include <Movement/StepTimer.h>

constexpr uint16_t Lis3dAddress = 0b0011000;						// bottom bit is 1 if SDO/SA0 connected to Vcc, 0 if connected to ground
constexpr uint32_t Lis3dI2CTimeout = 25;
constexpr uint8_t FifoInterruptLevel = 24;							// how full the FIFO must get before we want an interrupt

static constexpr uint8_t WhoAmIValue = 0x33;

LIS3DH::LIS3DH(SharedI2CMaster& dev, Pin p_int1Pin, bool addressLSB) noexcept
	: SharedI2CClient(dev, (addressLSB) ? Lis3dAddress | 0x0001 : Lis3dAddress), int1Pin(p_int1Pin)
{
}

// Do a quick test to check whether the accelerometer is present, returning true if it is
bool LIS3DH::CheckPresent() noexcept
{
	uint8_t val;
	return ReadRegister(LisRegister::WhoAmI, val) && val == WhoAmIValue;
}

uint8_t LIS3DH::ReadStatus() noexcept
{
	uint8_t val;
	return (ReadRegister(LisRegister::Status, val)) ? val : 0xFF;
}

// Configure the accelerometer to collect for the requested axis at or near the requested sampling rate and the requested resolution in bits.
// Actual collection does not start until the first call to Collect8bitData or Collect16bitData.
void LIS3DH::Configure(uint16_t& samplingRate, uint8_t& resolution) noexcept
{
	// Set up control registers 1 and 4 according to the selected sampling rate and resolution
	ctrlReg1 = 0;													// collect no axes for now
	uint8_t ctrlReg4 = (1 << 7);									// set block data update
	if (resolution >= 12)											// if high resolution mode
	{
		resolution = 12;
		ctrlReg4 |= (1 << 3);										// set HR
	}
	else if (resolution < 10)										// if low res mode
	{
		resolution = 8;
		ctrlReg1 |= (1 << 3);										// set LP
	}
	else
	{
		resolution = 10;
	}

	uint8_t odr;
	if (samplingRate >= 1000)
	{
		if (resolution >= 10)
		{
			odr = 0x9;												// in 10- or 12-bit mode, so select 1.344kHz (next lowest is 400Hz)
			samplingRate = 1344;
		}
		else if (samplingRate >= 5000)
		{
			odr = 0x9;												// select 5.376kHz
			samplingRate = 5376;
		}
		else
		{
			odr = 0x8;												// select 1.6kHz
			samplingRate = 1600;
		}
	}
	else if (samplingRate >= 400)
	{
		odr = 0x7;
		samplingRate = 400;
	}
	else if (samplingRate >= 200)
	{
		odr = 0x6;
		samplingRate = 200;
	}
	else if (samplingRate >= 100)
	{
		odr = 0x5;
		samplingRate = 100;
	}
	else if (samplingRate >= 50)
	{
		odr = 0x4;
		samplingRate = 500;
	}
	else if (samplingRate >= 25)
	{
		odr = 0x3;
		samplingRate = 25;
	}
	else if (samplingRate >= 10)
	{
		odr = 0x2;
		samplingRate = 10;
	}
	else
	{
		odr = 0x1;
		samplingRate = 1;
	}

	ctrlReg1 |= (odr << 4);

	// Set up the control registers, except set ctrlReg1 to 0 to select power down mode
	dataBuffer[0] = 0;							// ctrlReg1: for now select power down mode
	dataBuffer[1] = 0;							// ctrlReg2: high pass filter not used
	dataBuffer[2] = (1u << 2);					// ctrlReg3: enable fifo watermark interrupt
	dataBuffer[3] = ctrlReg4;
	dataBuffer[4] = (1u << 6);					// ctrlReg5: enable fifo
	dataBuffer[5] = 0;							// ctrlReg6: INT2 disabled, active high interrupts
	WriteRegisters(LisRegister::Ctrl1, dataBuffer, 6);

	// Set the fifo mode
	WriteRegister(LisRegister::FifoControl, (2u << 6) | (FifoInterruptLevel - 1));
}

void Int1Interrupt(CallbackParameter p) noexcept;		// forward declaration

// Start collecting data
void LIS3DH:: StartCollecting(uint8_t axes) noexcept
{
	ctrlReg1 |= (axes & 7);

	// Clear the fifo
	uint8_t val;
	while (ReadRegister(LisRegister::FifoSource, val) && (val & (1u << 5)) == 0)		// while fifo not empty
	{
		ReadRegisters(LisRegister::OutXL, dataBuffer, 6);
	}

#ifdef TOOL1LC
	// The experimental setup needs the pullup resistor enabled on the interrupt pin
	pinMode(int1Pin, INPUT_PULLUP);
#endif

	AtomicCriticalSectionLocker lock;
	WriteRegister(LisRegister::Ctrl1, ctrlReg1);
	lastInterruptTime = StepTimer::GetTimerTicks();
	attachInterrupt(int1Pin, Int1Interrupt, InterruptMode::rising, this);
	numLastRead = 0;
}

// Collect some 8-bit data from the FIFO, suspending until the data is available
unsigned int LIS3DH::CollectData(const uint16_t **collectedData, uint16_t &dataRate, bool &overflowed) noexcept
{
	// Wait until we have some data
	taskWaiting = TaskBase::GetCallerTaskHandle();
	while (!digitalRead(int1Pin))
	{
		TaskBase::Take();
	}
	taskWaiting = nullptr;

	// Get the fifo status to see how much data we can read and whether the fifo overflowed
	uint8_t fifoStatus;
	if (!ReadRegister(LisRegister::FifoSource, fifoStatus))
	{
		return 0;
	}

	uint8_t numToRead = fifoStatus & 0x1F;
	if (numToRead == 0 && (fifoStatus & 0x20) == 0)
	{
		numToRead = 32;
	}

	if (numToRead != 0)
	{
		// Read the data
		if (!ReadRegisters(LisRegister::OutXL, dataBuffer, 6 * numToRead))
		{
			return 0;
		}

		*collectedData = reinterpret_cast<const uint16_t*>(dataBuffer);
		overflowed = (fifoStatus & 0x40) != 0;
		dataRate = (lastInterruptInterval == 0) ? 0
					: (numLastRead * StepTimer::StepClockRate)/lastInterruptInterval;
		numLastRead = numToRead;
	}
	return numToRead;
}

// Stop collecting data
void LIS3DH::StopCollecting() noexcept
{
	WriteRegister(LisRegister::Ctrl1, 0);
}

bool LIS3DH::ReadRegisters(LisRegister reg, uint8_t *buffer, size_t numToRead) noexcept
{
	return Transfer((uint8_t)reg | 0x80, buffer, 1, numToRead, Lis3dI2CTimeout);
}

bool LIS3DH::WriteRegisters(LisRegister reg, uint8_t *buffer, size_t numToWrite) noexcept
{
	if ((uint8_t)reg < 0x1E)						// don't overwrite the factory calibration values
	{
		return false;
	}
	return Transfer((uint8_t)reg | 0x80, buffer, 1 + numToWrite, 0, Lis3dI2CTimeout);
}

bool LIS3DH::ReadRegister(LisRegister reg, uint8_t& val) noexcept
{
	return Transfer((uint8_t)reg, &val, 1, 1, Lis3dI2CTimeout);
}

bool LIS3DH::WriteRegister(LisRegister reg, uint8_t val) noexcept
{
	if ((uint8_t)reg < 0x1E)						// don't overwrite the factory calibration values
	{
		return false;
	}
	return Transfer((uint8_t)reg, &val, 2, 0, Lis3dI2CTimeout);
}

void LIS3DH::Int1Isr() noexcept
{
	const uint32_t now = StepTimer::GetTimerTicks();
	lastInterruptInterval = now - lastInterruptTime;
	lastInterruptTime = now;
	TaskBase::GiveFromISR(taskWaiting);
	taskWaiting = nullptr;
}

void Int1Interrupt(CallbackParameter p) noexcept
{
	static_cast<LIS3DH*>(p.vp)->Int1Isr();
}

#endif

// End
