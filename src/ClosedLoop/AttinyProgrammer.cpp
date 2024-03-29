/*
 * AttinyProgrammer.cpp
 *
 *  Created on: 10 Aug 2020
 *      Author: David
 */

#include "AttinyProgrammer.h"

#if SUPPORT_CLOSED_LOOP && defined(EXP1HCE)

#include <Hardware/SharedSpiClient.h>

#include <Hardware/IoPorts.h>
#include <Hardware/SharedSpiDevice.h>
#include <ClosedLoop/ClosedLoop.h>
#include <Platform.h>

/* The quadrature decoder is an attiny44.
 * At startup it reads the QuadratureErrorOut pin (which is also MOSI on the SPI bus) with the pullup resistor enabled.
 * - If it reads low, it does nothing and doesn't drive its 3 output pins.
 * - If it reads high, it turns transitions on the A and B quadrature inputs into up and down pulses.
 */

static constexpr uint32_t Attiny44aSignature = 0x1E9207;
static constexpr uint32_t Attiny44aPageSize = 64;				// flash page size in bytes (32 words)

//TODO use the attiny watchdog
static const uint8_t AttinyProgram[] =
{
	// 0x0000 Interrupt vectors (RJMP instructions). First one jumps to CRT start, others jump to dummy ISR.
	0x10, 0xC0, 0x22, 0xC0, 0x21, 0xC0, 0x20, 0xC0, 0x1F, 0xC0, 0x1E, 0xC0, 0x1D, 0xC0, 0x1C, 0xC0,
	0x1B, 0xC0, 0x1A, 0xC0, 0x19, 0xC0, 0x18, 0xC0, 0x17, 0xC0, 0x16, 0xC0, 0x15, 0xC0, 0x14, 0xC0,
	0x13, 0xC0,

	// 0x0022 CRT (init stack and copy lookup table to RAM)
	0x11, 0x24, 0x1F, 0xBE, 0xCF, 0xE5, 0xD1, 0xE0, 0xDE, 0xBF, 0xCD, 0xBF, 0x10, 0xE0, 0xA0, 0xE6,
	0xB0, 0xE0, 0xE2, 0xE8, 0xF0, 0xE0, 0x02, 0xC0, 0x05, 0x90, 0x0D, 0x92, 0xA0, 0x37, 0xB1, 0x07,
	0xD9, 0xF7, 0x02, 0xD0, 0x1B, 0xC0,

	// 0x0048 dummy ISR for all interrupt vectors
	0xDB, 0xCF,

	// 0x004a main()
	0x8A, 0xEC, 0x8B, 0xBB, 0x80, 0xE1,
	0x00, 0x00, 0x81, 0x50, 0xE9, 0xF7, 0xCE, 0x9B, 0xFF, 0xCF, 0x8A, 0xE8, 0x8B, 0xBB, 0x80, 0xE7,
	0x8A, 0xBB, 0x85, 0xEF, 0x81, 0xB9, 0x89, 0xB3,

	// Main processing loop, 11 instructions
	0xE8, 0x2F, 0xE6, 0x95, 0x1B, 0xBA, 0x89, 0xB3,
	0xE8, 0x2B, 0xF0, 0xE0, 0xE0, 0x5A, 0xFF, 0x4F, 0x90, 0x81, 0x9B, 0xBB, 0xF5, 0xCF,

	// 0x007E exit()
	0xF8, 0x94, 0xFF, 0xCF,

	// Lookup table (gets copied to RAM)
	0x8A, 0x9A, 0xAA, 0x8A, 0xAA, 0xCA, 0xCA, 0x9A, 0x9A, 0xCA, 0xCA, 0xAA, 0x8A, 0xAA, 0x9A, 0x8A
};

static const uint8_t AttinyFuses[3] =
{
	0x80,			// low fuse: external clock, not divided by 8
	0xDF,			// high fuse: serial programming enabled
	0xFF			// extended fuse: self programming disabled
};

AttinyProgrammer::AttinyProgrammer(SharedSpiDevice& spiDev) noexcept : spi(spiDev, 125000, SpiMode::mode0, false), programStatus(AttinyProgErrorCode::notChecked)
{
}

// Send a 4-byte command to the attiny and return the last byte received, or the 3rd byte if it is the enter programming mode command
uint8_t AttinyProgrammer::SendSpiQuad(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4) noexcept
{
	const uint8_t packet[4] = { b1, b2, b3, b4 };
	uint8_t reply[4];
	spi.TransceivePacket(packet, reply, 4);
	return (b1 == 0xAC && b2 == 0x53) ? reply[2] : reply[3];
}

// Set up the SPI channel and put the attiny in programming mode, returning true if successful
AttinyProgErrorCode AttinyProgrammer::SetupForProgramming() noexcept
{
	IoPort::SetPinMode(EncoderCsPin, OUTPUT_HIGH);					// make sure any attached encoder doesn't respond to SPI commands
	ClosedLoop::EnableEncodersSpi();
	spi.InitMaster();					// this forces SCLK low because we selected mode 0
	if (!spi.Select(10))				// this sets the mode and baud rate (and would activate CS if we were using one)
	{
		return AttinyProgErrorCode::spiBusy;
	}

	delayMicroseconds(100);					// let SCLK settle to low

	// SCK wasn't forced to zero during power up, so we must pulse RESET
	bool success = false;
	for (int i = 0; i < 3 && !success; ++i)
	{
		pinMode(QuadratureResetPin, OUTPUT_HIGH);
		delayMicroseconds(100);
		digitalWrite(QuadratureResetPin, false);
		delay(30);							// wait at least 20msec
		success = SendSpiQuad(0xAC, 0x53, 0x00, 0x00) == 0x53;
	}

	if (!success)
	{
		return AttinyProgErrorCode::cantEnterProgrammingMode;
	}

	deviceSignature = ((uint32_t)SendSpiQuad(0x30, 0x00, 0x00, 0x00) << 16)
					| ((uint32_t)SendSpiQuad(0x30, 0x00, 0x01, 0x00) << 8)
					| SendSpiQuad(0x30, 0x00, 0x02, 0x00);
	return (deviceSignature == Attiny44aSignature) ? AttinyProgErrorCode::good : AttinyProgErrorCode::badDeviceId;
}

void AttinyProgrammer::EndProgramming() noexcept
{
	spi.Deselect();
	ClosedLoop::DisableEncodersSpi();
	digitalWrite(QuadratureResetPin, true);
}

// Wait for the current programming command to complete, returning true if success
bool AttinyProgrammer::WaitUntilAttinyReady() noexcept
{
	const uint32_t startTime = millis();
	do
	{
		if ((SendSpiQuad(0xF0, 0x00, 0x00, 0x00) & 1u) == 0)
		{
			return true;
		}
	} while (millis() - startTime < 20);			// the longest command (chip erase) should take no more than 9ms
	return false;
}

void AttinyProgrammer::TurnAttinyOff() noexcept
{
	IoPort::SetPinMode(EncoderCsPin, OUTPUT_HIGH);				// make sure any attached encoder doesn't respond to data on the SPI bus
	IoPort::SetPinMode(QuadratureResetPin, OUTPUT_LOW);			// put the attiny in reset
	IoPort::SetPinMode(QuadratureErrorOutPin, OUTPUT_LOW);		// set the error out pin low
	delayMicroseconds(100);										// give the reset enough time
	IoPort::SetPinMode(QuadratureResetPin, OUTPUT_HIGH);		// release the reset
	delayMicroseconds(100);										// give the attiny time to read the error out pin and go into a loop doing nothing
}

// Verify the programming of the attiny
AttinyProgErrorCode AttinyProgrammer::DoVerify() noexcept
{
	AttinyProgErrorCode ret = AttinyProgErrorCode::good;
	for (size_t addr = 0; ret == AttinyProgErrorCode::good && addr < ARRAY_SIZE(AttinyProgram); ++addr)
	{
		const uint8_t progByte = SendSpiQuad(0x20 | ((addr & 1u) << 3), addr >> 9, addr >> 1, 0);
		if (progByte != AttinyProgram[addr])
		{
			ret = AttinyProgErrorCode::verifyFailed;
		}
	}

	if (ret == AttinyProgErrorCode::good)
	{
		if (   SendSpiQuad(0x50, 0x00, 0x00, 0x00) != AttinyFuses[0]
			|| SendSpiQuad(0x58, 0x08, 0x00, 0x00) != AttinyFuses[1]
			|| SendSpiQuad(0x50, 0x08, 0x00, 0x00) != AttinyFuses[2]
		   )
		{
			ret = AttinyProgErrorCode::fuseVerifyFailed;
		}
	}

	return ret;
}

// Check that the decoder is running current firmware, return true if yes
AttinyProgErrorCode AttinyProgrammer::CheckProgram() noexcept
{
	AttinyProgErrorCode ret = SetupForProgramming();
	if (ret == AttinyProgErrorCode::good)
	{
		ret = DoVerify();
	}

	EndProgramming();
	return ret;
}

void AttinyProgrammer::InitAttiny()
{
	programStatus = CheckProgram();
	if (programStatus == AttinyProgErrorCode::verifyFailed || programStatus == AttinyProgErrorCode::fuseVerifyFailed)
	{
		programStatus = Program();
	}
	if (programStatus == AttinyProgErrorCode::good)
	{
		TurnAttinyOff();
	}
}

// Update the program, return true if successful
AttinyProgErrorCode AttinyProgrammer::Program() noexcept
{
	AttinyProgErrorCode ret = SetupForProgramming();
	if (ret == AttinyProgErrorCode::good)
	{
		SendSpiQuad(0xAC, 0x80, 0x00, 0x00);						// send chip erase
		if (!WaitUntilAttinyReady())
		{
			ret = AttinyProgErrorCode::eraseTimeout;
		}
	}

	// We must program the flash one page at a time. Page size is 2K words on the attiny44a (1K words on attiny24a)
	bool finished = false;
	size_t addr = 0;
	size_t pageStartAddress = 0;
	while (ret == AttinyProgErrorCode::good && !finished)
	{
		SendSpiQuad(0x40 | ((addr & 1u) << 3), addr >> 9, addr >> 1, AttinyProgram[addr]);
		++addr;
		finished = (addr == ARRAY_SIZE(AttinyProgram));
		if (finished || (addr & (Attiny44aPageSize - 1)) == 0)
		{
			SendSpiQuad(0x4c, pageStartAddress >> 9, pageStartAddress >> 1, 0x00);
			pageStartAddress = addr;
			if (!WaitUntilAttinyReady())
			{
				ret = AttinyProgErrorCode::writeTimeout;
			}
		}
	}

	if (ret == AttinyProgErrorCode::good)
	{
		// Program the fuses
		ret = AttinyProgErrorCode::writeTimeout;
		SendSpiQuad(0xAC, 0xA0, 0, AttinyFuses[0]);
		if (WaitUntilAttinyReady())
		{
			SendSpiQuad(0xAC, 0xA8, 0, AttinyFuses[1]);
			if (WaitUntilAttinyReady())
			{
				SendSpiQuad(0xAC, 0xA4, 0, AttinyFuses[2]);
				if (WaitUntilAttinyReady())
				{
					ret = AttinyProgErrorCode::good;
				}
			}
		}
	}

	if (ret == AttinyProgErrorCode::good)
	{
		ret = DoVerify();
	}

	EndProgramming();
	return ret;
}

#endif

// End
