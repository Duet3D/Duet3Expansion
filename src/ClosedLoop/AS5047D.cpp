/*
 * AS5047D.cpp
 *
 *  Created on: 10 Jun 2020
 *      Author: David
 */

#include <ClosedLoop/AS5047D.h>

#if SUPPORT_CLOSED_LOOP

#include <Hardware/IoPorts.h>

constexpr uint16_t AS5047RegNop = 0x0000;
constexpr uint16_t AS5047RegErrfl = 0x0001;
constexpr uint16_t AS5047RegProg = 0x0003;
constexpr uint16_t AS5047RegDiag = 0x3FFC;
constexpr uint16_t AS5047RegMag = 0x3FFD;
constexpr uint16_t AS5047RegAngleUnc = 0x3FFE;
constexpr uint16_t AS5047RegAngleCom = 0x3FFF;

constexpr uint16_t AS5047WriteCommand = 0x4000;

// Adjust the top bit of a word to make it even parity
static inline constexpr uint16_t AddParityBit(uint16_t w)
{
	uint16_t p = w ^ (w << 8);
	p ^= p << 4;
	p ^= p << 2;
	p ^= p << 1;
	return w ^ (p & 0x8000);
}

static inline constexpr bool CheckEvenParity(uint16_t w)
{
	w ^= w >> 8;
	w ^= w >> 4;
	w ^= w >> 2;
	w ^= w >> 1;
	return (w & 1u) == 0;
}

AS5047D::AS5047D(SharedSpiDevice& p_spi, Pin p_csPin) : spi(p_spi), csPin(p_csPin)
{
}

void AS5047D::Init()
{
	IoPort::SetPinMode(csPin, OUTPUT_HIGH);
	spi.SetClockFrequencyAndMode(6000000, SpiMode::mode1);
}

int16_t AS5047D::GetAngle()
{
	uint16_t response;
	DoSpiTransaction(AddParityBit(AS5047RegAngleCom), response);
	DoSpiTransaction(AddParityBit(AS5047RegNop), response);

	//TODO how to report an error?
	return (int16_t)(response & 0x3FFF);
}

void AS5047D::Diagnostics(const StringRef &reply)
{
	uint16_t response;
	if (DoSpiTransaction(AddParityBit(AS5047RegDiag), response) && DoSpiTransaction(AddParityBit(AS5047RegNop), response))
	{
		reply.printf("AS5047 agc %u", response & 0x007F);
		if ((response & 0x0100) == 0)
		{
			reply.cat(", offset loop not ready");
		}
		if ((response & 0x0200) != 0)
		{
			reply.cat(", CORDIC overflow");
		}
		if ((response & 0x0400) != 0)
		{
			reply.cat(", magnet too strong");
		}
		if ((response & 0x0800) != 0)
		{
			reply.cat(", magnet too weak");
		}
	}
	else
	{
		reply.copy("Failed to read AS5047 diagnostics");
	}
}

bool AS5047D::DoSpiTransaction(uint16_t command, uint16_t &response)
{
	// We have exclusive access to this SPI so we don't need to get the mutex
	IoPort::WriteDigital(csPin, false);
	delayMicroseconds(1);			// need at least 350ns before the clock
	const bool ok = spi.TransceivePacket(reinterpret_cast<const uint8_t*>(&command), reinterpret_cast<uint8_t*>(&response), 2);
	delayMicroseconds(1);			// need at least half an SPI clock here
	IoPort::WriteDigital(csPin, false);
	return ok && (response & 0x4000) == 0 && CheckEvenParity(response);
}

#endif
