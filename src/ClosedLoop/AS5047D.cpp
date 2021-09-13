/*
 * AS5047D.cpp
 *
 *  Created on: 10 Jun 2020
 *      Author: David
 */

#include <ClosedLoop/AS5047D.h>

#if SUPPORT_CLOSED_LOOP

#include <Hardware/IoPorts.h>
#include <ClosedLoop/ClosedLoop.h>

constexpr uint16_t AS5047RegNop = 0x0000;
constexpr uint16_t AS5047RegErrfl = 0x0001;
constexpr uint16_t AS5047RegProg = 0x0003;
constexpr uint16_t AS5047RegDiag = 0x3FFC;
constexpr uint16_t AS5047RegMag = 0x3FFD;
constexpr uint16_t AS5047RegAngleUnc = 0x3FFE;
constexpr uint16_t AS5047RegAngleCom = 0x3FFF;

constexpr uint16_t AS5047ReadCommand = 0x4000;

constexpr uint32_t AS54047ClockFrequency = 5000000;			// maximum is a little under 10MHz

// Convert nanoseconds to clock cycles
static inline constexpr uint32_t NanoSecondsToClocks(uint32_t ns) noexcept
{
	return ((SystemCoreClockFreq/1000000) * ns)/1000;
}

constexpr uint32_t Clocks350ns = NanoSecondsToClocks(350);
constexpr uint32_t ClocksHalfSclk = SystemCoreClockFreq/(2 * AS54047ClockFrequency);

// Adjust the top bit of a 16-bit word to make it even parity
static inline constexpr uint16_t AddParityBit(uint16_t w) noexcept
{
	uint16_t p = w ^ (w << 8);
	p ^= p << 4;
	p ^= p << 2;
	p ^= p << 1;
	return w ^ (p & 0x8000);
}

// Return true if the passed 16-bit word has even parity
static inline constexpr bool CheckEvenParity(uint16_t w) noexcept
{
	w ^= w >> 8;
	w ^= w >> 4;
	w ^= w >> 2;
	w ^= w >> 1;
	return (w & 1u) == 0;
}

// Check a response returning true if it is good
static inline bool CheckResponse(uint16_t response) noexcept
{
	return CheckEvenParity(response) && (response & 0x4000) == 0;
}

AS5047D::AS5047D(SharedSpiDevice& spiDev, Pin p_csPin) noexcept
	: SpiEncoder(spiDev, AS54047ClockFrequency, SpiMode::mode1, false, p_csPin),
	  AbsoluteEncoder()
{
}

// Initialise the encoder and enable it if successful. If there are any warnings or errors, put the corresponding message text in 'reply'.
GCodeResult AS5047D::Init(const StringRef& reply) noexcept
{
	// See if we can read sensible data from the encoder
	uint16_t responseDiag, responseErrfl;
	if (GetDiagnosticRegisters(responseDiag, responseErrfl))
	{
		if (CheckResponse(responseDiag))
		{
			if ((responseDiag & 0x0F00) == 0x100)
			{
				Enable();
				return GCodeResult::ok;
			}

			reply.copy("Encoder warning");
			if ((responseDiag & 0x0100) == 0)
			{
				reply.cat(": offset loop not ready");
			}
			if ((responseDiag & 0x0200) != 0)
			{
				reply.cat(": CORDIC overflow");
			}
			if ((responseDiag & 0x0400) != 0)
			{
				reply.cat(": magnet too strong");
			}
			if ((responseDiag & 0x0800) != 0)
			{
				reply.cat(": magnet too weak");
			}
			Enable();
			return GCodeResult::warning;
		}

		reply.copy("Bad response from encoder");
		return GCodeResult::error;
	}

	reply.copy("Failed to read encoder diagnostic registers");
	return GCodeResult::error;
}

void AS5047D::Enable() noexcept
{
	IoPort::SetPinMode(csPin, OUTPUT_HIGH);
	ClosedLoop::EnableEncodersSpi();
}

void AS5047D::Disable() noexcept
{
	IoPort::SetPinMode(csPin, OUTPUT_HIGH);
	ClosedLoop::DisableEncodersSpi();
}

uint32_t AS5047D::GetAbsolutePosition(bool& error) noexcept
{
	if (spi.Select(0))			// get the mutex and set the clock rate
	{
		uint16_t response;
		const bool ok = DoSpiTransaction(AddParityBit(AS5047ReadCommand | AS5047RegAngleCom), response)
					 && (DelayCycles(GetCurrentCycles(), Clocks350ns), 				// need at least 350ns CS high time
						 DoSpiTransaction(AddParityBit(AS5047ReadCommand | AS5047RegNop), response));
		spi.Deselect();			// release the mutex
		if (ok && CheckResponse(response))
		{
			response &= 0x3FFF;
			error = false;
			return ((response & 0x2000) ? response | 0xFFFFC000 : response) + AS5047D_ABS_READING_OFFSET;
		}
	}

	error = true;
	return 0;
}

// Get the diagnostic register and the error flags register
bool AS5047D::GetDiagnosticRegisters(uint16_t& diagReg, uint16_t& errFlags) noexcept
{
	if (spi.Select(0))			// get the mutex and set the clock rate
	{
		const bool ok = DoSpiTransaction(AddParityBit(AS5047ReadCommand | AS5047RegDiag), diagReg)
					 && (DelayCycles(GetCurrentCycles(), Clocks350ns), 				// need at least 350ns CS high time
						 DoSpiTransaction(AddParityBit(AS5047ReadCommand | AS5047RegErrfl), diagReg))
					 && (DelayCycles(GetCurrentCycles(), Clocks350ns), 				// need at least 350ns CS high time
						 DoSpiTransaction(AddParityBit(AS5047ReadCommand | AS5047RegNop), errFlags));
		spi.Deselect();			// release the mutex
		return ok;
	}

	return false;
}

// Get diagnostic information and append it to a string
void AS5047D::AppendDiagnostics(const StringRef &reply) noexcept
{
	reply.catf(", encoder full rotations %d", (int) fullRotations);
	reply.catf(", encoder last angle %d", (int) lastAngle);
	reply.catf(", zero crossing index=%d, zero crossing offset=%d", zeroCrossingIndex, zeroCrossingOffset);
	uint16_t responseDiag, responseErrfl;
	if (GetDiagnosticRegisters(responseDiag, responseErrfl))
	{
		if (CheckResponse(responseDiag))
		{
			reply.catf(", agc %u", responseDiag & 0x007F);
			if ((responseDiag & 0x0100) == 0)
			{
				reply.cat(", offset loop not ready");
			}
			if ((responseDiag & 0x0200) != 0)
			{
				reply.cat(", CORDIC overflow");
			}
			if ((responseDiag & 0x0400) != 0)
			{
				reply.cat(", magnet too strong");
			}
			if ((responseDiag & 0x0800) != 0)
			{
				reply.cat(", magnet too weak");
			}
		}
		else
		{
			reply.cat(", bad diag register response");
		}

		if (CheckResponse(responseErrfl))
		{
			if (responseErrfl & 0x01)
			{
				reply.cat(", framing error");
			}
			if (responseErrfl & 0x02)
			{
				reply.cat(", invalid command error");
			}
			if (responseErrfl & 0x04)
			{
				reply.cat(", parity error");
			}
			if ((responseErrfl & 0x07) == 0)
			{
				reply.cat(", no error");
			}
		}
		else
		{
			reply.cat(", bad errfl register response");
		}
	}
	else
	{
		reply.cat(", failed to read encoder diagnostics");
	}
}

// Perform an SPI transaction. Caller must get ownership of the SPI device first and release it afterwards, possibly after doing multiple transactions.
// Leave at least 350ns between multiple calls to this function.
bool AS5047D::DoSpiTransaction(uint16_t command, uint16_t &response) noexcept
{
	IoPort::WriteDigital(csPin, false);
	const uint8_t txBuffer[2] = { (uint8_t)(command >> 8), (uint8_t)(command & 0xFF) };
	uint8_t rxBuffer[2];
	const bool ok = spi.TransceivePacket(txBuffer, rxBuffer, 2);
	IoPort::WriteDigital(csPin, true);
	response = ((uint16_t)rxBuffer[0]) << 8 | rxBuffer[1];
	return ok;
}

#endif
