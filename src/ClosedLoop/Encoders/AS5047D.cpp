/*
 * AS5047D.cpp
 *
 *  Created on: 10 Jun 2020
 *      Author: David
 */

#include "AS5047D.h"

#if SUPPORT_CLOSED_LOOP

#include <Hardware/IoPorts.h>
#include <ClosedLoop/ClosedLoop.h>

constexpr unsigned int AS5047DResolutionBits = 14;

constexpr uint16_t AS5047RegNop = 0x0000;
constexpr uint16_t AS5047RegErrfl = 0x0001;
constexpr uint16_t AS5047RegProg = 0x0003;
constexpr uint16_t AS5047RegDiag = 0x3FFC;
constexpr uint16_t AS5047RegMag = 0x3FFD;
constexpr uint16_t AS5047RegAngleUnc = 0x3FFE;
constexpr uint16_t AS5047RegAngleCom = 0x3FFF;

constexpr uint16_t AS5047ReadCommand = 0x4000;

constexpr uint32_t AS5047ClockFrequency = 5000000;			// maximum is a little under 10MHz

// Convert nanoseconds to clock cycles
static inline constexpr uint32_t NanoSecondsToClocks(uint32_t ns) noexcept
{
	return ((SystemCoreClockFreq/1000000) * ns)/1000;
}

constexpr uint32_t Clocks350ns = NanoSecondsToClocks(350);
constexpr uint32_t ClocksHalfSclk = SystemCoreClockFreq/(2 * AS5047ClockFrequency);

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

AS5047D::AS5047D(uint32_t p_stepsPerRev, SharedSpiDevice& spiDev, Pin p_csPin) noexcept
	: SpiEncoder(spiDev, AS5047ClockFrequency, SpiMode::mode1, false, p_csPin),
	  AbsoluteRotaryEncoder(p_stepsPerRev, AS5047DResolutionBits)
{
}

// Initialise the encoder and enable it if successful. If there are any warnings or errors, put the corresponding message text in 'reply'.
GCodeResult AS5047D::Init(const StringRef& reply) noexcept
{
	// See if we can read sensible data from the encoder
	DiagnosticRegisters regs;
	if (GetDiagnosticRegisters(regs))
	{
		if (CheckResponse(regs.diag))
		{
			if ((regs.diag & 0x0F00) == 0x100)
			{
				Enable();
				return GCodeResult::ok;
			}

			reply.copy("Encoder warning");
			if ((regs.diag & 0x0100) == 0)
			{
				reply.cat(": offset loop not ready");
			}
			if ((regs.diag & 0x0200) != 0)
			{
				reply.cat(": CORDIC overflow");
			}
			if ((regs.diag & 0x0400) != 0)
			{
				reply.cat(": magnet too strong");
			}
			if ((regs.diag & 0x0800) != 0)
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

// Return the current position as reported by the encoder
bool AS5047D::GetRawReading() noexcept
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
			rawReading = response & 0x3FFF;
			return false;
		}
	}

	return true;
}

// Get the diagnostic register and the error flags register
bool AS5047D::GetDiagnosticRegisters(DiagnosticRegisters& regs) noexcept
{
	if (spi.Select(0))			// get the mutex and set the clock rate
	{
		const bool ok = DoSpiTransaction(AddParityBit(AS5047ReadCommand | AS5047RegDiag), regs.diag)
					 && (DelayCycles(GetCurrentCycles(), Clocks350ns), 				// need at least 350ns CS high time
						 DoSpiTransaction(AddParityBit(AS5047ReadCommand | AS5047RegMag), regs.diag))
					 && (DelayCycles(GetCurrentCycles(), Clocks350ns), 				// need at least 350ns CS high time
						 DoSpiTransaction(AddParityBit(AS5047ReadCommand | AS5047RegErrfl), regs.mag))
					 && (DelayCycles(GetCurrentCycles(), Clocks350ns), 				// need at least 350ns CS high time
						 DoSpiTransaction(AddParityBit(AS5047ReadCommand | AS5047RegNop), regs.errFlags));
		spi.Deselect();			// release the mutex
		return ok;
	}

	return false;
}

// Get diagnostic information and append it to a string
void AS5047D::AppendDiagnostics(const StringRef &reply) noexcept
{
	reply.catf("Encoder reverse polarity: %s", (IsReversed()) ? "yes" : "no");
	reply.catf(", full rotations %" PRIi32, fullRotations);
	reply.catf(", last angle %" PRIu32, currentAngle);
	reply.catf(", minCorrection=%.1f, maxCorrection=%.1f", (double)minLUTCorrection, (double)maxLUTCorrection);
	DiagnosticRegisters regs;
	if (GetDiagnosticRegisters(regs))
	{
		if (CheckResponse(regs.diag))
		{
			reply.catf(", agc %u", regs.diag & 0x007F);
			if ((regs.diag & 0x0100) == 0)
			{
				reply.cat(", offset loop not ready");
			}
			if ((regs.diag & 0x0200) != 0)
			{
				reply.cat(", CORDIC overflow");
			}
			if ((regs.diag & 0x0400) != 0)
			{
				reply.cat(", magnet too strong");
			}
			if ((regs.diag & 0x0800) != 0)
			{
				reply.cat(", magnet too weak");
			}
		}
		else
		{
			reply.cat(", bad diag register response");
		}

		if (CheckResponse(regs.mag))
		{
			reply.catf(", mag %u", regs.mag & 0x3FFF);
		}
		else
		{
			reply.cat(", bad mag register response");
		}

		if (CheckResponse(regs.errFlags))
		{
			if (regs.errFlags & 0x01)
			{
				reply.cat(", framing error");
			}
			if (regs.errFlags & 0x02)
			{
				reply.cat(", invalid command error");
			}
			if (regs.errFlags & 0x04)
			{
				reply.cat(", parity error");
			}
			if ((regs.errFlags & 0x07) == 0)
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

// Append short form status to a string. If there is an error then the user can use M122 to get more details.
void AS5047D::AppendStatus(const StringRef& reply) noexcept
{
	reply.lcatf("Magnetic encoder motor steps/rev %" PRIu32, stepsPerRev);
	DiagnosticRegisters regs;
	if (GetDiagnosticRegisters(regs))
	{
		if (CheckResponse(regs.diag) && CheckResponse(regs.mag) && (regs.diag & 0x0F00) == 0x0100)
		{
			reply.catf(", agc %u, mag %u", regs.diag & 0x007F, regs.mag & 0x3FFF);
		}
		else
		{
			reply.cat(", sensor error");
		}
	}
	else
	{
		reply.cat(", sensor comm error");
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
