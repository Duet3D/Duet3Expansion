/*
 * NeoPixelLedStrip.cpp
 *
 *  Created on: 30 Apr 2023
 *      Author: David
 */

#include <LedStrips/NeoPixelLedStrip.h>

#if SUPPORT_LED_STRIPS

#if SUPPORT_PIO_NEOPIXEL
WS2812* NeoPixelLedStrip::ws2812Device = nullptr;
#endif

NeoPixelLedStrip::NeoPixelLedStrip(bool p_isRGBW) noexcept
	: LocalLedStrip((p_isRGBW) ? LedStripType::NeoPixel_RGBW : LedStripType::NeoPixel_RGB, DefaultNeoPixelSpiClockFrequency),
	  isRGBW(p_isRGBW)
{
#if SUPPORT_PIO_NEOPIXEL
	// Create the PIO device if this is the first string
	if (ws2812Device == nullptr)
	{
		ws2812Device = new WS2812(NoPin, false, DmaChanWS2812);
	}
	useDma = true;
#endif
}

GCodeResult NeoPixelLedStrip::Configure(CanMessageGenericParser& parser, const StringRef& reply, uint8_t& extra) noexcept
{
	bool seen = false;
	GCodeResult rslt = CommonConfigure(parser, reply, seen, extra);
	if (seen)
	{
		// Nothing specific to configure for Neopixel strips in Duet3D builds
		return rslt;
	}

	return CommonReportDetails(reply);
}

GCodeResult NeoPixelLedStrip::HandleM150(CanMessageGenericParser& parser, const StringRef& reply) noexcept
{
#if SUPPORT_DMA_NEOPIXEL
	if (DmaInProgress())													// if we are sending something
	{
		return GCodeResult::notFinished;
	}
#endif

	if (needStartDelay && StepTimer::GetTimerTicks() - whenTransferFinished < MinNeoPixelResetTicks)
	{
		return GCodeResult::notFinished;									// give the NeoPixels time to reset
	}

	LedParams params;
	params.GetM150Params(parser);
	params.ApplyBrightness();

#if SUPPORT_DMA_NEOPIXEL
	if (UsesDma())
	{
		SpiSendData(params);
	}
	else
#elif SUPPORT_PIO_NEOPIXEL
	if (UsesDma())
	{
		PioSendData(params);
	}
	else
#endif
	{
		BitBangData(params);
	}
	return GCodeResult::ok;
}

// Return the number of buffer bytes we need per LED
size_t NeoPixelLedStrip::GetBytesPerLed() const noexcept
{
	const size_t bytesPerLed = (isRGBW) ? 4 : 3;
#if SUPPORT_PIO_NEOPIXEL
	// PIO strings always use 4 bytes per pixel
	return (useDma) ? 4 : bytesPerLed;
#else
	return (useDma) ? bytesPerLed * 4 : bytesPerLed;
#endif
}

#if SUPPORT_DMA_NEOPIXEL

// Encode one NeoPixel byte into the buffer.
// A 0 bit is encoded as 1000
// A 1 bit is encoded as 1110
// All encoding is MSB first
static void EncodeNeoPixelByte(uint8_t *p, uint8_t val) noexcept
{
	static constexpr uint8_t EncodedByte[4] = { 0b10001000, 0b10001110, 0b11101000, 0b11101110 };

# if SAME70 && USE_16BIT_SPI
	// Swap bytes for 16-bit DMA
	*p++ = EncodedByte[(val >> 4) & 3];
	*p++ = EncodedByte[val >> 6];
	*p++ = EncodedByte[val & 3];
	*p++ = EncodedByte[(val >> 2) & 3];
# else
	*p++ = EncodedByte[val >> 6];
	*p++ = EncodedByte[(val >> 4) & 3];
	*p++ = EncodedByte[(val >> 2) & 3];
	*p++ = EncodedByte[val & 3];
# endif
}

// Send data to NeoPixel LEDs by DMA to SPI
GCodeResult NeoPixelLedStrip::SpiSendData(const LedParams& params) noexcept
{
	const unsigned int bytesPerLed = (isRGBW) ? 16 : 12;
	unsigned int numLeds = params.numLeds;
	uint8_t *p = chunkBuffer + (bytesPerLed * numAlreadyInBuffer);
	while (numLeds != 0 && p + bytesPerLed <= chunkBuffer + chunkBufferSize)
	{
		EncodeNeoPixelByte(p, (uint8_t)params.green);
		p += 4;
		EncodeNeoPixelByte(p, (uint8_t)params.red);
		p += 4;
		EncodeNeoPixelByte(p, (uint8_t)params.blue);
		p += 4;
		if (isRGBW)
		{
			EncodeNeoPixelByte(p, (uint8_t)params.white);
			p += 4;
		}
		--numLeds;
		++numAlreadyInBuffer;
	}

	if (!params.following)
	{
		DmaSendChunkBuffer(bytesPerLed * numAlreadyInBuffer);		// send data by DMA to SPI
		numAlreadyInBuffer = 0;
		needStartDelay = true;
	}
	return GCodeResult::ok;
}

#elif SUPPORT_PIO_NEOPIXEL

// Send data to NeoPixel LEDs by PIO
GCodeResult NeoPixelLedStrip::PioSendData(const LedParams& params) noexcept
{
	const unsigned int bytesPerLed = 4;
	unsigned int numLeds = params.numLeds;
	uint8_t *p = chunkBuffer + (bytesPerLed * numAlreadyInBuffer);
	while (numLeds != 0 && p + bytesPerLed <= chunkBuffer + chunkBufferSize)
	{
		*p++ = isRGBW ? (uint8_t)params.white : 0;
		*p++ = (uint8_t)params.blue;
		*p++ = (uint8_t)params.red;
		*p++ = (uint8_t)params.green;
		--numLeds;
		++numAlreadyInBuffer;
	}
	if (!params.following)
	{
		if (ws2812Device != nullptr)
		{
			ws2812Device->Configure(port.GetPin(), isRGBW);
			ws2812Device->SendData((uint32_t *)chunkBuffer, numAlreadyInBuffer);
		}
		numAlreadyInBuffer = 0;
		needStartDelay = true;
	}
	return GCodeResult::ok;

}

#endif

// Bit bang data to Neopixels
constexpr uint32_t NanosecondsToCycles(uint32_t ns) noexcept
{
	return (ns * (uint64_t)SystemCoreClockFreq)/1000000000u;
}

// Send data to NeoPixel LEDs by bit banging
GCodeResult NeoPixelLedStrip::BitBangData(const LedParams& params) noexcept
{
	unsigned int numLeds = params.numLeds;
	const unsigned int bytesPerLed = (isRGBW) ? 4 : 3;
	uint8_t *p = chunkBuffer + (bytesPerLed * numAlreadyInBuffer);
	while (numLeds != 0 && p + bytesPerLed <= chunkBuffer + chunkBufferSize)
	{
		*p++ = (uint8_t)params.green;
		*p++ = (uint8_t)params.red;
		*p++ = (uint8_t)params.blue;
		if (isRGBW)
		{
			*p++ = (uint8_t)params.white;
		}
		--numLeds;
		++numAlreadyInBuffer;
	}

	if (!params.following)
	{
		const uint8_t *q = chunkBuffer;
		if (q < p)
		{
			const Pin pin = port.GetPin();
			if (port.GetTotalInvert())
			{
				BitBangDataInverted(q, p, pin);
			}
			else
			{
				BitBangDataNormal(q, p, pin);
			}
		}

		numAlreadyInBuffer = 0;
		whenTransferFinished = StepTimer::GetTimerTicks();
		needStartDelay = true;
	}
	return GCodeResult::ok;
}

constexpr uint32_t T0H = NanosecondsToCycles(300);
constexpr uint32_t T1H = NanosecondsToCycles(700);
constexpr uint32_t TCY = NanosecondsToCycles(1200);

#if SAMC21
// When bit-banging Neopixels we can't afford to wait for instructions to be fetched from flash memory
[[gnu::optimize("O2")]] __attribute__((section(".time_critical")))
#elif RP2040
// When bit-banging Neopixels we can't afford to wait for instructions to be fetched from flash memory
[[gnu::optimize("O3")]] __attribute__((section(".time_critical")))
#endif
void NeoPixelLedStrip::BitBangDataNormal(const uint8_t *start, const uint8_t *end, Pin pin) noexcept
{
	uint32_t nextDelay = TCY;
	IrqDisable();
#if !SAMC21
	uint32_t lastTransitionTime = GetCurrentCycles();
#endif
	fastDigitalWriteLow(pin);			// this is needed on Duet 2 at least, to prevent the first pulse being too long
	do
	{
		uint8_t c = *start++;
		for (unsigned int i = 0; i < 8; ++i)
		{
			// The high-level time is critical, the low-level time is not.
			// On the SAME5x the high-level time easily gets extended too much, so do as little work as possible during that time.
			const uint32_t diff = ((c >> 7u) - 1u) & (T1H - T0H);
			uint32_t highTime = T1H - diff;
#if SAMC21
			// SAMC21 is too slow to use DelayCycles for timings <1us or even an inlined version of it for timings <500ns
			while (nextDelay > 5)
			{
				asm volatile ("nop");
				nextDelay -= 5;
			}
			nextDelay = TCY - highTime;
			fastDigitalWriteHigh(pin);
			while (highTime > 5)
			{
				__asm volatile("nop");
				highTime -= 5;
			}
			fastDigitalWriteLow(pin);
#else
			lastTransitionTime = DelayCycles(lastTransitionTime, nextDelay);
			fastDigitalWriteHigh(pin);
			lastTransitionTime = DelayCycles(lastTransitionTime, highTime);
			fastDigitalWriteLow(pin);
			nextDelay = TCY - highTime;
#endif
			c <<= 1;
		}
	}
	while (start < end);
	IrqEnable();
}

#if SAMC21
// When bit-banging Neopixels we can't afford to wait for instructions to be fetched from flash memory
[[gnu::optimize("O2")]] __attribute__((section(".time_critical")))
#elif RP2040
// When bit-banging Neopixels we can't afford to wait for instructions to be fetched from flash memory
[[gnu::optimize("O3")]] __attribute__((section(".time_critical")))
#endif
void NeoPixelLedStrip::BitBangDataInverted(const uint8_t *start, const uint8_t *end, Pin pin) noexcept
{
	uint32_t nextDelay = TCY;
	IrqDisable();
#if !SAMC21
	uint32_t lastTransitionTime = GetCurrentCycles();
#endif
	fastDigitalWriteHigh(pin);
	do
	{
		uint8_t c = *start++;
		for (unsigned int i = 0; i < 8; ++i)
		{
			// The high-level time is critical, the low-level time is not.
			// On the SAME5x the high-level time easily gets extended too much, so do as little work as possible during that time.
			const uint32_t diff = ((c >> 7u) - 1u) & (T1H - T0H);
			uint32_t highTime = T1H - diff;
#if SAMC21
			// SAMC21 is too slow to use DelayCycles for timings <1us or even an inlined version of it for timings <500ns
			while (nextDelay > 5)
			{
				asm volatile ("nop");
				nextDelay -= 5;
			}
			nextDelay = TCY - highTime;
			fastDigitalWriteLow(pin);
			while (highTime > 5)
			{
				asm volatile("nop");
				highTime -= 5;
			}
			fastDigitalWriteHigh(pin);
#else
			lastTransitionTime = DelayCycles(lastTransitionTime, nextDelay);
			fastDigitalWriteLow(pin);
			lastTransitionTime = DelayCycles(lastTransitionTime, highTime);
			fastDigitalWriteHigh(pin);
			nextDelay = TCY - highTime;
#endif
			c <<= 1;
		}
	}
	while (start < end);
	IrqEnable();
}

#endif

// End
