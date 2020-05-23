/*
 * QuadratureDecoder.cpp
 *
 *  Created on: 23 May 2020
 *      Author: David
 */

#include "QuadratureDecoder.h"

#if SUPPORT_CLOSED_LOOP

#include <Hardware/IoPorts.h>
#include <Hardware/SharedSpiDevice.h>

static volatile uint16_t counterHighWord;
static SharedSpiDevice spiDev(1000000, SpiMode::mode0, false);

// Set up the SPI channel and put the attiny in programming mode
static void SetupForProgramming()
{

	//TODO
}

// Disable the decoder. Call this during initialisation. Can also be called later if necessary.
void QuadratureDecoder::Disable()
{
	IoPort::SetPinMode(AttinyResetPin, OUTPUT_LOW);
}

// Enable the decoder and reset the counter to zero. Won't work if the decoder has never been programmed.
void QuadratureDecoder::Enable()
{
	//TODO disable the SPI channel
	SetCounter(0);
	digitalWrite(AttinyResetPin, true);
}

// Check that the decoder is running current firmware, return true if yes
bool QuadratureDecoder::CheckProgram()
{
	SetupForProgramming();
	//TODO
	return false;
}

// Update the program, return true if successful
bool QuadratureDecoder::Program()
{
	SetupForProgramming();
	//TODO
	return false;
}

// Get the 32-bit position
int32_t QuadratureDecoder::GetCounter()
{
	uint16_t highWord;
	uint32_t lowWord;
	do
	{
		highWord = counterHighWord;
		QuadratureTcc->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC_Val;
		while (QuadratureTcc->SYNCBUSY.bit.COUNT) { }
		lowWord = QuadratureTcc->COUNT.reg & 0xFFFF;
	} while (highWord != counterHighWord);
	return lowWord | ((uint32_t)highWord << 16);
}

// Set the position. Call this after homing.
void QuadratureDecoder::SetCounter(int32_t pos)
{
	const uint32_t upos = (uint32_t)pos;
	// In case of pulses arriving from the encoder, we may need to set this more than one
	do
	{
		QuadratureTcc->COUNT.reg = upos;
		counterHighWord = (uint16_t)(upos >> 16);
	} while (GetCounter() != pos);
}

#endif

// End
