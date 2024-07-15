/*
 * TCA6408A.h
 *
 *  Created on: 15 Jan 2024
 *      Author: David
 */

#ifndef SRC_HARDWARE_TCA6408A_H_
#define SRC_HARDWARE_TCA6408A_H_

#include <RepRapFirmware.h>

#if SUPPORT_TCA6408A

#include "SharedI2CClient.h"

// TCA6408A I2C expander class
class TCA6408A : public SharedI2CClient
{
public:
	TCA6408A(SharedI2CMaster& dev) noexcept;

	bool Init(uint8_t inputPins, uint8_t initialOutputs) noexcept;		// initialise the device returning true if it was found
	void SetOneOutputBitState(unsigned int bitnum, bool on) noexcept;	// set the state of one of the pins configured as an output - must call Poll afterwards to actually set the pin
	void SetOutputBitsState(uint8_t bitsToSet, uint8_t mask) noexcept;	// set the states of some bits in the output register - must call Poll afterwards to actually set the pin
	uint8_t GetInputRegister() noexcept { return inputRegister; }		// read the saved input register, which gets updated when Poll is called
	void Poll() noexcept;												// update the output and read the inputs

private:
	enum class TCA6408ARegister
	{
		input = 0, output, polarityInversion, config					// these are all 8-bit registers
	};

	static constexpr uint32_t TCA6408A_I2CTimeout = 25;					// timeout in milliseconds when waiting to acquire the I2C bus

	bool Read8(TCA6408ARegister reg, uint8_t& val) noexcept;
	bool Write8(TCA6408ARegister reg, uint8_t val) noexcept;

	uint8_t inputRegister;
	volatile uint8_t outputRegister;
	volatile bool outputNeedsUpdating = false;
};

#endif

#endif /* SRC_HARDWARE_TCA6408A_H_ */
