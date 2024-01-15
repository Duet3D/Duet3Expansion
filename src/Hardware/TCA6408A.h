/*
 * TCA6408A.h
 *
 *  Created on: 15 Jan 2024
 *      Author: David
 */

#ifndef SRC_HARDWARE_TCA6408A_H_
#define SRC_HARDWARE_TCA6408A_H_

#include <RepRapFirmware.h>

#if SUPPORT_AS5601				// currently we only use the TCA6408A in conjunction with the AS5601

#include "SharedI2CClient.h"

// TCA6408A I2C expander class
class TCA6408A : public SharedI2CClient
{
public:
	TCA6408A(SharedI2CMaster& dev) noexcept;

	// Initialise the device
	void Init() noexcept;

	// Check whether the device was found during initialisation
	bool Present() const noexcept { return found; }

private:
	enum class TCA6408ARegister
	{
		input = 0, output, polarityInversion, config					// these are all 8-bit registers
	};

	static constexpr uint32_t TCA6408A_I2CTimeout = 25;					// timeout in milliseconds when waiting to acquire the I2C bus

	bool Read8(TCA6408ARegister reg, uint8_t& val) noexcept;
	bool Write8(TCA6408ARegister reg, uint8_t val) noexcept;

	bool found = false;
};

#endif

#endif /* SRC_HARDWARE_TCA6408A_H_ */
