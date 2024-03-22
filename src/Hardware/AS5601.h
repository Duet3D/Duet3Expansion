/*
 * AS5601.h
 *
 *  Created on: 15 Jan 2024
 *      Author: David
 */

#ifndef SRC_HARDWARE_AS5601_H_
#define SRC_HARDWARE_AS5601_H_

#include <RepRapFirmware.h>

#if SUPPORT_AS5601

#include "SharedI2CClient.h"

// AS5601 magnetic rotary encoder class
class AS5601 : public SharedI2CClient
{
public:
	AS5601(SharedI2CMaster& dev) noexcept;

	bool Init() noexcept;													// initialise the device returning true if it was found
	bool Read(uint16_t& angle, uint8_t& status, uint8_t& agc) noexcept;		// read the raw angle, status and agc registers

	// Status register bits
	static constexpr uint8_t StatusMH = 1u << 3;
	static constexpr uint8_t StatusML = 1u << 4;
	static constexpr uint8_t StatusMD = 1u << 5;

private:
	enum class AS5601Register
	{
		// Configuration registers
		zmco = 0,
		zpos = 0x01,							// 16 bits
		config = 0x07,							// 16 bits
		abn = 0x09, pushthr,
		// Output registers
		rawAngle = 0x0C, angle = 0x0E,			// both 16 bits
		// Status registers
		status = 0x0B, agc = 0x1A,
		magnitude								// 16 bits
	};

	static constexpr uint32_t AS5601_I2CTimeout = 25;					// timeout in milliseconds when waiting to acquire the I2C bus

	bool Read8(AS5601Register reg, uint8_t& val, bool releaseBus = true) noexcept;
	bool Read16(AS5601Register reg, uint16_t& val, bool releaseBus = true) noexcept;
	bool Write8(AS5601Register reg, uint8_t val) noexcept;
	bool Write16(AS5601Register reg, uint16_t val) noexcept;
};

#endif

#endif /* SRC_HARDWARE_AS5601_H_ */
