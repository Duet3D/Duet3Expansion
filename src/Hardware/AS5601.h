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

// AS5601 class
class AS5601 : public SharedI2CClient
{
public:
	AS5601(SharedI2CMaster& dev) noexcept;

private:
	enum class AS5601Register
	{
		// Configuration registers
		zmco = 0, zposhi, zposlo, confhi = 7, conflo, abn, pushthr,
		// Output registers
		rawAngleHi = 0x0C, rawAngleLo, anleHo, angleLo,
		// Status registers
		status = 0x0B, agc = 0x1A, magHigh, magLo
	};

	static constexpr uint32_t AS5601_I2CTimeout = 25;					// timeout in milliseconds when waiting to acquire the I2C bus

	bool Read8(AS5601Register reg, uint8_t& val) noexcept;
	bool Read16(AS5601Register reg, uint16_t val) noexcept;
	bool Write8(AS5601Register reg, uint8_t val) noexcept;
	bool Write16(AS5601Register reg, uint16_t val) noexcept;
};

#endif

#endif /* SRC_HARDWARE_AS5601_H_ */
