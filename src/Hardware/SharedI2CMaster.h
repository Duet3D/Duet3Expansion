/*
 * SharedI2CMaster.h
 *
 *  Created on: 13 Mar 2021
 *      Author: David
 */

#ifndef SRC_HARDWARE_SHAREDI2CMASTER_H_
#define SRC_HARDWARE_SHAREDI2CMASTER_H_

#include <RepRapFirmware.h>

#if SUPPORT_I2C_SENSORS

#include <RTOSIface/RTOSIface.h>

class SharedI2CMaster
{
public:
	SharedI2CMaster(uint8_t sercomNum) noexcept;

	void SetClockFrequency(uint32_t freq) noexcept;
	bool Transfer(uint16_t address, const uint8_t *txBuffer, uint8_t *rxBuffer, size_t numToWrite, size_t numToRead) noexcept;

	bool Take(uint32_t timeout) noexcept;		// get ownership of this I2C interface, return true if successful
	void Release() noexcept;

	void Diagnostics(const StringRef& reply) noexcept;

	void Interrupt() noexcept;

private:
	enum class I2cState : uint8_t
	{
		idle = 0, writing, sendingTenBitAddressForRead, reading, protocolError
	};

	void Enable() const noexcept;
	void Disable() const noexcept;
	bool InternalTransfer(uint16_t address, const uint8_t *txBuffer, uint8_t *rxBuffer, size_t numToWrite, size_t numToRead) noexcept;
	void ProtocolError()  noexcept;

#if RP2040
	//TODO
#else
	Sercom * const hardware;
#endif

	TaskHandle taskWaiting;
	Mutex mutex;

	uint32_t currentClockRate;
	const uint8_t *txTransferBuffer;
	uint8_t *rxTransferBuffer;
	size_t numLeftToRead, numLeftToWrite;
	unsigned int busErrors, naks, contentions, otherErrors;
	uint16_t currentAddress;
	volatile I2cState state;
};

#endif

#endif /* SRC_HARDWARE_SHAREDI2CMASTER_H_ */
