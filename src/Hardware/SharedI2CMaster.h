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

	void SetClockFrequency(uint32_t freq) const noexcept;
	bool Transfer(uint16_t address, uint8_t firstByte, uint8_t *buffer, size_t numToWrite, size_t numToRead) noexcept;

	bool Take(uint32_t timeout) noexcept { return mutex.Take(timeout); }		// get ownership of this SPI, return true if successful
	void Release() noexcept { mutex.Release(); }

	void Diagnostics(const StringRef& reply) noexcept;

	void Interrupt() noexcept;

private:
	enum class I2cState : uint8_t
	{
		idle = 0, sendingAddressForWrite, writing, sendingTenBitAddressForRead, sendingAddressForRead, reading, protocolError
	};

	void Enable() const noexcept;
	void Disable() const noexcept;
	bool InternalTransfer(uint16_t address, uint8_t firstByte, uint8_t *buffer, size_t numToWrite, size_t numToRead) noexcept;
	void ProtocolError()  noexcept;

#if RP2040
	//TODO
#else
	Sercom * const hardware;
#endif

	TaskHandle taskWaiting;
	Mutex mutex;

	uint8_t *transferBuffer;
	size_t numLeftToRead, numLeftToWrite;
	uint16_t currentAddress;
	unsigned int busErrors, naks, otherErrors;
	uint8_t firstByteToWrite;
	volatile I2cState state;
};

#endif

#endif /* SRC_HARDWARE_SHAREDI2CMASTER_H_ */
