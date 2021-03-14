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
	struct ErrorCounts
	{
		uint32_t naks;
		uint32_t sendTimeouts;
		uint32_t recvTimeouts;
		uint32_t finishTimeouts;
		uint32_t resets;

		void Clear() noexcept;
	};

	SharedI2CMaster(uint8_t sercomNum) noexcept;

	void SetClockFrequency(uint32_t freq) const noexcept;
	bool Transfer(uint16_t address, uint8_t firstByte, uint8_t *buffer, size_t numToWrite, size_t numToRead) noexcept;
	ErrorCounts GetErrorCounts(bool clear) noexcept;

	bool Take(uint32_t timeout) noexcept { return mutex.Take(timeout); }		// get ownership of this SPI, return true if successful
	void Release() noexcept { mutex.Release(); }

	void Interrupt() noexcept;

private:
	void Enable() const noexcept;
	void Disable() const noexcept;
	bool WaitForStatus(uint8_t statusBits) noexcept;
	size_t InternalTransfer(uint16_t address, uint8_t firstByte, uint8_t *buffer, size_t numToWrite, size_t numToRead) noexcept;

	Sercom * const hardware;
	TaskHandle taskWaiting;
	ErrorCounts errorCounts;			// error counts
	Mutex mutex;
};

#endif

#endif /* SRC_HARDWARE_SHAREDI2CMASTER_H_ */
