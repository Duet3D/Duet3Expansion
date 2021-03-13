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

	SharedI2CMaster(uint8_t sercomNum);

	void SetClockFrequency(uint32_t freq) const;
	size_t Transfer(uint16_t address, uint8_t *buffer, size_t numToWrite, size_t numToRead) noexcept;
	ErrorCounts GetErrorCounts(bool clear) noexcept;

	bool Take(uint32_t timeout) { return mutex.Take(timeout); }					// get ownership of this SPI, return true if successful
	void Release() { mutex.Release(); }

private:
	void Enable() const;
	void Disable() const;
//	bool WaitForStatus(uint32_t statusBit, uint32_t& timeoutErrorCounter, WaitForStatusFunc statusWaitFunc) noexcept;
//	bool WaitTransferComplete(WaitForStatusFunc statusWaitFunc) noexcept;
//	bool WaitByteSent(WaitForStatusFunc statusWaitFunc) noexcept;
//	bool WaitByteReceived(WaitForStatusFunc statusWaitFunc) noexcept;
	size_t InternalTransfer(uint16_t address, uint8_t *buffer, size_t numToWrite, size_t numToRead) noexcept;

	Sercom * const hardware;
	Mutex mutex;
};

#endif

#endif /* SRC_HARDWARE_SHAREDI2CMASTER_H_ */
