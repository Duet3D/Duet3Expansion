/*
 * ADS131M02.h
 *
 *  Created on: 3 Jan 2026
 *      Author: David
 */

#ifndef SRC_HARDWARE_DRIVERS_ADS131M02_H_
#define SRC_HARDWARE_DRIVERS_ADS131M02_H_

#include <RepRapFirmware.h>

#if SUPPORT_ADS131M02

#include <SPI/SpiDevice.h>
#include <InputMonitors/InputMonitor.h>

class ADS131M02 : public SpiDevice
{
public:
	ADS131M02() noexcept;
	bool DeviceOk() const noexcept { return initOk; }
	int32_t GetLatestData() const noexcept { return compositeData; }			// no lock needed because 32-bit data access is atomic
	bool Activate(InputMonitor& monitor) noexcept;
	void Deactivate() noexcept;

	[[noreturn]] void TaskLoop() noexcept;

private:
	static constexpr uint32_t TaskStackWords = 200;			//TODO optimise this

	enum class Ads131M02Register : uint8_t
	{
		id = 0x00, status, mode, clock, gain,
		cfg =0x06, thrshols_msw, thrshold_lsw,
		ch0_cfg = 0x09, ch0_ocal_msw, ch0_ocal_lsw, ch0_gcal_msw, ch0_gcal_lsw,
		ch1_cfg = 0x0e, ch1_ocal_msw, ch1_ocal_lsw, ch1_gcal_msw, ch1_gcal_lsw,
		regmap_crc = 0x3e, reserved
	};

	enum class Ads131M02Command : uint16_t
	{
		nullCmd = 0,
		reset = 0x0011,
		standby = 0x0022,
		wakeup = 0x0033,
		lock = 0x0555,
		unlock = 0x0655,
		rreg = 0xA000,			// need to or-in the register number
		wreg = 0x6000			// need to or-in he register number
	};

	bool SendSimpleCommand(Ads131M02Command cmd) noexcept;						// send a simple command, not a read or write register
	bool ReadRegister(Ads131M02Register regNum) noexcept;						// send the command to read a 16-bit register, need to do another transfer to actually read it
	bool WriteRegister(Ads131M02Register regNum, uint16_t val) noexcept;		// Write a 16-bit register

	uint8_t regWriteBuffer[3 * 4];							// we always transfer four 24-bit words
	uint8_t regReadBuffer[3 * 4];							// we always transfer four 24-bit words

	uint16_t rslt;											// the result of the previous command
	uint32_t channel0Data;
	uint32_t channel1Data;
	int32_t compositeData = 0;

	InputMonitor *volatile inputMonitor = nullptr;			// when the load cell is in use this points to the associated input monitor, otherwise it is null

	Task<TaskStackWords> *adcTask = nullptr;
	bool initOk = false;
};

#endif

#endif /* SRC_HARDWARE_DRIVERS_ADS131M02_H_ */
