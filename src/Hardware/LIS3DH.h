/*
 * LIS3DH.h
 *
 *  Created on: 14 Mar 2021
 *      Author: David
 */

#ifndef SRC_HARDWARE_LIS3DH_H_
#define SRC_HARDWARE_LIS3DH_H_

#include <RepRapFirmware.h>

#if SUPPORT_I2C_SENSORS && SUPPORT_LIS3DH

#include "SharedI2CClient.h"

class LIS3DH : public SharedI2CClient
{
public:
	LIS3DH(SharedI2CMaster& dev, bool addressLSB) noexcept;

	bool CheckPresent() noexcept;

private:
	static constexpr uint8_t WhoAmIRegister = 0x0F;
	static constexpr uint8_t WhoAmIValue = 0x33;

	struct AuxRegisters
	{
		uint8_t statusAux;
		uint8_t outAdc1L;
		uint8_t outAdc1H;
		uint8_t outAdc2L;
		uint8_t outAdc2H;
		uint8_t outAdc3L;
		uint8_t outAdc3H;
		static constexpr uint8_t FirstRegNum = 0x07;
	} auxRegisters;

	struct CtrlRegisters
	{
		uint8_t ctrlReg0;
		uint8_t tempCfgReg;
		uint8_t ctrlReg1;
		uint8_t ctrlReg2;
		uint8_t ctrlReg3;
		uint8_t ctrlReg4;
		uint8_t ctrlReg5;
		uint8_t ctrlReg6;
		static constexpr uint8_t FirstRegNum = 0x1E;
	} ctrlRegisters;

	struct OutRegisters
	{
		uint8_t status;
		uint8_t outXL;
		uint8_t outXH;
		uint8_t outYL;
		uint8_t outYH;
		uint8_t outZL;
		uint8_t outZH;
		static constexpr uint8_t FirstRegNum = 0x27;
	};

	template<class T> bool ReadRegisters(T& registers) noexcept;
	template<class T> bool WriteRegisters(const T& registers) noexcept;
	bool ReadRegister(uint8_t regNum, uint8_t& val) noexcept;
	bool WriteRegister(uint8_t regNum, uint8_t val) noexcept;
};

#endif

#endif /* SRC_HARDWARE_LIS3DH_H_ */
