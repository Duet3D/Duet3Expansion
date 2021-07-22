/*
 * AttinyProgrammer.h
 *
 *  Created on: 10 Aug 2020
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_ATTINYPROGRAMMER_H_
#define SRC_CLOSEDLOOP_ATTINYPROGRAMMER_H_

#include <RepRapFirmware.h>

#if SUPPORT_CLOSED_LOOP && defined(EXP1HCE)

#include <Hardware/SharedSpiClient.h>

NamedEnum(AttinyProgErrorCode, uint8_t,
	notChecked,
	good,
	spiBusy ,
	cantEnterProgrammingMode ,
	badDeviceId,
	verifyFailed,
	fuseVerifyFailed,
	eraseTimeout,
	writeTimeout,
	fuseWriteTimeout
);

class AttinyProgrammer
{
public:
	AttinyProgrammer(SharedSpiDevice& spiDev) noexcept;

	void InitAttiny() noexcept;
	void TurnAttinyOff() noexcept;
	AttinyProgErrorCode GetProgramStatus() noexcept { return programStatus; }

private:
	AttinyProgErrorCode CheckProgram() noexcept;	// Check that the decoder is running current firmware, return true if yes
	AttinyProgErrorCode Program() noexcept;			// Update the program, return true if successful
	AttinyProgErrorCode DoVerify() noexcept;

	uint8_t SendSpiQuad(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4) noexcept;
	AttinyProgErrorCode SetupForProgramming() noexcept;
	void EndProgramming() noexcept;
	bool WaitUntilAttinyReady() noexcept;

	SharedSpiClient spi;
	uint32_t deviceSignature = 0;
	AttinyProgErrorCode programStatus;
};

#endif

#endif /* SRC_CLOSEDLOOP_ATTINYPROGRAMMER_H_ */
