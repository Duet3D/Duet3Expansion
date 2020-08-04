/*
 * QuadratureDecoder.h
 *
 *  Created on: 23 May 2020
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_QUADRATUREENCODER_H_
#define SRC_CLOSEDLOOP_QUADRATUREENCODER_H_

#include <RepRapFirmware.h>

#if SUPPORT_CLOSED_LOOP

#include "SpiEncoder.h"

enum class AttinyProgErrorCode : uint8_t
{
	success = 0,
	spiBusy = 1,
	cantEnterProgrammingMode = 2,
	badDeviceId = 3,
	verifyFailed = 4,
	fuseVerifyFailed = 5,
	eraseTimeout = 6,
	writeTimeout = 7,
	fuseWriteTimeout = 8
};

class QuadratureEncoder : public SpiEncoder
{
public:
	QuadratureEncoder() noexcept;

	void Enable() noexcept override;				// Enable the decoder and reset the counter to zero. Won't work if the decoder has never been programmed.
	void Disable() noexcept override;				// Disable the decoder. Call this during initialisation. Can also be called later if necessary.
	int32_t GetReading() noexcept override;			// Get the 32-bit position
	void SetReading(int32_t pos) noexcept;			// Set the position. Call this after homing.

	static void TurnAttinyOff() noexcept;

private:
	AttinyProgErrorCode CheckProgram() noexcept;	// Check that the decoder is running current firmware, return true if yes
	AttinyProgErrorCode Program() noexcept;			// Update the program, return true if successful

	uint8_t SendSpiQuad(uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4) noexcept;
	AttinyProgErrorCode SetupForProgramming() noexcept;
	void EndProgramming() noexcept;
	bool WaitUntilAttinyReady() noexcept;
	AttinyProgErrorCode DoVerify() noexcept;

	uint32_t deviceSignature = 0;
	uint16_t counterLow, counterHigh;
};

#endif

#endif /* SRC_CLOSEDLOOP_QUADRATUREENCODER_H_ */
