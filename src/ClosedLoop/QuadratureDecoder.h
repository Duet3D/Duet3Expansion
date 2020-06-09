/*
 * QuadratureDecoder.h
 *
 *  Created on: 23 May 2020
 *      Author: David
 */

#ifndef SRC_CLOSEDLOOP_QUADRATUREDECODER_H_
#define SRC_CLOSEDLOOP_QUADRATUREDECODER_H_

#include <RepRapFirmware.h>

#if SUPPORT_CLOSED_LOOP

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

namespace QuadratureDecoder
{
	void Disable();							// Disable the decoder. Call this during initialisation. Can also be called later if necessary.
	void Enable();							// Enable the decoder and reset the counter to zero. Won't work if the decoder has never been programmed.
	AttinyProgErrorCode CheckProgram();		// Check that the decoder is running current firmware, return true if yes
	AttinyProgErrorCode Program();			// Update the program, return true if successful
	int32_t GetCounter();					// Get the 32-bit position
	void SetCounter(int32_t pos);			// Set the position. Call this after homing.
}

#endif

#endif /* SRC_CLOSEDLOOP_QUADRATUREDECODER_H_ */
