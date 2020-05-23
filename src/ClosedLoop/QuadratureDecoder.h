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

namespace QuadratureDecoder
{
	void Disable();					// Disable the decoder. Call this during initialisation. Can also be called later if necessary.
	void Enable();					// Enable the decoder and reset the counter to zero. Won't work if the decoder has never been programmed.
	bool CheckProgram();			// Check that the decoder is running current firmware, return true if yes
	bool Program();					// Update the program, return true if successful
	int32_t GetCounter();			// Get the 32-bit position
	void SetCounter(int32_t pos);	// Set the position. Call this after homing.
}

#endif

#endif /* SRC_CLOSEDLOOP_QUADRATUREDECODER_H_ */
